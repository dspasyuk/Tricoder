#include "config.h"
#include "DFRobot_AS7341.h"
#include <ArduinoJson.h>

// Scan timing
unsigned long scanStartTime = 0;
unsigned long scanElapsedTime = 0;

// Global object definitions
WebServer server(80);
Preferences preferences;
DFRobot_AS7341 as7341;

// WiFi configuration
String ssid = "";
String password = "";
bool apMode = false;

// Data collection variables
bool collecting = false;
unsigned int accumulationTime = 1; // seconds
unsigned int scanInterval = 2; // seconds
const int interpolationPoints = 100; // Number of points for the smooth curve

// AS7341 channel center wavelengths (nm)
const float channelWavelengths[10] = {
  415, 445, 480, 515, 555, 590, 630, 680, 910, 550
};
const char* channelNames[10] = {
  "F1 (415nm)", "F2 (445nm)", "F3 (480nm)", "F4 (515nm)", 
  "F5 (555nm)", "F6 (590nm)", "F7 (630nm)", "F8 (680nm)", 
  "NIR (910nm)", "Clear"
};

// Store raw and normalized readings
float normalizedReadings[10] = {0};
float interpolatedSpectrum[interpolationPoints][2]; // Array to store [wavelength, intensity]

DFRobot_AS7341::sModeOneData_t data1;
DFRobot_AS7341::sModeTwoData_t data2;

// Cubic interpolation for smoother curves (Catmull-Rom spline)
float cubicInterpolate(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t;
  float t3 = t2 * t;
  float a0 = -0.5*p0 + 1.5*p1 - 1.5*p2 + 0.5*p3;
  float a1 = p0 - 2.5*p1 + 2*p2 - 0.5*p3;
  float a2 = -0.5*p0 + 0.5*p2;
  float a3 = p1;
  return a0*t3 + a1*t2 + a2*t + a3;
}

// Read and accumulate sensor data
void readAndAccumulate() {
  long accumulatedReadings[10] = {0};
  int readingCount = 0;
  unsigned long startTime = millis();

  Serial.println("Starting accumulation...");

  while (millis() - startTime < accumulationTime * 1000) {
    as7341.startMeasure(as7341.eF1F4ClearNIR);
    data1 = as7341.readSpectralDataOne();
    
    as7341.startMeasure(as7341.eF5F8ClearNIR);
    data2 = as7341.readSpectralDataTwo();

    accumulatedReadings[0] += data1.ADF1;
    accumulatedReadings[1] += data1.ADF2;
    accumulatedReadings[2] += data1.ADF3;
    accumulatedReadings[3] += data1.ADF4;
    accumulatedReadings[4] += data2.ADF5;
    accumulatedReadings[5] += data2.ADF6;
    accumulatedReadings[6] += data2.ADF7;
    accumulatedReadings[7] += data2.ADF8;
    accumulatedReadings[8] += data1.ADNIR;
    accumulatedReadings[9] += data1.ADCLEAR;
    
    readingCount++;
    delay(50); // Small delay between readings
  }

  Serial.print("Accumulated ");
  Serial.print(readingCount);
  Serial.println(" readings.");

  // Average the readings from all 10 channels
  float avgReadings[10];
  for (int i = 0; i < 10; i++) {
    avgReadings[i] = (readingCount > 0) ? (float)accumulatedReadings[i] / readingCount : 0;
    normalizedReadings[i] = avgReadings[i]; // Directly assign without normalization
  }
}

// Get interpolated value at a specific wavelength
float getInterpolatedValue(float wavelength) {
    // Use an array of the 8 visible channels for interpolation
    const float visWavelengths[] = {415, 445, 480, 515, 555, 590, 630, 680};
    const int numVisChannels = 8;

    if (wavelength <= visWavelengths[0]) return normalizedReadings[0];
    if (wavelength >= visWavelengths[numVisChannels - 1]) return normalizedReadings[numVisChannels - 1];

    int lowerIdx = 0;
    for (int i = 0; i < numVisChannels - 1; i++) {
        if (wavelength >= visWavelengths[i] && wavelength <= visWavelengths[i + 1]) {
            lowerIdx = i;
            break;
        }
    }

    float t = (wavelength - visWavelengths[lowerIdx]) / (visWavelengths[lowerIdx + 1] - visWavelengths[lowerIdx]);

    float p0 = (lowerIdx > 0) ? normalizedReadings[lowerIdx - 1] : normalizedReadings[lowerIdx];
    float p1 = normalizedReadings[lowerIdx];
    float p2 = normalizedReadings[lowerIdx + 1];
    float p3 = (lowerIdx < numVisChannels - 2) ? normalizedReadings[lowerIdx + 2] : normalizedReadings[lowerIdx + 1];

    return cubicInterpolate(p0, p1, p2, p3, t);
}

// Generate the interpolated spectrum
void generateInterpolatedSpectrum() {
  float minWavelength = 415;
  float maxWavelength = 680;
  float step = (maxWavelength - minWavelength) / (interpolationPoints - 1);

  for (int i = 0; i < interpolationPoints; i++) {
    float wavelength = minWavelength + i * step;
    interpolatedSpectrum[i][0] = wavelength;
    float intensity = getInterpolatedValue(wavelength);
    interpolatedSpectrum[i][1] = max(0.0f, intensity);
  }
}

void handleStatus() {
  StaticJsonDocument<200> doc;
  doc["startTime"] = scanStartTime;
  doc["elapsedTime"] = scanElapsedTime;
  doc["heap"] = ESP.getFreeHeap();
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleLight() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      as7341.enableLed(true);
      //as7341.setLedCurrent(10);
      server.send(200, "text/plain", "Light ON");
    } else {
      as7341.enableLed(false);
      server.send(200, "text/plain", "Light OFF");
    }
  } else {
    server.send(400, "text/plain", "Missing state parameter");
  }
}

void setup() {
  Serial.begin(115200);

  while (as7341.begin() != 0) {
    Serial.println("Could not find AS7341 sensor! Check wiring.");
    delay(3000);
  }
  Serial.println("AS7341 sensor found!");

  as7341.setAtime(100);
  as7341.setAGAIN(128);
  as7341.setAstep(999);
  as7341.enableSpectralMeasure(true);

  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  preferences.end();

  if (ssid.length() > 0) {
    Serial.println("Attempting to connect to saved WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
  }

  if (WiFi.status() != WL_CONNECTED) {
    startAPMode();
  } else {
    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    apMode = false;
  }

  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save-wifi", HTTP_POST, handleSaveWifi);
  server.on("/start", handleStart);
  server.on("/data", handleData);
  server.on("/getconfig", handleGetConfig);
  server.on("/setconfig", HTTP_POST, handleSetConfig);
  server.on("/reset-wifi", handleResetWifi);
  server.on("/status", handleStatus);
  server.on("/light", handleLight);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
