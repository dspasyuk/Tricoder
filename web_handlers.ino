#include "config.h"
#include "html_pages.h"

void startAPMode() {
  Serial.println("Starting Access Point mode...");
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Tricorder", "12345678");
  Serial.println("AP Started");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
}

void handleRoot() {
  if (apMode) {
    server.send_P(200, "text/html", CONFIG_PAGE);
  } else {
    String html = String(MAIN_PAGE_START) + ssid + String(MAIN_PAGE_END);
    server.send(200, "text/html", html);
  }
}

void handleConfig() {
  server.send_P(200, "text/html", CONFIG_PAGE);
}

void handleSaveWifi() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", server.arg("ssid"));
    preferences.putString("password", server.arg("password"));
    preferences.end();
    server.send(200, "text/plain", "WiFi credentials saved. Restarting...");
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleStart() {
  if (collecting) {
    server.send(400, "text/plain", "Collection already in progress.");
    return;
  }
  collecting = true;
  scanStartTime = millis();
  scanElapsedTime = 0;
  
  // Perform data collection and processing
  readAndAccumulate();
  generateInterpolatedSpectrum();
  
  scanElapsedTime = millis() - scanStartTime;
  collecting = false;
  Serial.println("Data collection and processing complete.");
  server.send(200, "text/plain", "Collection complete");
}

void handleData() {
  // Create a JSON object with three properties: interpolated, raw, and labels
  String json = "{";

  // 1. Add interpolated data
  json += "\"interpolated\":[";
  for (int i = 0; i < interpolationPoints; i++) {
    json += "[" + String(interpolatedSpectrum[i][0], 2) + "," + String(interpolatedSpectrum[i][1], 4) + "]";
    if (i < interpolationPoints - 1) json += ",";
  }
  json += "],";

  // 2. Add raw channel data (9 channels, excluding Clear)
  json += "\"raw\":[";
  for (int i = 0; i < 9; i++) {
    json += "[" + String(channelWavelengths[i]) + "," + String(normalizedReadings[i], 4) + "]";
    if (i < 8) json += ",";
  }
  json += "],";

  // 3. Add channel names for labels (9 channels)
  json += "\"labels\":[";
  for (int i = 0; i < 9; i++) {
    json += "\"" + String(channelNames[i]) + "\"";
    if (i < 8) json += ",";
  }
  json += "],";

  // 4. Add connector line data
  json += "\"connector\":[";
  // First point: last point of interpolated spectrum
  json += "[" + String(interpolatedSpectrum[interpolationPoints - 1][0], 2) + "," + String(interpolatedSpectrum[interpolationPoints - 1][1], 4) + "],";
  // Second point: the NIR channel raw data (index 8)
  json += "[" + String(channelWavelengths[8]) + "," + String(normalizedReadings[8], 4) + "]";
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

void handleGetConfig() {
  String json = "{\"accumulationTime\": " + String(accumulationTime) + ", \"scanInterval\": " + String(scanInterval) + "}";
  server.send(200, "application/json", json);
}

void handleSetConfig() {
  bool updated = false;
  if (server.hasArg("accumulationTime")) {
    accumulationTime = server.arg("accumulationTime").toInt();
    if (accumulationTime < 1) accumulationTime = 1;
    if (accumulationTime > 60) accumulationTime = 60;
    updated = true;
  }
  if (server.hasArg("scanInterval")) {
    scanInterval = server.arg("scanInterval").toInt();
    if (scanInterval < 1) scanInterval = 1;
    if (scanInterval > 3600) scanInterval = 3600;
    updated = true;
  }
  
  if (updated) {
    server.send(200, "text/plain", "Configuration updated.");
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleResetWifi() {
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();
  server.send(200, "text/plain", "WiFi settings cleared. Restarting...");
  delay(1000);
  ESP.restart();
}