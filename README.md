# ESP32-based Spectrometer (Tricorder)
<img width="1876" height="958" alt="image" src="https://github.com/user-attachments/assets/883b8ced-9c39-4042-a934-771675de8c6b" />


This project turns an ESP32 and a DFRobot AS7341 spectral sensor into a web-based spectrometer. It reads spectral data, processes it, and displays it as a smooth curve on a web interface.

## Features

*   **Spectral Sensing:** Measures light intensity across 10 different channels using the AS7341 sensor.
*   **Data Interpolation:** Uses a Catmull-Rom spline to generate a smooth, interpolated spectrum from the raw sensor data.
*   **Web Interface:** Provides a web-based UI to:
    *   View the real-time spectrum graph.
    *   Start and stop data collection.
    *   Configure device settings.
    *   Control the onboard LED.
*   **WiFi Management:**
    *   Starts in Access Point (AP) mode for initial configuration.
    *   Connects to a saved WiFi network in Station (STA) mode.
    *   Allows saving and resetting WiFi credentials.
*   **Configuration Persistence:** Saves WiFi and device settings to non-volatile storage.

## Hardware
<img width="541" height="603" alt="Screenshot from 2025-10-19 20-16-00" src="https://github.com/user-attachments/assets/bc63433d-a392-40e3-9af1-195b6592eda2" />

*   **Microcontroller:** ESP32
*   **Sensor:** DFRobot AS7341 Visible Light Sensor

## How to Use

1.  **First Boot (AP Mode):**
    *   On the first boot, or if no WiFi credentials are saved, the device will start in Access Point (AP) mode.
    *   Connect to the WiFi network created by the ESP32 (the SSID will be "Tricorder-XXXX").
    *   Open a web browser and navigate to `192.168.4.1`.
    *   Go to the "Config" page, enter your WiFi credentials, and save. The device will then restart and connect to your network.

2.  **Normal Operation (STA Mode):**
    *   Once connected to your WiFi network, the device's IP address will be printed to the Serial monitor.
    *   Access the web interface by navigating to this IP address in your browser.
    *   From the main page, you can start and stop spectral data collection and see the results plotted in real-time.

## Dependencies

*   [DFRobot_AS7341](https://github.com/DFRobot/DFRobot_AS7341)
*   [ArduinoJson](https://arduinojson.org/)
*   ESP32 Core for Arduino (for WebServer, WiFi, and Preferences libraries)
