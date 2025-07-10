# World Weather Watcher – SD-Logging Weather Station  
Built as a CESI school project to explore low-power data acquisition, mode management, and on-device storage.

## Project summary
An ESP8266-based station records ambient temperature, humidity, pressure, and light every minute, writes the readings to an SD card with rolling files, and supports four runtime modes you can cycle through with a single push-button:

| Mode | Purpose | Indicators |
|------|---------|------------|
| **STANDARD** | Normal 1-min sampling, full-power Wi-Fi upload if available | Status LED steady green |
| **ECONOMIC** | 10-min sampling, Wi-Fi off, deep-sleep between wakes | LED single green blink |
| **CONFIGURATION** | Keeps Wi-Fi on and starts a captive-portal for SSID / NTP setup | LED breathing blue |
| **MAINTENANCE** | Stops sampling; mount SD card over USB for data retrieval | LED solid yellow |

All settings persist across power-cycles (saved to EEPROM).  
If the SD card fills up, the sketch raises a red LED alert and pauses logging.

## Hardware
| Part | Notes |
|------|-------|
| ESP8266 NodeMCU (or Wemos D1 mini) | 3 V3 logic |
| BME280 sensor breakout | I²C address 0x76 (configurable) |
| BH1750 light sensor | I²C address 0x23 |
| MicroSD adapter | Uses SPI pins; CS on D8 |
| RGB LED | Common-anode, driven by D5/D6/D7 |
| Momentary push-button | Connected to D3 + GND with internal pull-up |
