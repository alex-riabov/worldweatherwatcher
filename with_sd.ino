#include <Arduino.h>
#include <ChainableLED.h>
#include <Wire.h>
#include <RTClib.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <avr/pgmspace.h>
 
// Constants and Pins
const uint8_t redButtonPin = 2;
const uint8_t greenButtonPin = 3;
const uint32_t holdDuration = 5000; // Duration to switch mode (5s)
 
// LED and Sensor Pins
const uint8_t ledDataPin = 5;
const uint8_t ledClockPin = 6;
const uint8_t lightSensorPin = A1;
 
// GPS Pins and Object Initialization
SoftwareSerial gpsSerial(8, 9); // RX, TX
TinyGPSPlus gps;
 
// Mode Enum and State Variables
enum Mode : uint8_t { STANDARD, CONFIGURATION, MAINTENANCE, ECONOMIC };
Mode currentMode = STANDARD;
 
// Initialize Components
ChainableLED leds(ledDataPin, ledClockPin, 1);
RTC_DS3231 rtc;
HTU21D mySensor;
 
// SD Card variables
File dataFile;
const uint8_t chipSelect = 4;
 
// Button state variables
bool EtatbtnRouge = false;
bool EtatbtnVert = false;
uint32_t buttonPressTimeRouge;
uint32_t buttonPressTimeVert;

uint32_t lastLogTime = 0;
uint32_t logInterval = 5000; // Default log interval of 10 minutes
uint32_t logIntervalEconomic = logInterval * 2; // Default log interval of 10 minutes
uint32_t FILE_MAX_SIZE = 2048; // 2KB max file size
uint32_t TIMEOUT = 30000; // 30s timeout
uint8_t revisionNumber = 0;
char filename[13];
bool modeChange = false; // Tracks mode toggling between STANDARD and ECONOMIC

 
// Messages stored in PROGMEM to save RAM
const char msgSDCardFail[] PROGMEM = "SD card initialization failed!";
const char msgSDCardInit[] PROGMEM = "SD card initialized.";
const char msgModeMaintenance[] PROGMEM = "Mode: Maintenance";
const char msgModeStandard[] PROGMEM = "Mode: Standard";
const char msgModeEconomic[] PROGMEM = "Mode: Economic";

// Error Codes
enum ErrorCode {
    NO_ERROR,
    RTC_ERROR,
    GPS_ERROR,
    SENSOR_ERROR,
    SENSOR_INCOHERENT,
    SD_FULL,
    SD_WRITE_ERROR
};

// Global error state
ErrorCode currentError = NO_ERROR;


void printFromPROGMEM(const char *msg) {
    char buffer[50];
    strcpy_P(buffer, msg);
    Serial.println(buffer);
}
 
void setup() {
    Serial.begin(9600);
    Wire.begin();
    gpsSerial.begin(9600);
 
    // Initialize LED
    leds.setColorRGB(0, 0, 255, 0); // Green by default
 
    // Setup buttons
    pinMode(redButtonPin, INPUT_PULLUP);
    pinMode(greenButtonPin, INPUT_PULLUP);
 
    // Initialize RTC
    if (!rtc.begin()) {
        Serial.println(F("RTC failed to start. Check connections."));
        currentError = RTC_ERROR;
        while (1) indicateError(currentError); // Stop and indicate error
}
 
    // Initialize HTU21D
    mySensor.begin();
 
    // Initialize SD Card
    if (!SD.begin(chipSelect)) {
        printFromPROGMEM(msgSDCardFail);
        currentError = SD_WRITE_ERROR;
        while (1) indicateError(currentError);
    }
    printFromPROGMEM(msgSDCardInit);
    createFilename(0);

}
 
void loop() {
    // Update GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
 
    // Check the current mode
    switch (currentMode) {
        case STANDARD:
            if (millis() - lastLogTime >= logInterval) {
                acquireAndLogData();
                lastLogTime = millis();
            }
            break;
        case ECONOMIC:
            if (millis() - lastLogTime >= logIntervalEconomic) {
                acquireAndLogData();
                lastLogTime = millis();
            }
            break;
 
        case MAINTENANCE:
            // In maintenance mode, only display data on Serial
            acquireDataAndDisplay();
            break;
    }
 
    // Handle button presses
    checkButtonPresses();
}
 
void acquireAndLogData() {
    DateTime now = rtc.now();
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(" - Temp: "));
    Serial.print(mySensor.readTemperature(), 1);
    Serial.print(F(" C, Humidity: "));
    Serial.print(mySensor.readHumidity(), 1);
    Serial.print(F(" %, Light: "));
    Serial.print(analogRead(lightSensorPin));
    
    // GPS data handling
    static bool gpsSkip = false;
    if (currentMode == ECONOMIC) gpsSkip = !gpsSkip; // Skip GPS data every other acquisition
    
    if (!gpsSkip && gps.location.isValid()) {
        Serial.print(F(", Lat: "));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", Lon: "));
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print(F(", GPS: No Fix"));
    }
    Serial.println();
    
    // Log data to SD card
    if (currentMode == STANDARD || ECONOMIC) {
        dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile) {
            dataFile.print(now.timestamp(DateTime::TIMESTAMP_FULL));
            dataFile.print(F(" - Temp: "));
            dataFile.print(mySensor.readTemperature(), 1);
            dataFile.print(F(" C, Humidity: "));
            dataFile.print(mySensor.readHumidity(), 1);
            dataFile.print(F(" %, Light: "));
            dataFile.print(analogRead(lightSensorPin));
            
            if (!gpsSkip && gps.location.isValid()) {
                dataFile.print(F(", Lat: "));
                dataFile.print(gps.location.lat(), 6);
                dataFile.print(F(", Lon: "));
                dataFile.print(gps.location.lng(), 6);
            } else {
                dataFile.print(F(", GPS: NA"));
            }
            dataFile.println();
            dataFile.close();
        } else {
            Serial.println(F("Error opening data file"));
        }
    }
    if (!gps.location.isValid()) {
    currentError = GPS_ERROR;
    indicateError(currentError);
    if (!dataFile) {
    currentError = SD_WRITE_ERROR;
    indicateError(currentError);
    }
  }
}


bool isSdCardFull() {
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        bool isFull = dataFile.size() >= FILE_MAX_SIZE;
        dataFile.close();
        return isFull;
    } else {
        // If file can't be opened, assume an SD write error
        currentError = SD_WRITE_ERROR;
        indicateError(currentError);
        Serial.println(F("Error opening data file"));
        return true; // Treat as full to prevent further attempts
    }
}


 
void acquireDataAndDisplay() {
    DateTime now = rtc.now();
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(" - Temp: "));
    Serial.print(mySensor.readTemperature(), 1);
    Serial.print(F(" C, Humidity: "));
    Serial.print(mySensor.readHumidity(), 1);
    Serial.print(F(" %, Light: "));
    Serial.print(analogRead(lightSensorPin));
 
    if (gps.location.isValid()) {
        Serial.print(F(", Lat: "));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(", Lon: "));
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print(F(", GPS: No Fix"));
    }
    Serial.println();
}
 
void checkButtonPresses() {
    // Check red button press for entering/exiting Maintenance mode
    if (digitalRead(redButtonPin) == LOW) {
        unsigned long pressStartTime = millis();
        while (digitalRead(redButtonPin) == LOW) {} // Wait for button release
        unsigned long pressDuration = millis() - pressStartTime;

        if (pressDuration >= holdDuration) {
            if (currentMode == STANDARD || currentMode == ECONOMIC) {
                currentMode = MAINTENANCE;
                leds.setColorRGB(0, 255, 165, 0); // Orange for maintenance mode
                printFromPROGMEM(msgModeMaintenance);
            } else if (currentMode == MAINTENANCE) {
                if (!modeChange) {
                    currentMode = STANDARD;
                    leds.setColorRGB(0, 0, 255, 0); // Green for standard mode
                } else {
                    currentMode = ECONOMIC;
                    leds.setColorRGB(0, 0, 0, 255); // Blue for economic mode
                }
                printFromPROGMEM(msgModeStandard);
            }
        }
    }

    // Check green button press for entering/exiting Economic mode
    if (digitalRead(greenButtonPin) == LOW) {
        unsigned long pressStartTime = millis();
        while (digitalRead(greenButtonPin) == LOW) {} // Wait for button release
        unsigned long pressDuration = millis() - pressStartTime;

        if (pressDuration >= holdDuration) {
            if (currentMode == STANDARD) {
                currentMode = ECONOMIC;
                modeChange = true;
                leds.setColorRGB(0, 0, 0, 255); // Blue for economic mode
                printFromPROGMEM(msgModeEconomic);
            } else if (currentMode == ECONOMIC) {
                currentMode = STANDARD;
                modeChange = false;
                leds.setColorRGB(0, 0, 255, 0); // Green for standard mode
                printFromPROGMEM(msgModeStandard);
            }
        }
    }
}

 
void basculeModeRouge() {
    if (currentMode != MAINTENANCE) {
        currentMode = MAINTENANCE;
        leds.setColorRGB(0, 255, 165, 0); // Orange for maintenance mode
        printFromPROGMEM(msgModeMaintenance);
    } else {
        currentMode = STANDARD;
        leds.setColorRGB(0, 0, 255, 0); // Green for standard mode
        printFromPROGMEM(msgModeStandard);
    }
}
 
void basculeModeVert() {
    if (currentMode == STANDARD) {
        currentMode = ECONOMIC;
        leds.setColorRGB(0, 0, 0, 255); // Blue for economic mode
        printFromPROGMEM(msgModeEconomic);
    } else if (currentMode == ECONOMIC) {
        currentMode = STANDARD;
        leds.setColorRGB(0, 0, 255, 0); // Green for standard mode
        printFromPROGMEM(msgModeStandard);
    }
}

void createFilename(uint8_t revision) {
    DateTime now = rtc.now();
    snprintf(filename, sizeof(filename), "%02d%02d%02d_%d.LOG", 
             now.year() % 100, now.month(), now.day(), revision);
}

void indicateError(ErrorCode error) {
    switch (error) {
        case RTC_ERROR:
            leds.setColorRGB(0, 255, 0, 0); // Red and Blue (1Hz, equal duration)
            delay(500);
            leds.setColorRGB(0, 0, 0, 255);
            delay(500);
            break;
        case GPS_ERROR:
            leds.setColorRGB(0, 255, 0, 0); // Red
            delay(500);
            leds.setColorRGB(0, 255, 255, 0); // Yellow
            delay(500);
            break;
        case SENSOR_ERROR:
            leds.setColorRGB(0, 255, 0, 0); // Red and Green (1Hz, equal duration)
            delay(500);
            leds.setColorRGB(0, 0, 255, 0);
            delay(500);
            break;
        case SENSOR_INCOHERENT:
            leds.setColorRGB(0, 255, 0, 0); // Red and Green (1Hz, longer green duration)
            delay(500);
            leds.setColorRGB(0, 0, 255, 0);
            delay(1000);
            break;
        case SD_FULL:
            leds.setColorRGB(0, 255, 0, 0); // Red and White (1Hz, equal duration)
            delay(500);
            leds.setColorRGB(0, 255, 255, 255);
            delay(500);
            break;
        case SD_WRITE_ERROR:
            leds.setColorRGB(0, 255, 0, 0); // Red and White (1Hz, longer white duration)
            delay(500);
            leds.setColorRGB(0, 255, 255, 255);
            delay(1000);
            break;
        case NO_ERROR:
        default:
            leds.setColorRGB(0, 0, 255, 0); // Green for normal operation
            break;
    }
}


