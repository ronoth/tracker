#include <Arduino.h>
#include "board.h"
#include <RadioLib.h>
#include <TinyGPS++.h>

// GPS Module pins
static const int RXPin = 42, TXPin = 45;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial gpsSerial(1);

// LoRa radio setup
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI);

// Structure to hold GPS data
struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int satellites;
    float hdop;
    bool locationValid;
    bool altitudeValid;
    bool speedValid;
} gpsData;

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.println("GPS LoRa Transmitter");

    // Initialize GPS Module
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // Initialize SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // Initialize LoRa radio
    Serial.println(F("Initializing LoRa radio"));
    int16_t res = radio.begin(
        915.0 /* freq */,
        125.0 /* bw */,
        9 /* sf */,
        7 /* cr*/,
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, /* sync word */
        22 /* Power */,
        8 /* preambleLength */,
        1.6 /* tcxoVoltage*/,
        true /* useRegulatorLDO */);

    if(res != RADIOLIB_ERR_NONE) {
        Serial.print(F("Failed to initialize LoRa radio. Code:"));
        Serial.println(res);
        while(1);
    }
    radio.setDio2AsRfSwitch(true);
}

void updateGPSData() {
    // Update GPS data structure with latest readings
    gpsData.locationValid = gps.location.isValid();
    if (gpsData.locationValid) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
    }

    gpsData.altitudeValid = gps.altitude.isValid();
    if (gpsData.altitudeValid) {
        gpsData.altitude = gps.altitude.meters();
    }

    gpsData.speedValid = gps.speed.isValid();
    if (gpsData.speedValid) {
        gpsData.speed = gps.speed.kmph();
    }

    gpsData.satellites = gps.satellites.value();
    gpsData.hdop = gps.hdop.hdop();
}

void loop() {
    // Read GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // Check if GPS is working
    if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println("No GPS detected");
        while(true);
    }

    // Update GPS data structure
    updateGPSData();

    // Create a string with GPS data
    char lora_buff[100];
    if (gpsData.locationValid) {
        snprintf(lora_buff, sizeof(lora_buff), 
            "GPS:%.6f,%.6f,%.1f,%.1f,%d,%.1f",
            gpsData.latitude,
            gpsData.longitude,
            gpsData.altitudeValid ? gpsData.altitude : 0.0,
            gpsData.speedValid ? gpsData.speed : 0.0,
            gpsData.satellites,
            gpsData.hdop
        );
    } else {
        snprintf(lora_buff, sizeof(lora_buff), 
            "GPS:NoFix,%d,%.1f",
            gpsData.satellites,
            gpsData.hdop
        );
    }

    // Transmit the data
    Serial.print("Transmitting: ");
    Serial.println(lora_buff);
    radio.startTransmit(lora_buff, strlen(lora_buff));
    
    // Wait for 3 seconds before next transmission
    delay(3000);
}
