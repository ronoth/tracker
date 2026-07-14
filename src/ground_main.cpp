#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "board.h"
#include "flight_state.h"
#include "gps_data.h"
#include "lora_config.h"
#include "ground_common.h"

// LoRa radio
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI);

// OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// WiFi and Web Server
const char* ssid = "RocketTracker";
const char* password = "rocket123";
WebServer server(80);

// BLE
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_TELEMETRY_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_NAME_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_FREQ_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26aa"

BLEServer* pServer = nullptr;
BLECharacteristic* pTelemetryChar = nullptr;
BLECharacteristic* pNameChar = nullptr;
BLECharacteristic* pFreqChar = nullptr;

// Persistent storage
Preferences preferences;
bool bleClientConnected = false;
unsigned long lastBLEUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL = 1000;

// Station name set by mobile app
char stationName[32] = "";

// Flag to reinitialize LoRa from main loop (cannot do SPI from BLE callback)
volatile bool loraReinitNeeded = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        bleClientConnected = true;
        Serial.println("BLE client connected");
    }
    void onDisconnect(BLEServer* pServer) override {
        bleClientConnected = false;
        Serial.println("BLE client disconnected");
        pServer->startAdvertising();
    }
};

// LoRa config
float loraFrequency = LORA_DEFAULT_FREQUENCY; // MHz, overridden from preferences

// Forward declaration
void initializeLoRa();

class NameCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0 && value.length() < sizeof(stationName)) {
            strncpy(stationName, value.c_str(), sizeof(stationName) - 1);
            stationName[sizeof(stationName) - 1] = '\0';
            Serial.print("Station name set: ");
            Serial.println(stationName);
            preferences.begin("config", false);
            preferences.putString("name", stationName);
            preferences.end();
        }
    }
};

class FreqCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            float newFreq = atof(value.c_str());
            if (isValidLoRaFrequency(newFreq)) {
                loraFrequency = newFreq;
                Serial.print("Frequency set: ");
                Serial.print(loraFrequency, 1);
                Serial.println(" MHz");
                preferences.begin("config", false);
                preferences.putFloat("frequency", loraFrequency);
                preferences.end();
                // Update readable characteristic value
                String freqStr = String(loraFrequency, 1);
                pCharacteristic->setValue(freqStr.c_str());
                // Defer LoRa reinit to main loop — SPI from BLE callback context can hang
                loraReinitNeeded = true;
            }
        }
    }
};

// Interrupt flags
volatile bool receivedFlag = false;
volatile bool enableInterrupt = true;

// Per-tracker GPS data received over LoRa
GPSData trackers[MAX_TRACKERS];

// Set true on first received packet; never cleared — gates the "waiting" screen
bool hasEverReceived = false;

// Display state
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 1000;
bool displayInitialized = false;
uint8_t bootLine = 0;

void setFlag(void) {
    if (!enableInterrupt) {
        return;
    }
    receivedFlag = true;
}

void bootStatus(const char* msg) {
    Serial.print("[BOOT] ");
    Serial.println(msg);

    if (!displayInitialized) return;

    int y = 16 + bootLine * 10;
    if (y + 10 > SCREEN_HEIGHT) return;

    display.setCursor(0, y);
    display.print("> ");
    display.println(msg);
    display.display();
    bootLine++;
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='3'>";
    html += "<title>Rocket Tracker</title>";
    html += "<style>";
    html += "body { font-family: Arial; margin: 20px; background: #f0f0f0; }";
    html += ".container { background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }";
    html += ".gps-data { font-size: 18px; margin: 10px 0; }";
    html += ".button { display: block; width: 100%; padding: 15px; margin: 10px 0; background: #007AFF; color: white; text-decoration: none; border-radius: 8px; text-align: center; font-size: 16px; box-sizing: border-box; }";
    html += ".button:hover { background: #0056b3; }";
    html += ".status { font-size: 14px; color: #666; margin: 5px 0; }";
    html += ".no-data { color: #ff6b6b; font-size: 20px; text-align: center; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>&#128640; Rocket Tracker</h1>";

    unsigned long currentTime = millis();
    if (!hasEverReceived) {
        html += "<div class='no-data'>No GPS Data Available</div>";
        html += "<p>Waiting for rocket signal...</p>";
    } else {
        // One block per tracked tracker
        for (int i = 0; i < MAX_TRACKERS; i++) {
            GPSData& t = trackers[i];
            if (!t.dataValid) continue;

            html += "<h2>&#128752; ";
            html += trackerLabel(t);
            html += "</h2>";
            html += "<div class='gps-data'><strong>State:</strong> ";
            html += flightStateName(t.vehicleState);
            html += "</div>";

            if (t.hasGPSFix) {
                html += "<div class='gps-data'>";
                html += "<strong>Latitude:</strong> " + String(t.latitude, 6) + "<br>";
                html += "<strong>Longitude:</strong> " + String(t.longitude, 6) + "<br>";
                html += "<strong>Altitude:</strong> " + String(t.altitude, 1) + " m<br>";
                html += "<strong>Satellites:</strong> " + String(t.satellites) + "<br>";
                html += "</div>";

                String appleMapsUrl = "maps://maps.apple.com/?q=" + String(t.latitude, 6) + "," + String(t.longitude, 6);
                html += "<a href='" + appleMapsUrl + "' class='button'>&#128205; Open in Apple Maps</a>";

                String googleMapsUrl = "https://maps.google.com/maps?q=" + String(t.latitude, 6) + "," + String(t.longitude, 6);
                html += "<a href='" + googleMapsUrl + "' class='button'>&#128506; Open in Google Maps</a>";

            } else if (t.lastGPSFixTime > 0) {
                uint32_t locAge = (currentTime - t.lastGPSFixTime) / 1000;
                html += "<div class='no-data' style='font-size:16px;color:#e6a817;'>No GPS Fix &mdash; Last Known (" + String(locAge) + "s ago)</div>";
                html += "<div class='gps-data'>";
                html += "<strong>Latitude:</strong> " + String(t.latitude, 6) + "<br>";
                html += "<strong>Longitude:</strong> " + String(t.longitude, 6) + "<br>";
                html += "<strong>Altitude:</strong> " + String(t.altitude, 1) + " m<br>";
                html += "</div>";

                // Still provide map links for last known position
                String appleMapsUrl = "maps://maps.apple.com/?q=" + String(t.latitude, 6) + "," + String(t.longitude, 6);
                html += "<a href='" + appleMapsUrl + "' class='button'>&#128205; Open in Apple Maps</a>";

                String googleMapsUrl = "https://maps.google.com/maps?q=" + String(t.latitude, 6) + "," + String(t.longitude, 6);
                html += "<a href='" + googleMapsUrl + "' class='button'>&#128506; Open in Google Maps</a>";

            } else {
                html += "<div class='no-data'>No GPS Fix</div>";
                html += "<p>Satellites: " + String(t.satellites) + "</p>";
            }

            // Per-tracker signal quality and data age
            html += "<div class='status'>";
            html += "Signal: " + String(t.rssi, 0) + " dBm, SNR: " + String(t.snr, 1) + " dB<br>";
            if (t.batteryVoltage > 0) {
                html += "Battery: " + String(t.batteryVoltage, 2) + " V<br>";
            }
            html += "Last update: " + String((currentTime - t.lastUpdate) / 1000) + " seconds ago";
            html += "</div>";
        }
    }
    
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}

void initializeWiFi() {
    Serial.println("Starting WiFi Access Point");
    
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    server.on("/", handleRoot);
    server.begin();
    Serial.println("Web server started");
}

void initializeDisplay() {
    // Enable power to display
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);

    // Initialize I2C for OLED
    Wire.begin(OLED_I2C_SDA, OLED_I2C_SCL);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        return;
    }
    
    displayInitialized = true;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Ground Station");
    display.display();
}

void initializeLoRa() {
    // Initialize LoRa SPI
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // Initialize LoRa radio for reception
    int16_t res = radio.begin(
        loraFrequency,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_POWER,
        LORA_PREAMBLE_LEN,
        LORA_TCXO_VOLTAGE,
        LORA_USE_LDO
    );

    if (res != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to initialize LoRa radio. Code: ");
        Serial.println(res);
        return;
    }

    // Set interrupt handler
    radio.setDio1Action(setFlag);

    // Start listening
    res = radio.startReceive();
    if (res != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to start receive. Code: ");
        Serial.println(res);
        return;
    }

    Serial.println("LoRa radio initialized and listening");
}

void updateDisplay() {
    if (!displayInitialized) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    
    lastDisplayUpdate = currentTime;
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
      
    // Header: station name (set by mobile app) left, LoRa frequency (MHz)
    // right-aligned; name truncated to keep the frequency intact
    String freqStr = String(loraFrequency, 1);
    int freqX = SCREEN_WIDTH - 6 * freqStr.length();
    int maxNameChars = (freqX - 6) / 6; // at least one blank column before freq
    for (int i = 0; i < maxNameChars && stationName[i]; i++) {
        display.print(stationName[i]);
    }
    display.setCursor(freqX, 0);
    display.print(freqStr);
    display.println();

    GPSData* cur = mostRecentTracker(trackers);
    if (!hasEverReceived || !cur) {
        display.println("NO DATA");
        display.print("Waiting for signal...");
    } else {
        // Most recently heard tracker + flight state
        display.print(trackerLabel(*cur));
        display.print(" [");
        display.print(flightStateName(cur->vehicleState));
        display.println("]");

        if (cur->hasGPSFix) {
            display.print("LAT: ");
            display.println(cur->latitude, 4);
            display.print("LON: ");
            display.println(cur->longitude, 4);
            display.print("ALT: ");
            display.print(cur->altitude, 0);
            display.print("m SAT:");
            display.println(cur->satellites);
        } else if (cur->lastGPSFixTime > 0) {
            // No current fix — last known position; LLOC line shows its age
            display.println("NO FIX - LAST KNOWN");
            display.print("LAT: ");
            display.println(cur->latitude, 4);
            display.print("LON: ");
            display.println(cur->longitude, 4);
        } else {
            display.println("NO GPS FIX");
            display.print("Sats:");
            display.print(cur->satellites);
            display.print(" HDOP:");
            display.println(cur->hdop, 1);
        }

        // Signal quality + vehicle battery (older vehicles don't send battery)
        display.print("RSSI:");
        display.print(cur->rssi, 0);
        display.print(" SNR:");
        display.println(cur->snr, 1);
        if (cur->batteryVoltage > 0) {
            display.print("BAT:");
            display.print(cur->batteryVoltage, 2);
            display.println("V");
        }

        // Packet age and location age — label and value bold when stale
        uint32_t dataAge = (currentTime - cur->lastUpdate) / 1000;
        uint32_t locAge = cur->lastGPSFixTime > 0 ? (currentTime - cur->lastGPSFixTime) / 1000 : 0;
        char ageBuf[32];
        snprintf(ageBuf, sizeof(ageBuf), "DAGE:%lus", (unsigned long)dataAge);
        printBoldIf(display, ageBuf, dataAge > STALE_AGE_SEC);
        snprintf(ageBuf, sizeof(ageBuf), " LLOC:%lus", (unsigned long)locAge);
        printBoldIf(display, ageBuf, locAge > STALE_AGE_SEC);
        display.println();
    }

    display.display();
}

void initializeBLE() {
    BLEDevice::init("RocketTracker");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);
    pTelemetryChar = pService->createCharacteristic(
        BLE_TELEMETRY_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pTelemetryChar->addDescriptor(new BLE2902());

    pNameChar = pService->createCharacteristic(
        BLE_NAME_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pNameChar->setCallbacks(new NameCharCallbacks());

    pFreqChar = pService->createCharacteristic(
        BLE_FREQ_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pFreqChar->setCallbacks(new FreqCharCallbacks());
    String freqStr = String(loraFrequency, 1);
    pFreqChar->setValue(freqStr.c_str());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started");
}

void updateBLE() {
    if (!bleClientConnected) return;

    unsigned long currentTime = millis();
    if (currentTime - lastBLEUpdate < BLE_UPDATE_INTERVAL) return;
    lastBLEUpdate = currentTime;

    if (!hasEverReceived) return;

    // One self-contained notification per tracker (see buildTrackerJSON)
    for (int i = 0; i < MAX_TRACKERS; i++) {
        if (!trackers[i].dataValid) continue;
        String json = buildTrackerJSON(trackers[i], currentTime, loraFrequency);
        pTelemetryChar->setValue(json.c_str());
        pTelemetryChar->notify();
    }
}

void setup() {
    Serial.begin(115200);
    bootStatus("Starting...");

    bootStatus("Loading config");
    preferences.begin("config", true);
    loraFrequency = preferences.getFloat("frequency", LORA_DEFAULT_FREQUENCY);
    String savedName = preferences.getString("name", "");
    if (savedName.length() > 0 && savedName.length() < sizeof(stationName)) {
        strncpy(stationName, savedName.c_str(), sizeof(stationName) - 1);
        stationName[sizeof(stationName) - 1] = '\0';
    }
    preferences.end();

    bootStatus("Init OLED display");
    initializeDisplay();

    bootStatus("Init WiFi AP");
    initializeWiFi();

    bootStatus("Init BLE");
    initializeBLE();

    bootStatus("Init LoRa");
    initializeLoRa();

    memset(trackers, 0, sizeof(trackers));

    bootStatus("Ready!");
}

void loop() {
    // Check for received packets
    if (receivedFlag) {
        enableInterrupt = false;
        receivedFlag = false;

        String str;
        int state = radio.readData(str);

        if (state == RADIOLIB_ERR_NONE) {
            Serial.print("Packet received: ");
            Serial.println(str);

            GPSData* rec = parseGPSPacket(str.c_str(), trackers);
            if (rec) {
                hasEverReceived = true;
                rec->rssi = radio.getRSSI();
                rec->snr = radio.getSNR();

                Serial.print("RSSI: ");
                Serial.print(rec->rssi);
                Serial.print(" dBm, SNR: ");
                Serial.print(rec->snr);
                Serial.println(" dB");
            }
        } else {
            Serial.print("Failed to read packet, code: ");
            Serial.println(state);
        }

        // Restart listening
        radio.startReceive();
        enableInterrupt = true;
    }

    // Reinitialize LoRa if frequency was changed via BLE
    if (loraReinitNeeded) {
        loraReinitNeeded = false;
        Serial.println("Reinitializing LoRa with new frequency...");
        initializeLoRa();
    }

    // Handle web server requests
    server.handleClient();

    // Update display
    updateDisplay();

    // Update BLE
    updateBLE();
    
    // Small delay to prevent overwhelming the system
    delay(10);
}