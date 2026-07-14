#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "board_wio_l1.h"
#include "flight_state.h"
#include "gps_data.h"
#include "lora_config.h"
#include "ground_common.h"

using namespace Adafruit_LittleFS_Namespace;

// LoRa radio (on default SPI bus)
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI);

// E-ink display (on SPI1 bus)
GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> eink(
    GxEPD2_213_B74(PIN_EINK_CS, PIN_EINK_DC, PIN_EINK_RES, PIN_EINK_BUSY));

// BLE UUIDs (little-endian byte order for Bluefruit)
const uint8_t BLE_SVC_UUID[] = {
    0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
    0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f
};
const uint8_t BLE_TEL_UUID[] = {
    0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
};
const uint8_t BLE_NAME_UUID[] = {
    0xa9, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
};
const uint8_t BLE_FREQ_UUID[] = {
    0xaa, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe
};

BLEService rktService(BLE_SVC_UUID);
BLECharacteristic telemetryChar(BLE_TEL_UUID);
BLECharacteristic nameChar(BLE_NAME_UUID);
BLECharacteristic freqChar(BLE_FREQ_UUID);

// Config (persisted to internal flash)
struct Config {
    uint32_t magic;
    float frequency;
    char stationName[32];
};
#define CONFIG_MAGIC 0x524F434B // "ROCK"

float loraFrequency = LORA_DEFAULT_FREQUENCY;
char stationName[32] = "";
volatile bool loraReinitNeeded = false;
bool bleClientConnected = false;

// LoRa interrupt
volatile bool receivedFlag = false;
volatile bool enableInterrupt = true;

// Per-tracker GPS data received over LoRa
GPSData trackers[MAX_TRACKERS];

// Set true on first received packet; never cleared — gates the "waiting" screen
bool hasEverReceived = false;

// Display timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 5000;
uint8_t partialRefreshCount = 0;
const uint8_t FULL_REFRESH_EVERY = 30;
bool displayNeedsUpdate = false;
bool displayReady = false;
uint8_t bootLine = 0;

// BLE timing
unsigned long lastBLEUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL = 1000;

void setFlag(void) {
    if (!enableInterrupt) return;
    receivedFlag = true;
}

// ── Boot status ──

void blinkLED(int count, int onMs = 200, int offMs = 200) {
    for (int i = 0; i < count; i++) {
        digitalWrite(PIN_LED1, HIGH);
        delay(onMs);
        digitalWrite(PIN_LED1, LOW);
        delay(offMs);
    }
    delay(500);
}

void bootStatus(const char* msg) {
    Serial.print("[BOOT] ");
    Serial.println(msg);

    if (displayReady) {
        eink.setPartialWindow(0, 0, eink.width(), eink.height());
        eink.firstPage();
        do {
            eink.fillScreen(GxEPD_WHITE);
            eink.setTextSize(2);
            eink.setCursor(4, 4);
            eink.println("BOOT");
            eink.setTextSize(1);
            eink.setCursor(4, 28);
            eink.println(msg);
            // Show all boot lines
            for (int i = 0; i < bootLine && i < 8; i++) {
                eink.setCursor(4, 42 + i * 12);
                eink.print("> OK");
            }
        } while (eink.nextPage());
        bootLine++;
    }
}

// ── Config persistence ──

void loadConfig() {
    InternalFS.begin();
    File file(InternalFS);
    if (file.open("config.dat", FILE_O_READ)) {
        Config cfg;
        file.read((uint8_t*)&cfg, sizeof(cfg));
        file.close();
        if (cfg.magic == CONFIG_MAGIC) {
            loraFrequency = cfg.frequency;
            strncpy(stationName, cfg.stationName, sizeof(stationName) - 1);
            stationName[sizeof(stationName) - 1] = '\0';
        }
    }
    Serial.print("Config: freq=");
    Serial.print(loraFrequency, 1);
    Serial.print(" MHz, name=");
    Serial.println(stationName[0] ? stationName : "(none)");
}

void saveConfig() {
    Config cfg;
    cfg.magic = CONFIG_MAGIC;
    cfg.frequency = loraFrequency;
    strncpy(cfg.stationName, stationName, sizeof(cfg.stationName) - 1);
    cfg.stationName[sizeof(cfg.stationName) - 1] = '\0';

    InternalFS.remove("config.dat");
    File file(InternalFS);
    if (file.open("config.dat", FILE_O_WRITE)) {
        file.write((uint8_t*)&cfg, sizeof(cfg));
        file.close();
    }
}

// ── BLE callbacks ──

void connectCallback(uint16_t conn_handle) {
    (void)conn_handle;
    bleClientConnected = true;
    Serial.println("BLE client connected");
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    bleClientConnected = false;
    Serial.println("BLE client disconnected");
}

void nameWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;
    if (len > 0 && len < sizeof(stationName)) {
        memcpy(stationName, data, len);
        stationName[len] = '\0';
        Serial.print("Station name set: ");
        Serial.println(stationName);
        saveConfig();
    }
}

void freqWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    (void)conn_hdl;
    (void)chr;
    char buf[16];
    uint16_t n = min((uint16_t)(sizeof(buf) - 1), len);
    memcpy(buf, data, n);
    buf[n] = '\0';

    float newFreq = atof(buf);
    if (isValidLoRaFrequency(newFreq)) {
        loraFrequency = newFreq;
        Serial.print("Frequency set: ");
        Serial.print(loraFrequency, 1);
        Serial.println(" MHz");
        saveConfig();

        String freqStr = String(loraFrequency, 1);
        freqChar.write(freqStr.c_str(), freqStr.length());

        loraReinitNeeded = true;
    }
}

// ── BLE init ──

bool bleEnabled = false;

void initializeBLE() {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    bool ok = Bluefruit.begin(1, 0);
    Bluefruit.autoConnLed(false);

    if (!ok) {
        Serial.println("[BLE] Bluefruit.begin() FAILED");
        blinkLED(8, 100, 100);
        bootStatus("BLE FAILED!");
        return;
    }
    Serial.println("[BLE] begin() OK");
    blinkLED(3, 100, 100);
    bleEnabled = true;
    Bluefruit.setName("RocketTracker4");
    Bluefruit.setTxPower(4);
    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    Serial.println("[BLE] service begin");
    rktService.begin();

    Serial.println("[BLE] telemetry char");
    telemetryChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    telemetryChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    telemetryChar.setMaxLen(512); // multi-tracker JSON payload
    telemetryChar.begin();

    Serial.println("[BLE] name char");
    nameChar.setProperties(CHR_PROPS_WRITE);
    nameChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    nameChar.setWriteCallback(nameWriteCallback);
    nameChar.setMaxLen(32);
    nameChar.begin();

    Serial.println("[BLE] freq char");
    freqChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    freqChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    freqChar.setWriteCallback(freqWriteCallback);
    freqChar.setMaxLen(16);
    freqChar.begin();

    String freqStr = String(loraFrequency, 1);
    freqChar.write(freqStr.c_str(), freqStr.length());

    Serial.println("[BLE] advertising");
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(rktService);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    Serial.println("[BLE] done");
}

// ── LoRa ──

void initializeLoRa() {
    pinMode(LORA_RXEN, OUTPUT);
    digitalWrite(LORA_RXEN, HIGH);

    SPI.begin();
    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH);

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
        Serial.print("LoRa init failed: ");
        Serial.println(res);
        return;
    }

    radio.setDio2AsRfSwitch(true);
    radio.setDio1Action(setFlag);

    res = radio.startReceive();
    if (res != RADIOLIB_ERR_NONE) {
        Serial.print("Start receive failed: ");
        Serial.println(res);
        return;
    }

    Serial.println("LoRa initialized and listening");
}

// ── E-ink display ──

void initializeDisplay() {
    SPI1.begin();
    eink.epd2.selectSPI(SPI1, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    eink.init(115200, true, 20, false);
    eink.setRotation(1);
    eink.setTextColor(GxEPD_BLACK);

    eink.setFullWindow();
    eink.firstPage();
    do {
        eink.fillScreen(GxEPD_WHITE);
        eink.setTextSize(2);
        eink.setCursor(10, 10);
        eink.println("Ground Station");
        eink.setTextSize(1);
        eink.setCursor(10, 30);
        eink.println("Wio Tracker L1 E-ink");
    } while (eink.nextPage());

    displayReady = true;
}

void updateDisplay() {
    unsigned long currentTime = millis();
    GPSData* cur = mostRecentTracker(trackers);
    if (currentTime - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    if (!displayNeedsUpdate && cur && (currentTime - cur->lastUpdate < 10000)) return;
    lastDisplayUpdate = currentTime;
    displayNeedsUpdate = false;

    bool fullRefresh = (partialRefreshCount >= FULL_REFRESH_EVERY);
    if (fullRefresh) {
        eink.setFullWindow();
        partialRefreshCount = 0;
    } else {
        eink.setPartialWindow(0, 0, eink.width(), eink.height());
        partialRefreshCount++;
    }

    eink.firstPage();
    do {
        eink.fillScreen(GxEPD_WHITE);
        int y = 4;

        // Title: station name left, LoRa frequency (MHz) right-aligned; name
        // truncated to keep the frequency intact
        eink.setTextSize(2);
        eink.setCursor(4, y);
        String freqStr = String(loraFrequency, 1);
        int16_t freqX = eink.width() - 12 * freqStr.length() - 2;
        const char* title = stationName[0] ? stationName : "Ground Station";
        int maxNameChars = (freqX - 4 - 12) / 12; // at least one char gap before freq
        for (int i = 0; i < maxNameChars && title[i]; i++) {
            eink.print(title[i]);
        }
        eink.setCursor(freqX, y);
        eink.print(freqStr);
        y += 24;

        eink.setTextSize(1);

        if (!hasEverReceived || !cur) {
            eink.setTextSize(2);
            eink.setCursor(4, y + 16);
            eink.println("NO DATA");
            eink.setTextSize(1);
            eink.setCursor(4, y + 44);
            eink.println("Waiting for signal...");
        } else {
            // Most recently heard tracker + flight state
            eink.setCursor(4, y);
            eink.print(trackerLabel(*cur));
            eink.print(": ");
            eink.println(flightStateName(cur->vehicleState));
            y += 12;

            if (cur->hasGPSFix) {
                eink.setCursor(4, y);
                eink.print("LAT: ");
                eink.println(cur->latitude, 6);
                y += 12;

                eink.setCursor(4, y);
                eink.print("LON: ");
                eink.println(cur->longitude, 6);
                y += 12;

                eink.setCursor(4, y);
                eink.print("ALT: ");
                eink.print(cur->altitude, 1);
                eink.print("m  SAT: ");
                eink.println(cur->satellites);
                y += 12;
            } else if (cur->lastGPSFixTime > 0) {
                uint32_t fixAge = (currentTime - cur->lastGPSFixTime) / 1000;
                eink.setCursor(4, y);
                eink.print("NO FIX (");
                eink.print(fixAge);
                eink.println("s ago)");
                y += 12;
                eink.setCursor(4, y);
                eink.print("LAT: ");
                eink.println(cur->latitude, 6);
                y += 12;
                eink.setCursor(4, y);
                eink.print("LON: ");
                eink.println(cur->longitude, 6);
                y += 12;
            } else {
                eink.setCursor(4, y);
                eink.println("NO GPS FIX");
                y += 12;
                eink.setCursor(4, y);
                eink.print("Sats: ");
                eink.print(cur->satellites);
                eink.print("  HDOP: ");
                eink.println(cur->hdop, 1);
                y += 12;
            }

            // Signal quality + vehicle battery (older vehicles don't send battery)
            eink.setCursor(4, y);
            eink.print("RSSI: ");
            eink.print(cur->rssi, 0);
            eink.print(" dBm  SNR: ");
            eink.print(cur->snr, 1);
            eink.println(" dB");
            y += 12;
            if (cur->batteryVoltage > 0) {
                eink.setCursor(4, y);
                eink.print("BAT: ");
                eink.print(cur->batteryVoltage, 2);
                eink.println(" V");
                y += 12;
            }

            // Packet age and location age — label and value bold when stale
            uint32_t dataAge = (currentTime - cur->lastUpdate) / 1000;
            uint32_t locAge = cur->lastGPSFixTime > 0 ? (currentTime - cur->lastGPSFixTime) / 1000 : 0;
            char ageBuf[32];
            eink.setCursor(4, y);
            snprintf(ageBuf, sizeof(ageBuf), "Last Data: %lus", (unsigned long)dataAge);
            printBoldIf(eink, ageBuf, dataAge > STALE_AGE_SEC);
            snprintf(ageBuf, sizeof(ageBuf), " Loc: %lus", (unsigned long)locAge);
            printBoldIf(eink, ageBuf, locAge > STALE_AGE_SEC);
        }

    } while (eink.nextPage());
}

// ── BLE telemetry ──

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
        uint16_t sent = telemetryChar.notify(json.c_str(), json.length());
        Serial.print("[BLE] notify ");
        Serial.print(json.length());
        Serial.print("B, sent=");
        Serial.println(sent);
    }
}

// ── Main ──

void setup() {
    pinMode(PIN_LED1, OUTPUT);
    Serial.begin(115200);
    delay(3000);

    blinkLED(1);

    // BLE/SoftDevice must init before any SPI peripherals to avoid
    // interrupt priority conflicts that cause sd_softdevice_enable() to hang
    Serial.println("[BOOT] Init BLE...");
    initializeBLE();

    initializeDisplay();

    bootStatus("Loading config...");
    loadConfig();

    bootStatus("Init LoRa...");
    initializeLoRa();

    memset(trackers, 0, sizeof(trackers));

    blinkLED(10, 100, 100);
    bootStatus("Ready!");
}

void loop() {
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
                displayNeedsUpdate = true;
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

        radio.startReceive();
        enableInterrupt = true;
    }

    if (loraReinitNeeded) {
        loraReinitNeeded = false;
        Serial.println("Reinitializing LoRa with new frequency...");
        initializeLoRa();
    }

    updateDisplay();
    updateBLE();

    delay(10);
}
