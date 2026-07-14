#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DS33.h>
#include <MS5611.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include "board.h"
#include "flight_state.h"
#include "gps_data.h"
#include "sensor_data.h"
#include "lora_config.h"

// GPS Module
static const int RXPin = GPS_RX, TXPin = GPS_TX;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// SPI instances
SPIClass loraSPI(FSPI);  // Use FSPI for LoRa
SPIClass sdSPI(HSPI);    // Use HSPI for SD card

// LoRa radio
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);

// Sensors
Adafruit_MPL3115A2 mpl3115a2;
MS5611 ms5611;
Adafruit_LSM6DS33 lsm6ds;

// BMI160 Register addresses
#define BMI160_CHIP_ID          0x00
#define BMI160_ACC_CONF         0x40
#define BMI160_ACC_RANGE        0x41
#define BMI160_GYR_CONF         0x42
#define BMI160_GYR_RANGE        0x43
#define BMI160_CMD              0x7E
#define BMI160_DATA_8           0x0C  // Start of gyro/accel data (gyro at 0x0C, accel at 0x12)

// BMI160 Commands
#define BMI160_SOFT_RESET       0xB6
#define BMI160_ACC_NORMAL_MODE  0x11
#define BMI160_GYR_NORMAL_MODE  0x15

const uint8_t BMI160_ADDR = 0x69;

// BMI160 calibration values (hard-coded from calibration session)
const float BMI160_GYRO_X_BIAS = -1.18f;
const float BMI160_GYRO_Y_BIAS = 8.60f;
const float BMI160_GYRO_Z_BIAS = 4.06f;
const float BMI160_ACCEL_X_BIAS = 200.67f;
const float BMI160_ACCEL_Y_BIAS = -114.83f;
const float BMI160_ACCEL_Z_BIAS = 56.89f;

// BMI160 functions
void writeBMI160Register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readBMI160Register(uint8_t reg) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI160_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

void readBMI160Data(uint8_t reg, uint8_t* data, int length) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI160_ADDR, (uint8_t)length);
    for (int i = 0; i < length && Wire.available(); i++) {
        data[i] = Wire.read();
    }
}

bool initBMI160() {
    // Read chip ID
    uint8_t chip_id = readBMI160Register(BMI160_CHIP_ID);
    if (chip_id != 0xD1) {
        return false;
    }
    
    // Soft reset
    writeBMI160Register(BMI160_CMD, BMI160_SOFT_RESET);
    delay(100);
    
    // Set accelerometer to normal mode
    writeBMI160Register(BMI160_CMD, BMI160_ACC_NORMAL_MODE);
    delay(10);
    
    // Set gyroscope to normal mode  
    writeBMI160Register(BMI160_CMD, BMI160_GYR_NORMAL_MODE);
    delay(100);
    
    // Configure accelerometer: ±16g range, 26Hz
    writeBMI160Register(BMI160_ACC_RANGE, 0x0C);  // ±16g
    writeBMI160Register(BMI160_ACC_CONF, 0x27);   // 26Hz, normal mode
    
    // Configure gyroscope: ±2000°/s range, 26Hz
    writeBMI160Register(BMI160_GYR_RANGE, 0x00);  // ±2000°/s
    writeBMI160Register(BMI160_GYR_CONF, 0x27);   // 26Hz, normal mode
    
    delay(100);
    return true;
}

void readBMI160Motion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[12];
    readBMI160Data(BMI160_DATA_8, data, 12);
    
    // BMI160 data format: gyro first, then accel (little endian)
    *gx = (int16_t)(data[0] | (data[1] << 8));
    *gy = (int16_t)(data[2] | (data[3] << 8));
    *gz = (int16_t)(data[4] | (data[5] << 8));
    *ax = (int16_t)(data[6] | (data[7] << 8));
    *ay = (int16_t)(data[8] | (data[9] << 8));
    *az = (int16_t)(data[10] | (data[11] << 8));
}

// OLED Display (using secondary I2C bus Wire1)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RST);

// SD Card
SdFat sd;
FatFile dataFile;
const char* LOG_FILENAME = "flight_data.csv";

// Timing control
const float SAMPLE_RATE_HZ = 26.0f;
const unsigned long SAMPLE_INTERVAL = (unsigned long)(1000.0f / SAMPLE_RATE_HZ);
const unsigned long LORA_INTERVAL = 3000; // Transmit GPS every 3 seconds
const unsigned long DISPLAY_INTERVAL = 1000; // Update display every second
const unsigned long GPS_FIX_STALE_MS = 2000; // Fix older than this is no longer "current"
const int GPS_MIN_SATS = 3; // Reject "fixes" the module reports with fewer sats in use
unsigned long lastSampleTime = 0;
unsigned long lastLoRaTime = 0;
unsigned long lastDisplayTime = 0;

// Data structures
GPSData gpsData;
SensorData sensorData;

// Status flags
bool sdInitialized = false;
bool sensorsInitialized = false;
bool imuInitialized = false;
bool loraInitialized = false;
bool displayInitialized = false;
bool logFileReady = false;
bool gpsTimeValid = false;

// ── BLE config service (name + frequency, like the ground stations) ──
// Distinct service UUID so the mobile app can tell trackers from ground stations
#define BLE_SERVICE_UUID        "5fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_NAME_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_FREQ_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26aa"

Preferences preferences;
char trackerName[32] = "";
char trackerId[8] = ""; // 6 hex digits of the eFuse MAC — stable hardware identity
float loraFrequency = LORA_DEFAULT_FREQUENCY; // MHz, overridden from preferences
// Flag to reinitialize LoRa from main loop (cannot do SPI from BLE callback)
volatile bool loraReinitNeeded = false;

// BLE runs only during a config window after power-on, then shuts down to
// save battery and CPU for flight — unless a client is already connected
const unsigned long BLE_CONFIG_WINDOW_MS = 120000;
bool bleActive = false;
volatile bool bleClientConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        bleClientConnected = true;
        Serial.println("BLE client connected");
    }
    void onDisconnect(BLEServer* pServer) override {
        bleClientConnected = false;
        Serial.println("BLE client disconnected");
        // Re-advertise only inside the config window; after it closes the
        // main loop shuts BLE down instead
        if (millis() < BLE_CONFIG_WINDOW_MS) {
            pServer->startAdvertising();
        }
    }
};

class NameCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0 && value.length() < sizeof(trackerName)) {
            strncpy(trackerName, value.c_str(), sizeof(trackerName) - 1);
            trackerName[sizeof(trackerName) - 1] = '\0';
            // Name travels as a CSV field in the LoRa packet and a JSON string
            // in ground station telemetry — strip the delimiters
            for (char* c = trackerName; *c; c++) {
                if (*c == ',' || *c == '"' || *c == '\\') *c = ' ';
            }
            Serial.print("Tracker name set: ");
            Serial.println(trackerName);
            preferences.begin("config", false);
            preferences.putString("name", trackerName);
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
                String freqStr = String(loraFrequency, 1);
                pCharacteristic->setValue(freqStr.c_str());
                loraReinitNeeded = true;
            }
        }
    }
};

void loadConfig() {
    // Unique-per-chip ID: the NIC-specific last 3 bytes of the base MAC
    snprintf(trackerId, sizeof(trackerId), "%06X",
             (uint32_t)(ESP.getEfuseMac() >> 24) & 0xFFFFFF);

    preferences.begin("config", true);
    loraFrequency = preferences.getFloat("frequency", LORA_DEFAULT_FREQUENCY);
    String savedName = preferences.getString("name", "");
    if (savedName.length() > 0 && savedName.length() < sizeof(trackerName)) {
        strncpy(trackerName, savedName.c_str(), sizeof(trackerName) - 1);
        trackerName[sizeof(trackerName) - 1] = '\0';
    }
    preferences.end();
    Serial.print("Config: id=");
    Serial.print(trackerId);
    Serial.print(", freq=");
    Serial.print(loraFrequency, 1);
    Serial.print(" MHz, name=");
    Serial.println(trackerName[0] ? trackerName : "(none)");
}

void initializeBLE() {
    // Advertise the hardware ID when unnamed so scan lists stay distinguishable
    char defaultName[24];
    snprintf(defaultName, sizeof(defaultName), "Tracker-%s", trackerId);
    BLEDevice::init(trackerName[0] ? trackerName : defaultName);
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    BLECharacteristic* pNameChar = pService->createCharacteristic(
        BLE_NAME_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pNameChar->setCallbacks(new NameCharCallbacks());

    BLECharacteristic* pFreqChar = pService->createCharacteristic(
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
    bleActive = true;
    Serial.println("BLE advertising started (config window 120s)");
}

// Display sleep
bool displayAsleep = false;
unsigned long displayOnTime = 0;
const unsigned long DISPLAY_SLEEP_TIMEOUT = 60000; // 60 seconds
volatile bool buttonPressed = false;

void IRAM_ATTR buttonISR() {
    buttonPressed = true;
}

// Flight state machine
FlightStateMachine fsm;
FlightEventType lastFrameEvent = EVENT_NONE;

void onFlightEvent(const FlightEvent& event) {
    // Log event to SD if file is ready
    if (logFileReady) {
        char eventLine[80];
        snprintf(eventLine, sizeof(eventLine), "# EVENT: %s t=%lu val=%.2f\n",
            flightEventName(event.type), event.timestamp, event.value);
        dataFile.write(eventLine);
        dataFile.sync();
    }
}

// Sensor type detection
enum BarometerType {
    BARO_NONE,
    BARO_MPL3115A2,
    BARO_MS5611
};
BarometerType barometerType = BARO_NONE;

enum IMUType {
    IMU_NONE,
    IMU_LSM6DS33,
    IMU_BMI160
};
IMUType imuType = IMU_NONE;

#ifndef BOARD_V4 // no SD card on V4; its former SD pins (2/46) control the RF front-end
void initializeSD() {
    // Initialize dedicated SPI for SD card
    sdSPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

    // Create SPI configuration for custom SPI instance
    SdSpiConfig spiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(25), &sdSPI);

    if (!sd.begin(spiConfig)) {
        Serial.println("SD card initialization failed!");
        return;
    }

    sdInitialized = true;
    Serial.println("SD card hardware initialized - waiting for GPS time to create log file");
}
#endif

void createLogFile() {
    if (!sdInitialized || !gpsTimeValid) return;
    
    // Generate filename based on GPS date/time
    char filename[32];
    snprintf(filename, sizeof(filename), "flight_%04d%02d%02d_%02d%02d%02d.csv",
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
    
    // Create/open data file
    if (!dataFile.open(filename, O_WRITE | O_CREAT | O_EXCL)) {
        Serial.print("Failed to create log file: ");
        Serial.println(filename);
        return;
    }
    
    // Write configuration header as comments
    char configLine[150];
    
    // Flight date and time
    snprintf(configLine, sizeof(configLine), "# Flight Date: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
    dataFile.write(configLine);
    
    // LoRa configuration
    snprintf(configLine, sizeof(configLine), "# LoRa Config: %.1fMHz, 125kHz BW, SF9, CR7, 22dBm, Private Sync\n", loraFrequency);
    dataFile.write(configLine);
    
    // Sample rate
    snprintf(configLine, sizeof(configLine), "# Sample Rate: %.1f Hz\n", SAMPLE_RATE_HZ);
    dataFile.write(configLine);
    
    // IMU sensor type
    if (imuType == IMU_LSM6DS33) {
        snprintf(configLine, sizeof(configLine), "# IMU: LSM6DS33 (±16g, ±2000°/s, 26Hz)\n");
    } else if (imuType == IMU_BMI160) {
        snprintf(configLine, sizeof(configLine), "# IMU: BMI160 (±16g, ±2000°/s, 26Hz)\n");
    } else {
        snprintf(configLine, sizeof(configLine), "# IMU: None detected\n");
    }
    dataFile.write(configLine);
    
    // Barometer sensor type
    if (barometerType == BARO_MPL3115A2) {
        snprintf(configLine, sizeof(configLine), "# Barometer: MPL3115A2\n");
    } else if (barometerType == BARO_MS5611) {
        snprintf(configLine, sizeof(configLine), "# Barometer: MS5611\n");
    } else {
        snprintf(configLine, sizeof(configLine), "# Barometer: None detected\n");
    }
    dataFile.write(configLine);
    
    // Hardware info
    snprintf(configLine, sizeof(configLine), "# Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3)\n");
    dataFile.write(configLine);
    
    // Data format info
    snprintf(configLine, sizeof(configLine), "# Units: timestamp(ms), GPS(decimal degrees, m, km/h), pressure(Pa), temp(°C), accel(m/s²), gyro(rad/s), battery(V)\n");
    dataFile.write(configLine);

    // Write CSV header
    dataFile.write("timestamp,state,event,lat,lon,alt,speed,sats,hdop,pressure,temp,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,battV\n");
    dataFile.sync();
    
    logFileReady = true;
    Serial.print("Created log file: ");
    Serial.println(filename);
}

void initializeSensors() {
    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(800000); // 800kHz for high-speed logging
    delay(100);

    // Try to detect barometer sensor
    Serial.println("Detecting barometer sensor...");
    
    // First try MPL3115A2
    if (mpl3115a2.begin()) {
        barometerType = BARO_MPL3115A2;
        mpl3115a2.setMode(MPL3115A2_BAROMETER);
        mpl3115a2.write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_OS2 | MPL3115A2_CTRL_REG1_BAR);
        Serial.println("MPL3115A2 barometer detected and configured");
    }
    // If MPL3115A2 not found, try MS5611
    else if (ms5611.begin()) {
        barometerType = BARO_MS5611;
        Serial.println("MS5611 barometer detected and configured");
    }
    else {
        barometerType = BARO_NONE;
        Serial.println("No barometer sensor found!");
    }

    // Try to detect IMU sensor
    Serial.println("Detecting IMU sensor...");
    
    // First try LSM6DS33 at 0x6B
    if (lsm6ds.begin_I2C(0x6B)) {
        imuType = IMU_LSM6DS33;
        // Configure LSM6DS for 26Hz sampling
        lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
        lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
        lsm6ds.setAccelDataRate(LSM6DS_RATE_26_HZ);
        lsm6ds.setGyroDataRate(LSM6DS_RATE_26_HZ);
        imuInitialized = true;
        Serial.println("LSM6DS33 IMU detected and configured (±16g, ±2000°/s, 26Hz)");
    }
    // If LSM6DS33 not found, try BMI160 at 0x69
    else if (initBMI160()) {
        imuType = IMU_BMI160;
        imuInitialized = true;
        Serial.println("BMI160 IMU detected and configured (±16g, ±2000°/s, 26Hz)");
    }
    else {
        imuType = IMU_NONE;
        imuInitialized = false;
        Serial.println("No IMU sensor found!");
    }

    // Set sensorsInitialized if we have at least one working sensor
    sensorsInitialized = (barometerType != BARO_NONE) || imuInitialized;
    
    if (sensorsInitialized) {
        Serial.println("Sensors initialized successfully");
    } else {
        Serial.println("Warning: No sensors detected!");
    }
}

void initializeDisplay() {
    Serial.println("Initializing secondary I2C bus for OLED");
    
    // Enable OLED reset pin (from i2c utility example)
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, HIGH);
    
    // Initialize secondary I2C bus for OLED
    Wire1.begin(OLED_I2C_SDA, OLED_I2C_SCL);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        return;
    }
    
    displayInitialized = true;
    
    // Show startup message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
    
    Serial.println("OLED display initialized");
}

void initializeLoRa() {
    Serial.println("Initializing LoRa radio");

#ifdef BOARD_V4
    // V4 routes the SX1262 through a GC1109 front-end module; without these
    // the FEM is unpowered and TX/RX are severely attenuated.
    pinMode(FEM_POWER, OUTPUT);
    digitalWrite(FEM_POWER, HIGH);
    pinMode(FEM_CSD, OUTPUT);
    digitalWrite(FEM_CSD, HIGH);
    pinMode(FEM_CPS, OUTPUT);
    digitalWrite(FEM_CPS, HIGH);
#endif

    // Initialize dedicated SPI for LoRa
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    // Initialize LoRa radio for transmission (exact match to working main.cpp)
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
    
    radio.setDio2AsRfSwitch(true);
    // RadioLib defaults OCP to 60mA; +22dBm draws ~120mA and folds back without this
    radio.setCurrentLimit(140.0);
    loraInitialized = true;
    Serial.println("LoRa radio initialized");
}

void initializeBattery() {
    // Enable the VBAT divider (ADC_CTRL polarity differs per board); the
    // divider's drain is ~9uA so it stays on. Low attenuation to match the
    // high-resistance 390k/100k divider.
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, ADC_CTRL_ON);
    analogSetPinAttenuation(VBAT_SENSE, ADC_2_5db);
}

void updateBatteryVoltage() {
    uint32_t mv = 0;
    for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(VBAT_SENSE);
    gpsData.batteryVoltage = (mv / 4) * ADC_SCALER / 1000.0f;
}

void updateGPSData() {
    // Read GPS data
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    // TinyGPS++ isValid() latches true forever once any position is parsed,
    // and the GPS module can replay its battery-backed last position at warm
    // start — gate on commit age and sats-in-use so only a live fix counts.
    gpsData.hasGPSFix = gps.location.isValid()
                        && gps.location.age() < GPS_FIX_STALE_MS
                        && gps.satellites.value() >= GPS_MIN_SATS;
    if (gpsData.hasGPSFix) {
        gpsData.latitude = gps.location.lat();
        gpsData.longitude = gps.location.lng();
        gpsData.lastGPSFixTime = millis();
    }

    gpsData.altitudeValid = gps.altitude.isValid() && gps.altitude.age() < GPS_FIX_STALE_MS;
    if (gpsData.altitudeValid) {
        gpsData.altitude = gps.altitude.meters();
    }

    gpsData.speedValid = gps.speed.isValid() && gps.speed.age() < GPS_FIX_STALE_MS;
    if (gpsData.speedValid) {
        gpsData.speed = gps.speed.kmph();
    }

    gpsData.satellites = gps.satellites.value();
    gpsData.hdop = gps.hdop.hdop();
    
    // Check if GPS time is valid and create log file if needed
    if (!gpsTimeValid && gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2020) {
        gpsTimeValid = true;
        Serial.println("GPS time acquired - creating log file");
        createLogFile();
        fsm.setGPSReady();
    }
}

void readSensors() {
    // Read barometer based on detected sensor type
    if (barometerType == BARO_MPL3115A2) {
        sensorData.pressure = mpl3115a2.getPressure();
        sensorData.temperature = mpl3115a2.getTemperature();
    }
    else if (barometerType == BARO_MS5611) {
        int result = ms5611.read();
        if (result == MS5611_READ_OK) {
            sensorData.pressure = ms5611.getPressure();
            sensorData.temperature = ms5611.getTemperature();
        } else {
            // Read failed, use previous values or set to 0
            sensorData.pressure = 0;
            sensorData.temperature = 0;
        }
    }
    else {
        // No barometer detected
        sensorData.pressure = 0;
        sensorData.temperature = 0;
    }
    
    // Read IMU based on detected sensor type
    if (imuType == IMU_LSM6DS33) {
        sensors_event_t accel, gyro, temp;
        lsm6ds.getEvent(&accel, &gyro, &temp);
        
        sensorData.accelX = accel.acceleration.x;
        sensorData.accelY = accel.acceleration.y;
        sensorData.accelZ = accel.acceleration.z;
        sensorData.gyroX = gyro.gyro.x;
        sensorData.gyroY = gyro.gyro.y;
        sensorData.gyroZ = gyro.gyro.z;
    }
    else if (imuType == IMU_BMI160) {
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        readBMI160Motion(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        
        // Apply calibration to raw values
        float ax_cal = ax_raw - BMI160_ACCEL_X_BIAS;
        float ay_cal = ay_raw - BMI160_ACCEL_Y_BIAS;
        float az_cal = az_raw - BMI160_ACCEL_Z_BIAS;
        float gx_cal = gx_raw - BMI160_GYRO_X_BIAS;
        float gy_cal = gy_raw - BMI160_GYRO_Y_BIAS;
        float gz_cal = gz_raw - BMI160_GYRO_Z_BIAS;
        
        // Convert calibrated values to m/s² and rad/s
        // BMI160 ±16g range: 32768 LSB = 16g, so 1g = 2048 LSB, 1 m/s² = 208.7 LSB
        sensorData.accelX = ax_cal / 208.7f * 9.81f;  // Convert to m/s²
        sensorData.accelY = ay_cal / 208.7f * 9.81f;
        sensorData.accelZ = az_cal / 208.7f * 9.81f;
        
        // BMI160 ±2000°/s range: 32768 LSB = 2000°/s, so 1°/s = 16.384 LSB
        sensorData.gyroX = gx_cal / 16.384f * PI / 180.0f;  // Convert to rad/s
        sensorData.gyroY = gy_cal / 16.384f * PI / 180.0f;
        sensorData.gyroZ = gz_cal / 16.384f * PI / 180.0f;
    }
    else {
        // No IMU available, set to zero
        sensorData.accelX = sensorData.accelY = sensorData.accelZ = 0;
        sensorData.gyroX = sensorData.gyroY = sensorData.gyroZ = 0;
    }
    
    sensorData.timestamp = millis();
}

void logDataToSD() {
    if (!logFileReady) return;

    // Create CSV line with state and event columns
    char csvLine[220];
    snprintf(csvLine, sizeof(csvLine),
        "%lu,%d,%d,%.6f,%.6f,%.1f,%.1f,%d,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
        sensorData.timestamp,
        (int)fsm.getState(),
        (int)lastFrameEvent,
        gpsData.hasGPSFix ? gpsData.latitude : 0.0,
        gpsData.hasGPSFix ? gpsData.longitude : 0.0,
        gpsData.altitudeValid ? gpsData.altitude : 0.0,
        gpsData.speedValid ? gpsData.speed : 0.0,
        gpsData.satellites,
        gpsData.hdop,
        sensorData.pressure,
        sensorData.temperature,
        sensorData.accelX,
        sensorData.accelY,
        sensorData.accelZ,
        sensorData.gyroX,
        sensorData.gyroY,
        sensorData.gyroZ,
        gpsData.batteryVoltage
    );

    // Write to file
    dataFile.write(csvLine);

    // Sync to ensure data is written
    dataFile.sync();
}

void updateDisplay() {
    if (!displayInitialized) return;

    // Handle button press to wake display
    if (buttonPressed) {
        buttonPressed = false;
        if (displayAsleep) {
            displayAsleep = false;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        displayOnTime = millis();
    }

    // Sleep display after timeout
    if (!displayAsleep && (millis() - displayOnTime >= DISPLAY_SLEEP_TIMEOUT)) {
        displayAsleep = true;
        display.clearDisplay();
        display.display();
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        return;
    }

    if (displayAsleep) return;

    // In RECOVERY state, disable display to save power
    if (fsm.getState() == STATE_RECOVERY) {
        display.clearDisplay();
        display.display();
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Flight state (prominent, top line)
    display.print("STATE: ");
    display.println(flightStateName(fsm.getState()));

    // GPS Status
    display.print("GPS: ");
    if (gpsData.hasGPSFix) {
        display.print("FIX ");
        display.print("Sats:");
        display.print(gpsData.satellites);
        display.print(" H:");
        display.println(gpsData.hdop, 1);
    } else if (gpsData.lastGPSFixTime > 0) {
        uint32_t locAge = (millis() - gpsData.lastGPSFixTime) / 1000;
        display.print("LOST(");
        display.print(locAge);
        display.println("s)");
        display.print("LAT:");
        display.print(gpsData.latitude, 4);
        display.print(" LON:");
        display.println(gpsData.longitude, 4);
    } else {
        display.print("--- ");
        display.print("Sats:");
        display.print(gpsData.satellites);
        display.print(" H:");
        display.println(gpsData.hdop, 1);
    }

    // Logging and system status
    display.print("Log:");
    display.print(logFileReady ? "ON" : "--");
    display.print(" SD:");
    display.print(sdInitialized ? "OK" : "NO");
    display.print(" LoRa:");
    display.println(loraInitialized ? "OK" : "NO");

    // Sensors
    display.print("IMU:");
    if (imuType == IMU_LSM6DS33) display.print("LSM ");
    else if (imuType == IMU_BMI160) display.print("BMI ");
    else display.print("--- ");
    display.print("Baro:");
    if (barometerType == BARO_MPL3115A2) display.println("MPL");
    else if (barometerType == BARO_MS5611) display.println("MS5");
    else display.println("---");

    // Sensor readings
    if (sensorsInitialized) {
        display.print("P:");
        display.print(sensorData.pressure, 0);
        display.print(" T:");
        display.print(sensorData.temperature, 1);
        display.println("C");
    }

    // Altitude if available
    if (gpsData.altitudeValid) {
        display.print("Alt:");
        display.print(gpsData.altitude, 0);
        display.print("m");
        if (gpsData.speedValid) {
            display.print(" Spd:");
            display.print(gpsData.speed, 0);
        }
        display.println();
    }

    // LoRa frequency (MHz) + battery — no room for a dedicated line when the
    // lost-fix layout uses all 8 rows
    display.print("LoRa: ");
    display.print(loraFrequency, 1);
    display.print(" B:");
    display.print(gpsData.batteryVoltage, 2);
    display.println("V");

    display.display();
}

void transmitGPSData() {
    if (!loraInitialized) return;

    // Hardware ID + tracker name ride as the final fields so ground stations
    // can tell multiple trackers apart. ID is the stable key; name is empty
    // unless one was manually assigned — the UIs fall back to the ID.
    const char* name = trackerName;
    updateBatteryVoltage();
    char lora_buff[120];
    if (gpsData.hasGPSFix) {
        snprintf(lora_buff, sizeof(lora_buff),
            "GPS:%d,%.6f,%.6f,%.1f,%.1f,%d,%.1f,%s,%s,%.2f",
            (int)fsm.getState(),
            gpsData.latitude,
            gpsData.longitude,
            gpsData.altitudeValid ? gpsData.altitude : 0.0,
            gpsData.speedValid ? gpsData.speed : 0.0,
            gpsData.satellites,
            gpsData.hdop,
            trackerId,
            name,
            gpsData.batteryVoltage
        );
    } else {
        snprintf(lora_buff, sizeof(lora_buff),
            "GPS:%d,NoFix,%d,%.1f,%s,%s,%.2f",
            (int)fsm.getState(),
            gpsData.satellites,
            gpsData.hdop,
            trackerId,
            name,
            gpsData.batteryVoltage
        );
    }

    Serial.print("Transmitting: ");
    Serial.println(lora_buff);
    radio.startTransmit(lora_buff, strlen(lora_buff));
}

void setup() {
    Serial.begin(115200);
    Serial.println("Vehicle Telemetry System Starting...");

    // Initialize flight state machine
    fsm.begin(true);  // auto-arm enabled, no hardware pin
    fsm.onEvent(onFlightEvent);

    // Initialize user button for display wake
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON), buttonISR, FALLING);

    // Initialize GPS
#ifdef BOARD_V4
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
    pinMode(GPS_RESET, OUTPUT);
    digitalWrite(GPS_RESET, HIGH);
    delay(1000);
#endif
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Serial.println("GPS initialized");

    // Load persisted config (LoRa frequency, tracker name) before radio init
    loadConfig();

    initializeBattery();

    // Initialize all subsystems
#ifndef BOARD_V4
    initializeSD();
#endif
    initializeSensors();
    initializeDisplay();
    initializeLoRa();
    initializeBLE();

    displayOnTime = millis();

    // Transition FSM past INITIALIZING
    fsm.setInitialized();

    // Send first packet immediately so ground stations don't wait
    transmitGPSData();
    lastLoRaTime = millis();

    Serial.println("Vehicle system ready!");
    Serial.println("Logging format: timestamp,state,event,lat,lon,alt,speed,sats,hdop,pressure,temp,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,battV");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Update GPS data continuously
    updateGPSData();
    
    // Check for GPS timeout
    if (currentTime > 5000 && gps.charsProcessed() < 10) {
        Serial.println("No GPS detected");
    }

    // Sample sensors at 26Hz
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentTime;

        if (sensorsInitialized) {
            readSensors();

            // Compute acceleration magnitude for state machine
            float accelMag = sqrtf(
                sensorData.accelX * sensorData.accelX +
                sensorData.accelY * sensorData.accelY +
                sensorData.accelZ * sensorData.accelZ
            );

            // Update flight state machine
            fsm.update(accelMag, sensorData.pressure,
                       gpsData.altitudeValid ? gpsData.altitude : 0.0f,
                       gpsData.speedValid ? gpsData.speed / 3.6f : 0.0f,  // km/h to m/s
                       currentTime);

            lastFrameEvent = fsm.getLastEvent();

            logDataToSD();

            // Print CSV data for analysis
            Serial.print(sensorData.timestamp);
            Serial.print(",");
            Serial.print((int)fsm.getState());
            Serial.print(",");
            Serial.print((int)lastFrameEvent);
            Serial.print(",");
            Serial.print(sensorData.pressure, 2);
            Serial.print(",");
            Serial.print(sensorData.temperature, 2);
            Serial.print(",");
            Serial.print(sensorData.accelX, 3);
            Serial.print(",");
            Serial.print(sensorData.accelY, 3);
            Serial.print(",");
            Serial.print(sensorData.accelZ, 3);
            Serial.print(",");
            Serial.print(sensorData.gyroX, 4);
            Serial.print(",");
            Serial.print(sensorData.gyroY, 4);
            Serial.print(",");
            Serial.println(sensorData.gyroZ, 4);
        }
    }

    // Close the BLE config window: after 120s with no client connected,
    // shut BLE down entirely (power cycle to reconfigure)
    if (bleActive && !bleClientConnected && currentTime >= BLE_CONFIG_WINDOW_MS) {
        bleActive = false;
        Serial.println("BLE config window closed - shutting down BLE");
        BLEDevice::deinit(true);
    }

    // Reinitialize LoRa if frequency was changed via BLE
    if (loraReinitNeeded) {
        loraReinitNeeded = false;
        Serial.println("Reinitializing LoRa with new frequency...");
        initializeLoRa();
    }

    // Transmit GPS data via LoRa (faster in RECOVERY for location tracking)
    unsigned long loraInterval = (fsm.getState() == STATE_RECOVERY) ? 1000 : LORA_INTERVAL;
    if (currentTime - lastLoRaTime >= loraInterval) {
        lastLoRaTime = currentTime;
        transmitGPSData();
    }

    // Update display every second
    if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = currentTime;
        updateDisplay();
    }
}