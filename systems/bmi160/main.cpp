#include <Arduino.h>
#include <Wire.h>
#include "board.h"

// BMI160 Register addresses
#define BMI160_CHIP_ID          0x00
#define BMI160_ACC_CONF         0x40
#define BMI160_ACC_RANGE        0x41
#define BMI160_GYR_CONF         0x42
#define BMI160_GYR_RANGE        0x43
#define BMI160_CMD              0x7E
#define BMI160_DATA_14          0x12  // Start of accel/gyro data

// BMI160 Commands
#define BMI160_SOFT_RESET       0xB6
#define BMI160_ACC_NORMAL_MODE  0x11
#define BMI160_GYR_NORMAL_MODE  0x15

const uint8_t BMI160_ADDR = 0x69;

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
    Serial.println("Manual BMI160 initialization...");
    
    // Read chip ID
    uint8_t chip_id = readBMI160Register(BMI160_CHIP_ID);
    Serial.print("Chip ID: 0x");
    Serial.println(chip_id, HEX);
    
    if (chip_id != 0xD1) {
        Serial.println("BMI160 not found!");
        return false;
    }
    
    // Soft reset
    Serial.println("Performing soft reset...");
    writeBMI160Register(BMI160_CMD, BMI160_SOFT_RESET);
    delay(100);
    
    // Set accelerometer to normal mode
    Serial.println("Setting accelerometer to normal mode...");
    writeBMI160Register(BMI160_CMD, BMI160_ACC_NORMAL_MODE);
    delay(10);
    
    // Set gyroscope to normal mode  
    Serial.println("Setting gyroscope to normal mode...");
    writeBMI160Register(BMI160_CMD, BMI160_GYR_NORMAL_MODE);
    delay(100);
    
    // Configure accelerometer: ±16g range, 25Hz
    Serial.println("Configuring accelerometer: ±16g range...");
    writeBMI160Register(BMI160_ACC_RANGE, 0x0C);  // ±16g
    writeBMI160Register(BMI160_ACC_CONF, 0x26);   // 25Hz, normal mode
    
    // Configure gyroscope: ±250°/s range, 25Hz
    Serial.println("Configuring gyroscope: ±250°/s range...");
    writeBMI160Register(BMI160_GYR_RANGE, 0x03);  // ±250°/s
    writeBMI160Register(BMI160_GYR_CONF, 0x26);   // 25Hz, normal mode
    
    delay(100);
    Serial.println("BMI160 initialization complete!");
    return true;
}

void readBMI160Motion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[12];
    readBMI160Data(BMI160_DATA_14, data, 12);
    
    // BMI160 data format: gyro first, then accel (little endian)
    *gx = (int16_t)(data[0] | (data[1] << 8));
    *gy = (int16_t)(data[2] | (data[3] << 8));
    *gz = (int16_t)(data[4] | (data[5] << 8));
    *ax = (int16_t)(data[6] | (data[7] << 8));
    *ay = (int16_t)(data[8] | (data[9] << 8));
    *az = (int16_t)(data[10] | (data[11] << 8));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("BMI160 Manual Implementation Test");
    Serial.println("=================================");

    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz
    delay(100);

    // Test I2C communication
    Serial.println("Testing I2C communication at 0x69...");
    Wire.beginTransmission(0x69);
    uint8_t i2c_error = Wire.endTransmission();
    if (i2c_error == 0) {
        Serial.println("I2C device found at 0x69 - Good!");
    } else {
        Serial.print("I2C error code: ");
        Serial.println(i2c_error);
        while (1) delay(1000);
    }

    // Initialize BMI160 manually
    if (!initBMI160()) {
        Serial.println("BMI160 initialization failed!");
        while (1) delay(1000);
    }

    Serial.println("\nData Format:");
    Serial.println("Time(ms),AccelX(raw),AccelY(raw),AccelZ(raw),GyroX(raw),GyroY(raw),GyroZ(raw)");
    Serial.println("==============================================================================");
    
    delay(1000);
}

void loop() {
    static unsigned long lastReading = 0;
    static unsigned long lastBenchmark = 0;
    static int sampleCount = 0;
    
    unsigned long currentTime = millis();
    
    // Read sensor at 25Hz (40ms interval)
    if (currentTime - lastReading >= 40) {
        lastReading = currentTime;
        sampleCount++;
        
        // Read both accelerometer and gyroscope data
        int16_t ax, ay, az, gx, gy, gz;
        readBMI160Motion(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Print CSV data (raw values)
        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(ax);
        Serial.print(",");
        Serial.print(ay);
        Serial.print(",");
        Serial.print(az);
        Serial.print(",");
        Serial.print(gx);
        Serial.print(",");
        Serial.print(gy);
        Serial.print(",");
        Serial.println(gz);
    }

    // Print sample rate every second
    if (currentTime - lastBenchmark >= 1000) {
        Serial.print("Sample rate: ");
        Serial.print(sampleCount);
        Serial.println(" Hz");
        sampleCount = 0;
        lastBenchmark = currentTime;
    }
}