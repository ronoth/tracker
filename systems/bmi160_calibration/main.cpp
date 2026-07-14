#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "board.h"

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

// EEPROM addresses for calibration storage
#define EEPROM_MAGIC_ADDR     0
#define EEPROM_GYRO_X_ADDR    4
#define EEPROM_GYRO_Y_ADDR    8
#define EEPROM_GYRO_Z_ADDR    12
#define EEPROM_ACCEL_X_ADDR   16
#define EEPROM_ACCEL_Y_ADDR   20
#define EEPROM_ACCEL_Z_ADDR   24
#define EEPROM_MAGIC_VALUE    0x42494D49  // "BIMI" magic number

struct CalibrationData {
    float gyroX_bias;
    float gyroY_bias;
    float gyroZ_bias;
    float accelX_bias;
    float accelY_bias;
    float accelZ_bias;
};

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
    
    // Configure accelerometer: ±16g range, 100Hz for calibration
    writeBMI160Register(BMI160_ACC_RANGE, 0x0C);  // ±16g
    writeBMI160Register(BMI160_ACC_CONF, 0x28);   // 100Hz, normal mode
    
    // Configure gyroscope: ±2000°/s range, 100Hz for calibration
    writeBMI160Register(BMI160_GYR_RANGE, 0x00);  // ±2000°/s
    writeBMI160Register(BMI160_GYR_CONF, 0x28);   // 100Hz, normal mode
    
    delay(200);  // Let sensor stabilize
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

void saveCalibration(const CalibrationData& cal) {
    // Write magic number
    uint32_t magic = EEPROM_MAGIC_VALUE;
    EEPROM.put(EEPROM_MAGIC_ADDR, magic);
    
    // Write calibration data
    EEPROM.put(EEPROM_GYRO_X_ADDR, cal.gyroX_bias);
    EEPROM.put(EEPROM_GYRO_Y_ADDR, cal.gyroY_bias);
    EEPROM.put(EEPROM_GYRO_Z_ADDR, cal.gyroZ_bias);
    EEPROM.put(EEPROM_ACCEL_X_ADDR, cal.accelX_bias);
    EEPROM.put(EEPROM_ACCEL_Y_ADDR, cal.accelY_bias);
    EEPROM.put(EEPROM_ACCEL_Z_ADDR, cal.accelZ_bias);
    
    EEPROM.commit();
    Serial.println("Calibration saved to EEPROM");
}

bool loadCalibration(CalibrationData& cal) {
    uint32_t magic;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    
    if (magic != EEPROM_MAGIC_VALUE) {
        Serial.println("No valid calibration found in EEPROM");
        return false;
    }
    
    EEPROM.get(EEPROM_GYRO_X_ADDR, cal.gyroX_bias);
    EEPROM.get(EEPROM_GYRO_Y_ADDR, cal.gyroY_bias);
    EEPROM.get(EEPROM_GYRO_Z_ADDR, cal.gyroZ_bias);
    EEPROM.get(EEPROM_ACCEL_X_ADDR, cal.accelX_bias);
    EEPROM.get(EEPROM_ACCEL_Y_ADDR, cal.accelY_bias);
    EEPROM.get(EEPROM_ACCEL_Z_ADDR, cal.accelZ_bias);
    
    Serial.println("Calibration loaded from EEPROM");
    return true;
}

void performCalibration() {
    Serial.println("\n=== BMI160 CALIBRATION ===");
    Serial.println("IMPORTANT: Place the sensor on a flat, stable surface");
    Serial.println("Do NOT move the sensor during calibration!");
    Serial.println();
    
    for (int i = 10; i > 0; i--) {
        Serial.print("Starting calibration in ");
        Serial.print(i);
        Serial.println(" seconds...");
        delay(1000);
    }
    
    Serial.println("Calibrating... (30 seconds)");
    
    const int numSamples = 3000;  // 30 seconds at 100Hz
    long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    long accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;
    int validSamples = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        readBMI160Motion(&ax, &ay, &az, &gx, &gy, &gz);
        
        gyroX_sum += gx;
        gyroY_sum += gy;
        gyroZ_sum += gz;
        accelX_sum += ax;
        accelY_sum += ay;
        accelZ_sum += az;
        validSamples++;
        
        // Progress indicator
        if (i % 300 == 0) {
            Serial.print(".");
        }
        
        delay(10);  // 100Hz sampling
    }
    
    Serial.println();
    Serial.print("Collected ");
    Serial.print(validSamples);
    Serial.println(" samples");
    
    // Calculate average bias
    CalibrationData cal;
    cal.gyroX_bias = (float)gyroX_sum / validSamples;
    cal.gyroY_bias = (float)gyroY_sum / validSamples;
    cal.gyroZ_bias = (float)gyroZ_sum / validSamples;
    cal.accelX_bias = (float)accelX_sum / validSamples;
    cal.accelY_bias = (float)accelY_sum / validSamples;
    cal.accelZ_bias = (float)accelZ_sum / validSamples;

    // Auto-detect gravity axis: whichever axis has the largest absolute mean
    // has gravity on it. Subtract 1g (2048 LSB at ±16g) from that axis,
    // preserving sign so it works regardless of orientation.
    float absX = fabsf(cal.accelX_bias);
    float absY = fabsf(cal.accelY_bias);
    float absZ = fabsf(cal.accelZ_bias);

    if (absX > absY && absX > absZ) {
        cal.accelX_bias -= copysignf(2048.0f, cal.accelX_bias);
        Serial.println("Gravity detected on X axis");
    } else if (absY > absX && absY > absZ) {
        cal.accelY_bias -= copysignf(2048.0f, cal.accelY_bias);
        Serial.println("Gravity detected on Y axis");
    } else {
        cal.accelZ_bias -= copysignf(2048.0f, cal.accelZ_bias);
        Serial.println("Gravity detected on Z axis");
    }
    
    // Display results
    Serial.println("\nCalibration Results:");
    Serial.print("Gyro X bias: "); Serial.println(cal.gyroX_bias, 2);
    Serial.print("Gyro Y bias: "); Serial.println(cal.gyroY_bias, 2);
    Serial.print("Gyro Z bias: "); Serial.println(cal.gyroZ_bias, 2);
    Serial.print("Accel X bias: "); Serial.println(cal.accelX_bias, 2);
    Serial.print("Accel Y bias: "); Serial.println(cal.accelY_bias, 2);
    Serial.print("Accel Z bias: "); Serial.println(cal.accelZ_bias, 2);
    
    // Save to EEPROM
    saveCalibration(cal);
    Serial.println("\nCalibration complete!");
}

void testCalibration() {
    CalibrationData cal;
    if (!loadCalibration(cal)) {
        Serial.println("No calibration data available. Run calibration first.");
        return;
    }
    
    Serial.println("\n=== TESTING CALIBRATION ===");
    Serial.println("Calibrated data (should be close to zero for stationary sensor):");
    Serial.println("Raw_GX,Raw_GY,Raw_GZ,Cal_GX,Cal_GY,Cal_GZ,Raw_AX,Raw_AY,Raw_AZ,Cal_AX,Cal_AY,Cal_AZ");
    
    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        readBMI160Motion(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Apply calibration
        float cal_gx = gx - cal.gyroX_bias;
        float cal_gy = gy - cal.gyroY_bias;
        float cal_gz = gz - cal.gyroZ_bias;
        float cal_ax = ax - cal.accelX_bias;
        float cal_ay = ay - cal.accelY_bias;
        float cal_az = az - cal.accelZ_bias;
        
        // Print CSV format
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.print(gz); Serial.print(",");
        Serial.print(cal_gx, 1); Serial.print(",");
        Serial.print(cal_gy, 1); Serial.print(",");
        Serial.print(cal_gz, 1); Serial.print(",");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(cal_ax, 1); Serial.print(",");
        Serial.print(cal_ay, 1); Serial.print(",");
        Serial.println(cal_az, 1);
        
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("BMI160 Calibration Tool");
    Serial.println("=======================");
    
    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    delay(100);
    
    // Initialize EEPROM
    EEPROM.begin(512);
    
    // Initialize BMI160
    if (!initBMI160()) {
        Serial.println("BMI160 initialization failed!");
        Serial.println("Check wiring and I2C address (0x69)");
        while (1) delay(1000);
    }
    
    Serial.println("BMI160 detected successfully!");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  'c' - Perform calibration");
    Serial.println("  't' - Test current calibration");
    Serial.println("  'r' - Show raw sensor data");
    Serial.println("  's' - Show stored calibration");
    Serial.println();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'c':
            case 'C':
                performCalibration();
                break;
                
            case 't':
            case 'T':
                testCalibration();
                break;
                
            case 'r':
            case 'R':
                Serial.println("Raw sensor data (press any key to stop):");
                Serial.println("GX,GY,GZ,AX,AY,AZ");
                while (!Serial.available()) {
                    int16_t ax, ay, az, gx, gy, gz;
                    readBMI160Motion(&ax, &ay, &az, &gx, &gy, &gz);
                    Serial.print(gx); Serial.print(",");
                    Serial.print(gy); Serial.print(",");
                    Serial.print(gz); Serial.print(",");
                    Serial.print(ax); Serial.print(",");
                    Serial.print(ay); Serial.print(",");
                    Serial.println(az);
                    delay(100);
                }
                while (Serial.available()) Serial.read(); // Clear buffer
                break;
                
            case 's':
            case 'S':
                CalibrationData cal;
                if (loadCalibration(cal)) {
                    Serial.println("Stored calibration values:");
                    Serial.print("Gyro X bias: "); Serial.println(cal.gyroX_bias, 2);
                    Serial.print("Gyro Y bias: "); Serial.println(cal.gyroY_bias, 2);
                    Serial.print("Gyro Z bias: "); Serial.println(cal.gyroZ_bias, 2);
                    Serial.print("Accel X bias: "); Serial.println(cal.accelX_bias, 2);
                    Serial.print("Accel Y bias: "); Serial.println(cal.accelY_bias, 2);
                    Serial.print("Accel Z bias: "); Serial.println(cal.accelZ_bias, 2);
                }
                break;
                
            default:
                Serial.println("Unknown command. Use 'c', 't', 'r', or 's'");
                break;
        }
        
        Serial.println("\nReady for next command...");
    }
    
    delay(100);
}