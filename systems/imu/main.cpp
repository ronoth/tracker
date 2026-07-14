#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include "board.h"

class IMU {
private:
    TwoWire* _wire;
    Adafruit_LSM6DS33 _lsm6ds;
    bool _lsm6ds_found = false;
    
    // Sensor data structures
    struct IMUData {
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        unsigned long timestamp;
    } _last_reading;

public:
    IMU(TwoWire* wire = &Wire) : _wire(wire) {}

    bool begin() {
        // Initialize I2C
        _wire->begin(I2C_SDA, I2C_SCL);
        _wire->setClock(800000); // 800kHz
        delay(100); // Give I2C time to stabilize

        // Scan for LSM6DS33
        _wire->beginTransmission(0x6B);
        if (_wire->endTransmission() == 0) {
            if (_lsm6ds.begin_I2C(0x6B, _wire)) {
                _lsm6ds_found = true;
                // Configure LSM6DS
                _lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
                _lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
                _lsm6ds.setAccelDataRate(LSM6DS_RATE_26_HZ);
                _lsm6ds.setGyroDataRate(LSM6DS_RATE_26_HZ);
                Serial.println("LSM6DS33 found and configured");
            }
        }

        return _lsm6ds_found;
    }

    bool read(IMUData& data) {
        data.timestamp = millis();
        
        if (_lsm6ds_found) {
            sensors_event_t accel, gyro, temp;
            _lsm6ds.getEvent(&accel, &gyro, &temp);
            
            data.accel_x = accel.acceleration.x;
            data.accel_y = accel.acceleration.y;
            data.accel_z = accel.acceleration.z;
            data.gyro_x = gyro.gyro.x;
            data.gyro_y = gyro.gyro.y;
            data.gyro_z = gyro.gyro.z;
        } else {
            data.accel_x = data.accel_y = data.accel_z = 0;
            data.gyro_x = data.gyro_y = data.gyro_z = 0;
        }

        _last_reading = data;
        return _lsm6ds_found;
    }

    const IMUData& getLastReading() const {
        return _last_reading;
    }

    bool isAvailable() const { return _lsm6ds_found; }
};

// Example usage
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);

    IMU imu;
    if (!imu.begin()) {
        Serial.println("No IMU sensor found!");
        while (1);
    }

    Serial.println("IMU initialized!");
    Serial.println("Time(ms),AccelX(m/s^2),AccelY(m/s^2),AccelZ(m/s^2),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s)");
}

void loop() {
    static IMU imu;
    static IMU::IMUData data;
    
    if (imu.read(data)) {
        Serial.print(data.timestamp);
        Serial.print(",");
        Serial.print(data.accel_x);
        Serial.print(",");
        Serial.print(data.accel_y);
        Serial.print(",");
        Serial.print(data.accel_z);
        Serial.print(",");
        Serial.print(data.gyro_x);
        Serial.print(",");
        Serial.print(data.gyro_y);
        Serial.print(",");
        Serial.println(data.gyro_z);
    }
    
    delay(1000 / 26); // Match the 26Hz sample rate
}
