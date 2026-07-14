#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DS33.h>
#include "board.h"

// Sensor objects
Adafruit_MPL3115A2 baro;
Adafruit_LSM6DS33 lsm6ds;

// Timing control
const float SAMPLE_RATE_HZ = 26.0f; // Match LSM6DS sample rate
const unsigned long SAMPLE_INTERVAL = (unsigned long)(1000.0f / SAMPLE_RATE_HZ); // Convert Hz to ms interval
unsigned long lastSampleTime = 0;

// Timing diagnostics
unsigned long baroStartTime = 0;
unsigned long baroEndTime = 0;
unsigned long lsmStartTime = 0;
unsigned long lsmEndTime = 0;
unsigned long lastDiagnosticTime = 0;
const unsigned long DIAGNOSTIC_INTERVAL = 1000; // Print diagnostics every second

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Sensor Logger Starting...");

    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100); // Give power time to stabilize

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(800000); // Set I2C clock to 800kHz
    delay(100);

    // Initialize MPL3115A2
    if (!baro.begin()) {
        Serial.println("Could not find MPL3115A2 sensor");
        while (1);
    }
    baro.setMode(MPL3115A2_BAROMETER);
    // Sets the oversampling rate which determines the accuraccy and speed of the sensor
    baro.write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_OS2 | MPL3115A2_CTRL_REG1_BAR);
    Serial.println("MPL3115A2 initialized!");

    // Initialize LSM6DS
    if (!lsm6ds.begin_I2C(0x6B)) {
        Serial.println("Could not find LSM6DS sensor");
        while (1);
    }
    Serial.println("LSM6DS found!");

    // Configure LSM6DS
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6ds.setAccelDataRate(LSM6DS_RATE_26_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_26_HZ);

    Serial.println("Sensors initialized!");
    Serial.println("Time(ms),Pressure(Pa),Temperature(C),AccelX(m/s^2),AccelY(m/s^2),AccelZ(m/s^2),GyroX(rad/s),GyroY(rad/s),GyroZ(rad/s)");
}

void printDiagnostics() {
    unsigned long currentTime = millis();
    if (currentTime - lastDiagnosticTime >= DIAGNOSTIC_INTERVAL) {
        Serial.print("Diagnostics - Baro read time: ");
        Serial.print(baroEndTime - baroStartTime);
        Serial.print("ms, LSM read time: ");
        Serial.print(lsmEndTime - lsmStartTime);
        Serial.print("ms, Total loop time: ");
        Serial.print(currentTime - lastSampleTime);
        Serial.println("ms");
        lastDiagnosticTime = currentTime;
    }
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if it's time for a new sample
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
        lastSampleTime = currentTime;

        // Read MPL3115A2
        baroStartTime = millis();        
        float pressure = baro.getPressure();
        float temperature = baro.getTemperature();
        baroEndTime = millis();
        
        // Read LSM6DS
        lsmStartTime = millis();
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        lsm6ds.getEvent(&accel, &gyro, &temp);
        lsmEndTime = millis();

        // Print data in CSV format
        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(pressure);
        Serial.print(",");
        Serial.print(temperature);
        Serial.print(",");
        Serial.print(accel.acceleration.x);
        Serial.print(",");
        Serial.print(accel.acceleration.y);
        Serial.print(",");
        Serial.print(accel.acceleration.z);
        Serial.print(",");
        Serial.print(gyro.gyro.x);
        Serial.print(",");
        Serial.print(gyro.gyro.y);
        Serial.print(",");
        Serial.println(gyro.gyro.z);

        // Print timing diagnostics
        // printDiagnostics();
    }
}
