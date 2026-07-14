#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>
#include "board.h"

MS5611 ms5611;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("MS5611 Barometric Pressure Sensor Test");
    Serial.println("======================================");

    // Enable power to sensors
    pinMode(VEXT, OUTPUT);
    digitalWrite(VEXT, LOW);
    delay(100);

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz
    delay(100);

    // Initialize MS5611
    if (ms5611.begin()) {
        Serial.println("MS5611 found and initialized");
    } else {
        Serial.println("MS5611 not found!");
        while (1) delay(1000);
    }

    Serial.println("\nData Format:");
    Serial.println("Time(ms),Pressure(Pa),Temperature(C),Altitude(m)");
    Serial.println("===============================================");
    
    delay(1000);
}

void loop() {
    static unsigned long lastReading = 0;
    static unsigned long lastBenchmark = 0;
    static float referencePressure = 0;
    static bool referenceSet = false;
    static int sampleCount = 0;
    
    unsigned long currentTime = millis();
    
    // Read sensor at 50Hz (20ms interval)
    if (currentTime - lastReading >= 20) {
        lastReading = currentTime;
        sampleCount++;
        
        // Read the sensor
        int result = ms5611.read();
        
        if (result == MS5611_READ_OK) {
            float pressure = ms5611.getPressure();
            float temperature = ms5611.getTemperature();
            
            // Set reference pressure from first reading
            if (!referenceSet) {
                referencePressure = pressure;
                referenceSet = true;
                Serial.print("Reference pressure set to: ");
                Serial.print(referencePressure);
                Serial.println(" Pa");
            }
                       
            // Print CSV data
            Serial.print(currentTime);
            Serial.print(",");
            Serial.print(pressure, 2);
            Serial.print(",");
            Serial.print(temperature, 2);
            Serial.println();
        } else {
            Serial.print("MS5611 read failed with error: ");
            Serial.println(result);
        }
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