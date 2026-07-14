#pragma once
#include <stdint.h>

struct SensorData {
    uint32_t timestamp;
    float pressure;
    float temperature;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};
