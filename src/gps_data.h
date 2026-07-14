#pragma once
#include <stdint.h>
#include "flight_state.h"

struct GPSData {
    // Core position — set on both vehicle and ground
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int satellites;
    float hdop;
    float batteryVoltage;     // vehicle battery volts, 0 = unknown/not received

    bool hasGPSFix;           // current GPS position is valid
    bool altitudeValid;       // altitude reading is valid (vehicle only)
    bool speedValid;          // speed reading is valid (vehicle only)
    uint32_t lastGPSFixTime;  // millis() of last valid fix, 0 if never had one

    // Ground station fields — populated from received LoRa packets
    bool dataValid;           // at least one packet has been received
    uint32_t lastUpdate;      // millis() of last received packet
    float rssi;
    float snr;
    FlightState vehicleState;
    char trackerName[24];     // tracker name from packet, "" if not sent
    char trackerId[8];        // hardware ID (6 hex digits of MAC), "" if not sent
};
