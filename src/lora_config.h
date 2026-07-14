#pragma once

// LoRa link parameters — must match between vehicle (TX) and all ground stations (RX)
#define LORA_BANDWIDTH          125.0f
#define LORA_SPREADING_FACTOR   9
#define LORA_CODING_RATE        7
#define LORA_SYNC_WORD          RADIOLIB_SX126X_SYNC_WORD_PRIVATE
#define LORA_POWER              22
#define LORA_PREAMBLE_LEN       8
#define LORA_DEFAULT_FREQUENCY  914.0f

// SX1262 tunable range — bounds for user-configured frequencies (BLE config)
inline bool isValidLoRaFrequency(float mhz) {
    return mhz >= 150.0f && mhz <= 960.0f;
}
