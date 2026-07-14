#pragma once

// Wio Tracker L1 E-ink ground station pin definitions
// Pin names reference Arduino D-pin numbers from variant.h

// LoRa SX1262 (on default SPI bus)
#define LORA_NSS   D4
#define LORA_DIO1  D1
#define LORA_RST   D2
#define LORA_BUSY  D3
#define LORA_RXEN  D5

// LoRa hardware config
#define LORA_TCXO_VOLTAGE   1.8f
#define LORA_USE_LDO        false

// E-ink display (on SPI1 bus, pins defined in variant.h as PIN_EINK_*)
#define EINK_WIDTH  250
#define EINK_HEIGHT 122

// GPS L76K UART (9600 baud)
#define GPS_RX D7
#define GPS_TX D6

// User interface
#define USER_BUTTON D13
#define USER_LED    PIN_LED1
#define BUZZER_PIN  D12
