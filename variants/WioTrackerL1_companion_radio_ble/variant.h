#ifndef _WIO_TRACKER_L1_VARIANT_H_
#define _WIO_TRACKER_L1_VARIANT_H_

#include "WVariant.h"

// Clock
#define VARIANT_MCK (64000000ul)
// #define USE_LFXO
#define USE_LFRC

// Pin counts
#define PINS_COUNT (37u)
#define NUM_DIGITAL_PINS (37u)
#define NUM_ANALOG_INPUTS (8u)
#define NUM_ANALOG_OUTPUTS (0u)

// LEDs
#define PIN_LED1 (11)
#define PIN_LED2 (12)
#define LED_GREEN   PIN_LED1
#define LED_BLUE    PIN_LED2
#define LED_BUILTIN PIN_LED1
#define LED_STATE_ON 1

// Button
#define PIN_BUTTON D13

// Digital pin mapping
#define D0  0   // P1.09 GNSS_WAKEUP
#define D1  1   // P0.07 LORA_DIO1
#define D2  2   // P1.07 LORA_RESET
#define D3  3   // P1.10 LORA_BUSY
#define D4  4   // P1.14 LORA_CS
#define D5  5   // P1.08 LORA_RXEN (RF switch)
#define D6  6   // P0.27 GNSS_TX
#define D7  7   // P0.26 GNSS_RX
#define D8  8   // P0.30 SPI_SCK
#define D9  9   // P0.03 SPI_MISO
#define D10 10  // P0.28 SPI_MOSI
#define D11 11  // P1.01 User LED
#define D12 12  // P1.00 Buzzer
#define D13 13  // P0.08 User Button
#define D14 14  // P0.06 I2C SDA
#define D15 15  // P0.05 I2C SCL
#define D16 16  // P0.31 VBAT_ADC
#define D17 17  // P1.11 Grove SDA
#define D18 18  // P1.12 Grove SCL

// Analog pins
#define PIN_A0 0
#define PIN_A1 1
#define PIN_A2 2
#define PIN_A3 3
#define PIN_A4 4
#define PIN_A5 5
#define PIN_VBAT D16

// I2C
#define PIN_WIRE_SDA D14
#define PIN_WIRE_SCL D15
#define WIRE_INTERFACES_COUNT 1

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// SPI (LoRa - SPIM3)
// Global SPI pin constants required by libraries
#define PIN_SPI_MISO D9
#define PIN_SPI_MOSI D10
#define PIN_SPI_SCK  D8

static const uint8_t SS   = D4;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1 (E-ink - SPIM2)
#define SPI_INTERFACES_COUNT 2
#define PIN_SPI1_MISO (-1)
#define PIN_SPI1_MOSI 33
#define PIN_SPI1_SCK  31

// E-ink display
#define PIN_EINK_CS   36
#define PIN_EINK_BUSY 35
#define PIN_EINK_DC   34
#define PIN_EINK_RES  32
#define PIN_EINK_SCLK 31
#define PIN_EINK_MOSI 33

// GPS L76K UART
#define PIN_SERIAL1_TX D6
#define PIN_SERIAL1_RX D7

// Battery
#define BAT_READ 30
#define BATTERY_PIN PIN_VBAT
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define ADC_MULTIPLIER 2.0
#define AREF_VOLTAGE 3.6

// QSPI Flash (physical nRF52840 GPIO numbers, not Arduino pin numbers)
#define PIN_QSPI_SCK  (21)  // P0.21
#define PIN_QSPI_CS   (25)  // P0.25
#define PIN_QSPI_IO0  (20)  // P0.20
#define PIN_QSPI_IO1  (24)  // P0.24
#define PIN_QSPI_IO2  (22)  // P0.22
#define PIN_QSPI_IO3  (23)  // P0.23
#define EXTERNAL_FLASH_DEVICES P25Q16H
#define EXTERNAL_FLASH_USE_QSPI

// Buzzer
#define PIN_BUZZER D12

// Trackball (5-way joystick)
#define TB_UP    25
#define TB_DOWN  26
#define TB_LEFT  27
#define TB_RIGHT 28
#define TB_PRESS 29

// Unused serial2
#ifdef __cplusplus
extern "C" {
#endif
#define PIN_SERIAL2_RX (-1)
#define PIN_SERIAL2_TX (-1)
#ifdef __cplusplus
}
#endif

#endif
