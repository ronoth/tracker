#include "variant.h"
#include "nrf.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

// g_ADigitalPinMap maps Arduino pin numbers to nRF52840 GPIO numbers.
// nRF52840: P0.xx = xx, P1.xx = 32 + xx
const uint32_t g_ADigitalPinMap[] = {
    // D0-D10: LoRa and GPS control
    41,  // D0  P1.09  GNSS_WAKEUP
    7,   // D1  P0.07  LORA_DIO1
    39,  // D2  P1.07  LORA_RESET
    42,  // D3  P1.10  LORA_BUSY
    46,  // D4  P1.14  LORA_CS
    40,  // D5  P1.08  LORA_RXEN (RF switch)
    27,  // D6  P0.27  GNSS_TX
    26,  // D7  P0.26  GNSS_RX
    30,  // D8  P0.30  SPI_SCK
    3,   // D9  P0.03  SPI_MISO
    28,  // D10 P0.28  SPI_MOSI

    // D11-D12: LED and buzzer
    33,  // D11 P1.01  User LED
    32,  // D12 P1.00  Buzzer

    // D13: User button
    8,   // D13 P0.08  User Button

    // D14-D15: I2C
    6,   // D14 P0.06  I2C SDA
    5,   // D15 P0.05  I2C SCL

    // D16: Battery ADC
    31,  // D16 P0.31  VBAT_ADC

    // D17-D18: Grove I2C
    43,  // D17 P1.11  Grove SDA
    44,  // D18 P1.12  Grove SCL

    // D19-D24: QSPI Flash
    21,  // D19 P0.21  QSPI_SCK
    25,  // D20 P0.25  QSPI_CSN
    20,  // D21 P0.20  QSPI_IO0
    24,  // D22 P0.24  QSPI_IO1
    22,  // D23 P0.22  QSPI_IO2
    23,  // D24 P0.23  QSPI_IO3

    // D25-D29: Trackball (5-way joystick)
    36,  // D25 P1.04  TB_UP
    12,  // D26 P0.12  TB_DOWN
    11,  // D27 P0.11  TB_LEFT
    35,  // D28 P1.03  TB_RIGHT
    37,  // D29 P1.05  TB_PRESS

    // D30: Battery control
    4,   // D30 P0.04  BAT_CTL

    // D31-D36: E-ink display
    13,  // D31 P0.13  EINK_SCK
    14,  // D32 P0.14  EINK_RST
    15,  // D33 P0.15  EINK_MOSI
    16,  // D34 P0.16  EINK_DC
    17,  // D35 P0.17  EINK_BUSY
    19,  // D36 P0.19  EINK_CS
};

void initVariant()
{
    // QSPI chip select high (deselect flash)
    // Use nrf_gpio API since PIN_QSPI_CS is a raw GPIO number, not Arduino pin
    nrf_gpio_cfg_output(PIN_QSPI_CS);
    nrf_gpio_pin_set(PIN_QSPI_CS);

    // Enable battery voltage divider
    pinMode(BAT_READ, OUTPUT);
    digitalWrite(BAT_READ, HIGH);

    // LEDs off
    pinMode(PIN_LED1, OUTPUT);
    digitalWrite(PIN_LED1, LOW);
    pinMode(PIN_LED2, OUTPUT);
    digitalWrite(PIN_LED2, LOW);

    // Buzzer off
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
}
