
// ADC pins
#define VEXT 36
#define ADC_CTRL 37
#define VBAT_SENSE 1
#define ADC_SCALER 4.899

// User button (PRG)
#define USER_BUTTON 0

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C // I2C address (0x3C for 128x64)

#ifdef BOARD_V4

// GPS pins (L76K via SH1.25-8Pin connector)
#define GPS_RX 39
#define GPS_TX 38
#define GPS_EN 34      // VGNSS_ctrl, active LOW
#define GPS_RESET 42   // L76K reset, active LOW
#define GPS_PPS 41
#define GPS_STANDBY 40

// LED pins
#define LED 35

// I2C pins
#define I2C_SCL 5
#define I2C_SDA 6

// OLED I2C pins
#define OLED_I2C_SCL 18
#define OLED_I2C_SDA 17
#define OLED_RST 21

// No SD card on V4. Do NOT reuse GPIO 2/46 for SD — they control the RF front-end.

// GC1109 RF front-end (FEM) control — all three must be driven for usable TX/RX.
// TX/RX path switching (CTX) is wired to SX1262 DIO2 (setDio2AsRfSwitch).
#define FEM_POWER 7    // VFEM_Ctrl: LDO powering the FEM, HIGH = on
#define FEM_CSD 2      // Chip enable, HIGH = on
#define FEM_CPS 46     // HIGH = full PA on TX, LOW = PA bypass

// VBAT divider enable polarity (ADC_CTRL) — active HIGH on V4
#define ADC_CTRL_ON HIGH

// LoRa pins
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

// LoRa hardware config
#define LORA_TCXO_VOLTAGE   1.8f
#define LORA_USE_LDO        true

#else // BOARD_V3 (default)

// GPS pins
#define GPS_RX 48
#define GPS_TX 47

// LED pins
#define LED 35

// I2C pins
#define I2C_SCL 5
#define I2C_SDA 6

// OLED I2C pins
#define OLED_I2C_SCL 18
#define OLED_I2C_SDA 17
#define OLED_RST 21

// VBAT divider enable polarity (ADC_CTRL) — active LOW on V3
#define ADC_CTRL_ON LOW

// SD Card pins
#define SD_CS 3
#define SD_MOSI 4
#define SD_MISO 40
#define SD_CLK  39

// LoRa pins
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

// LoRa hardware config
#define LORA_TCXO_VOLTAGE   1.6f
#define LORA_USE_LDO        true

// Radio pins
#define RADIO_NSS 18

#endif