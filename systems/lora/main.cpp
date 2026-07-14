#include <Arduino.h>
#include <RadioLib.h>

// LoRa pins
#define LORA_CS 18
#define LORA_DIO0 26
#define LORA_DIO1 33
#define LORA_RST 23
#define LORA_BUSY 32

// LoRa radio instance
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_BUSY);

// Message to send
const char* message = "Hello";

void setup() {
  Serial.begin(115200);
  Serial.println(F("LoRa Test Starting..."));

  // Initialize LoRa
  Serial.print(F("Initializing LoRa..."));
  
  // Carrier frequency:           915.0 MHz
  // Bandwidth:                   125.0 kHz
  // Spreading factor:            7
  // Coding rate:                 5
  // Sync word:                   0x12
  // Output power:                17 dBm
  // Current limit:               100 mA
  // Preamble length:             8 symbols
  // Amplifier gain:              0 (automatic gain control)
  int state = radio.begin(915.0, 125.0, 7, 5, 0x12, 17, 100, 8, 0);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  Serial.print(F("Sending packet... "));
  
  // Send message
  int state = radio.transmit(message);
  
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));
    
    // print measured data rate
    Serial.print(F("Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
    
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
    
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occurred while transmitting packet
    Serial.println(F("timeout!"));
    
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  
  // Wait for 5 seconds before sending the next message
  delay(5000);
} 