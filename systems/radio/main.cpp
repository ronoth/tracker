#include <Arduino.h>
#include "board.h"
#include <RadioLib.h>

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
// and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}

void parseGPSData(const char* data) {
    // Check if it's a GPS packet
    if (strncmp(data, "GPS:", 4) != 0) {
        Serial.println("Not a GPS packet");
        return;
    }

    // Move pointer past "GPS:"
    const char* gpsData = data + 4;

    // Check if we have a GPS fix
    if (strncmp(gpsData, "NoFix", 5) == 0) {
        // Parse NoFix data
        int satellites;
        float hdop;
        if (sscanf(gpsData + 6, "%d,%f", &satellites, &hdop) == 2) {
            Serial.println("\n=== GPS Status ===");
            Serial.println("Fix: No GPS Fix");
            Serial.print("Satellites: ");
            Serial.println(satellites);
            Serial.print("HDOP: ");
            Serial.println(hdop);
        }
    } else {
        // Parse full GPS data
        float lat, lon, alt, speed;
        int satellites;
        float hdop;
        
        if (sscanf(gpsData, "%f,%f,%f,%f,%d,%f", 
                   &lat, &lon, &alt, &speed, &satellites, &hdop) == 6) {
            Serial.println("\n=== GPS Data ===");
            Serial.print("Latitude: ");
            Serial.println(lat, 6);
            Serial.print("Longitude: ");
            Serial.println(lon, 6);
            Serial.print("Altitude: ");
            Serial.print(alt);
            Serial.println(" meters");
            Serial.print("Speed: ");
            Serial.print(speed);
            Serial.println(" km/h");
            Serial.print("Satellites: ");
            Serial.println(satellites);
            Serial.print("HDOP: ");
            Serial.println(hdop);
        }
    }
}

void setup() {
  Serial.begin(115200);

  // Initialize SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  // Initialize LoRa radio
  Serial.println(F("Initializing LoRa radio"));
  int16_t res = radio.begin(
    915.0 /* freq */,
    125.0 /* bw */,
    9 /* sf */,
    7 /* cr*/,
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE, /* sync word */
    22 /* Power */,
    8 /* preambleLength */,
    1.6 /* tcxoVoltage*/,
    true /* useRegulatorLDO */);

  if(res != RADIOLIB_ERR_NONE) {
    Serial.print(F("Failed to initialize LoRa radio. Code:"));
    Serial.println(res);
    while(1);
  }

  // set the function that will be called
  // when packet is received
  radio.setDio1Action(setFlag);

  // start listening for packets
  Serial.print(F("Starting to listen... "));
  res = radio.startReceive();
  if(res != RADIOLIB_ERR_NONE) {
    Serial.print(F("Failed to start receive. Code:"));
    Serial.println(res);
    while(1);
  }
  Serial.println(F("success!"));
}

void loop() {
  // check if the flag is set
  if(receivedFlag) {
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    String str;
    int state = radio.readData(str);

    if(state == RADIOLIB_ERR_NONE) {
      // packet was successfully received
      Serial.println(F("\nReceived packet!"));

      // print the data of the packet
      Serial.print(F("Raw Data:\t"));
      Serial.println(str);

      // Parse and display GPS data
      parseGPSData(str.c_str());

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
    }

    // put module back to listen mode
    radio.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
  }
}
