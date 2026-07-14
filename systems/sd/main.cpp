/*
 * SD Card Write Throughput Test
 * 
 * This sketch tests the write throughput of an SD card using the SdFat library.
 * It creates a test file and writes data in blocks to measure the write speed.
 * 
 * Connections:
 * SD card attached to SPI bus as follows:
 * - MOSI - pin 11
 * - MISO - pin 12
 * - CLK - pin 13
 * - CS - pin 10 (default, can be changed below)
 * 
 * Requirements:
 * - SdFat library (install via Library Manager)
 */

#include <SdFat.h>
#include "board.h"

// Configuration
const uint32_t BLOCK_SIZE = 512;  // Write block size in bytes
const uint32_t NUM_BLOCKS = 1024;  // Number of blocks to write (512KB total)
const uint32_t FILE_SIZE = BLOCK_SIZE * NUM_BLOCKS;
const char* TEST_FILENAME = "SPEEDTST.BIN";

// Create the SdFat instance
SdFat sd;
FatFile testFile;

// Buffer for test data
uint8_t buffer[BLOCK_SIZE];

// Results
uint32_t writeTime = 0;
float writeMB = 0;
float writeMBps = 0;

void setup() {
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  Serial.begin(115200);
  
  // Wait for Serial monitor to open
  while (!Serial) {}

  Serial.println(F("SD Card Write Throughput Test"));
  Serial.println(F("------------------------------"));

  // Initialize the SD card
  Serial.print(F("Initializing SD card..."));
  if (!sd.begin(SD_CS, SPI_FULL_SPEED)) {
    Serial.println(F("initialization failed!"));
    if (sd.card()->errorCode()) {
      Serial.print(F("SD error: "));
      Serial.println(sd.card()->errorCode());
    }
    return;
  }
  Serial.println(F("done."));

  // Fill buffer with test data
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    buffer[i] = i & 0xFF;
  }

  // Remove existing test file
  sd.remove(TEST_FILENAME);

  Serial.println(F("Starting write test..."));
  Serial.print(F("File size: "));
  Serial.print(FILE_SIZE / 1024);
  Serial.println(F(" KB"));

  // Create a new file
  if (!testFile.open(TEST_FILENAME, O_WRITE | O_CREAT)) {
    Serial.println(F("Failed to open test file!"));
    return;
  }

  // Start timing
  uint32_t startTime = millis();

  // Write data to the file
  for (uint32_t i = 0; i < NUM_BLOCKS; i++) {
    if (testFile.write(buffer, BLOCK_SIZE) != BLOCK_SIZE) {
      Serial.println(F("Write failed!"));
      break;
    }

    // Print progress every 64 blocks (32KB)
    if (i % 64 == 0) {
      Serial.print('.');
    }
  }
  
  // Ensure all data is written to the SD card
  testFile.sync();
  
  // End timing
  writeTime = millis() - startTime;

  // Close the file
  testFile.close();

  // Calculate performance metrics
  writeMB = (float)FILE_SIZE / (1024 * 1024);
  writeMBps = 1000.0 * writeMB / writeTime;

  // Print results
  Serial.println();
  Serial.println(F("Test complete!"));
  Serial.print(F("Data written: "));
  Serial.print(writeMB);
  Serial.println(F(" MB"));
  
  Serial.print(F("Write time: "));
  Serial.print(writeTime / 1000.0);
  Serial.println(F(" seconds"));

  Serial.print(F("Write speed: "));
  Serial.print(writeMBps);
  Serial.println(F(" MB/s"));

  // Additional stats
  Serial.print(F("Write speed: "));
  Serial.print(writeMBps * 8);
  Serial.println(F(" Mbps"));

  Serial.print(F("Write speed: "));
  Serial.print(writeMB / (writeTime / 1000.0) * 60);
  Serial.println(F(" MB/min"));

  Serial.println(F("Test file has been left on the SD card for verification."));
  Serial.println(F("You can delete it manually if needed."));
}

void loop() {
  // Nothing to do here
}