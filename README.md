# Rocket Telemetry System

A rocket flight computer and ground station telemetry system built on ESP32 with LoRa communication.

## Overview

This project implements a complete rocket telemetry solution with two main components:

- **Vehicle Firmware**: Collects GPS, temperature, pressure, and accelerometer data at 26Hz, transmits GPS location via LoRa, logs full sensor data to SD card, and displays system status on OLED
- **Ground Station**: Receives GPS telemetry over LoRa, displays location on OLED, and provides WiFi web interface for iPhone/mobile Maps integration

## Hardware Requirements

- **Board**: Heltec WiFi LoRa 32 V3 (ESP32-S3)
- **Sensors**: GPS module, LSM6DS33 IMU, MPL3115A2 barometer
- **Communication**: SX1262 LoRa radio (915MHz)
- **Display**: SSD1306 OLED (128x64)
- **Storage**: SD card for data logging

## Quick Start

### Building and Uploading

```bash
# Vehicle firmware (transmits GPS, logs all sensors to SD)
pio run -e vehicle -t upload

# Ground station (receives GPS, displays on OLED)
pio run -e ground_station -t upload

# Monitor serial output
pio device monitor -b 115200
```

### Data Formats

**LoRa Transmission**: `GPS:lat,lon,alt,speed,sats,hdop`  
**SD Card Logging**: `timestamp,lat,lon,alt,speed,sats,hdop,pressure,temp,accelX,accelY,accelZ,gyroX,gyroY,gyroZ`

## Mobile Integration

### WiFi Web Interface

The ground station creates a WiFi hotspot for mobile device integration:

1. **Connect to WiFi**: Join "RocketTracker" network (password: rocket123)
2. **Open Browser**: Navigate to `192.168.4.1`
3. **View Live GPS**: Real-time rocket coordinates with auto-refresh
4. **Open in Maps**: Direct links to Apple Maps and Google Maps
5. **Track Rocket**: Navigate to rocket location using phone's Maps app

**Features**:
- Auto-refreshing GPS coordinates every 3 seconds
- Signal quality indicators (RSSI, SNR, data age)
- Mobile-optimized responsive design
- No app installation required

## Development

### Subsystem Testing

Individual component tests are available in `systems/` directory:

```bash
pio run -e gps -t upload          # GPS module test
pio run -e imu -t upload          # Accelerometer/gyroscope test  
pio run -e logger -t upload       # Multi-sensor logging test
pio run -e lora -t upload         # LoRa transmitter test
pio run -e radio -t upload        # LoRa receiver test
pio run -e oled -t upload         # OLED display test
pio run -e sd -t upload           # SD card performance test
pio run -e i2c -t upload          # I2C device scanner
```

### Pin Configuration

See `src/board.h` for complete pin definitions:
- **I2C Sensors**: SDA=6, SCL=5
- **I2C OLED**: SDA=17, SCL=18  
- **GPS**: RX=48, TX=47
- **LoRa**: NSS=8, SCK=9, MOSI=10, MISO=11
- **SD Card**: CS=3, MOSI=4, MISO=40, CLK=39

## System Architecture

### Vehicle Features
- **Dual I2C Buses**: Sensors on primary bus, OLED on secondary bus for isolation
- **Dual SPI Buses**: SD card on HSPI, LoRa on FSPI for no interference
- **26Hz Data Logging**: High-speed sensor sampling with CSV format
- **3-Second LoRa**: GPS transmission every 3 seconds for tracking
- **Status Display**: OLED shows GPS fix, satellite count, system health

### Ground Station Features  
- **LoRa Reception**: Receives and parses GPS telemetry packets
- **OLED Display**: Shows rocket location and signal quality
- **WiFi Hotspot**: Creates "RocketTracker" access point
- **Web Server**: Mobile-friendly interface with Maps integration
- **Live Updates**: Real-time coordinate display with auto-refresh

The modular design allows individual component testing before integration, with complete SPI/I2C bus isolation preventing hardware conflicts during operation.