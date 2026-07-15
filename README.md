# Rocket Tracker / Telemetry System

A rocket flight computer and ground station telemetry system built on ESP32 with LoRa communication.

> **📱 Get the iOS app — [Ronoth Tracker on the App Store](https://apps.apple.com/us/app/ronoth-tracker/id6787474660)**
> The companion mobile app connects to the ground station and displays live maps and telemetry data on your iPhone.

## Overview

This project implements a complete rocket telemetry solution with two main components:

- **Vehicle Firmware**: Collects GPS, temperature, pressure, and accelerometer data at 26Hz, transmits GPS location via LoRa, logs full sensor data to SD card, and displays system status on OLED
- **Ground Station**: Receives GPS telemetry over LoRa, displays location on OLED, and provides WiFi web interface for iPhone/mobile Maps integration

## Hardware Support

### Tracker

Currently supports Heltec LoRa 32 V3 or V4 (ESP32-S3)

- **Sensors**: GPS module, LSM6DS33 IMU, MPL3115A2 barometer
- **Communication**: SX1262 LoRa radio (915MHz)
- **Display**: SSD1306 OLED (128x64)
- **Storage**: SD card for data logging

### Ground Station

Any of the below boards will work.  No additional sensors, or GPS are required.  There are also a lot of 3D printed case options.

- Heltec WiFi LoRa 32 V3 or V4 (ESP32-S3)
- Wio Tracker L1 E-Ink

## Quick Start

### Web Flasher (no toolchain required)

Flash prebuilt release firmware straight from the browser at
**[flash.ronoth.com/tracker](https://flash.ronoth.com/tracker/)** — pick your
board, plug it in over USB, and install. Requires a Web Serial capable browser
(Chrome or Edge on desktop) and works for all ESP32-S3 boards. The Wio Tracker
L1 E-Ink is nRF52-based; download its `.uf2` from the
[latest release](https://github.com/ronoth/tracker/releases/latest) instead
(double-tap reset, then drag it onto the USB drive).

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