# Noise Sensor Library

This directory contains the noise sensor implementation for the CanAirIO Sensor Library.

## Hardware Architecture

The noise sensor system uses a **master-slave I2C architecture**:

```
┌─────────────────────────────────────┐
│       Master Board (Main MCU)       │
│                                     │
│  ┌───────────────────────────────┐  │
│  │   SensorNoise (ISensor)       │  │  ← I2C Master Client
│  │   Reads data via I2C          │  │     (src/sensors/SensorNoise.hpp)
│  └───────────┬───────────────────┘  │
│              │ I2C Bus              │
└──────────────┼──────────────────────┘
               │
               │ I2C Communication
               │
┌──────────────┼──────────────────────┐
│              │                      │
│  ┌───────────▼───────────────────┐  │
│  │  NoiseSensorI2CSlave          │  │  ← I2C Slave (Integrated)
│  │  (ESP32-C3 Firmware)          │  │     (ADC + I2C logic)
│  └───────────────────────────────┘  │
│                                     │
│    ESP32-C3 Slave Device            │
│    (Dedicated Noise Sensor)         │
└─────────────────────────────────────┘
```

## Files

### Master Side (Main Library)
- **`../SensorNoise.hpp/cpp`** - I2C Master Client
  - Implements `ISensor` interface
  - Communicates with ESP32-C3 slave via I2C
  - Auto-detects slave address (default: 0x08)
  - Reads noise measurements from slave

### Slave Side (ESP32-C3 Firmware)
- **`NoiseSensorI2CSlave.h/cpp`** - Integrated I2C Slave & Measurement
  - Runs on ESP32-C3 with microphone
  - Implements both I2C slave protocol AND ADC measurement logic
  - Reads microphone from GPIO4 (ADC)
  - Calculates noise levels (SPL, peak, min, avg) and legal averages

## Usage

### Master Board (Main Application)

Use `SensorNoise` through the main `Sensors` class:

```cpp
#include <Sensors.hpp>

Sensors sensors;
sensors.init();  // Auto-detects noise sensor on I2C bus

void loop() {
    sensors.readAllSensors();
    
    float noise = sensors.getNoise();
}
```

### ESP32-C3 Slave Device

Flash the ESP32-C3 with firmware using `NoiseSensorI2CSlave`:

```cpp
#include "noise/NoiseSensorI2CSlave.h"

NoiseSensorI2CSlave::Config config;
config.i2cAddress = 0x08;  // I2C slave address
config.adcPin = 4;         // Microphone on GPIO4

NoiseSensorI2CSlave noiseSlave(config);

void setup() {
    noiseSlave.begin();
}

void loop() {
    noiseSlave.update();  // Continuously read ADC and update data
}
```

## I2C Protocol

### Commands (Master → Slave)

| Command | Value | Description | Response |
|---------|-------|-------------|----------|
| `CMD_GET_DATA` | 0x01 | Get all sensor data | `SensorData` struct (32 bytes) |
| `CMD_IDENTIFY` | 0x09 | Identify sensor | `SensorIdentity` struct (5 bytes) |

## Hardware Setup

### ESP32-C3 Slave Connections
- **GPIO4**: Microphone ADC input
- **GPIO8**: I2C SDA
- **GPIO10**: I2C SCL
- **GND**: Common ground with master
- **3.3V**: Power supply

## Version
- NoiseSensorI2CSlave: v1.2 (Integrated)
- Sensor Type ID: 0x01
