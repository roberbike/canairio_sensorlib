#ifndef NOISE_PROTOCOL_H
#define NOISE_PROTOCOL_H

#include <stdint.h>

/**
 * @brief Common types and commands for Noise Sensor I2C protocol
 * This file is shared between the Master (SensorNoise) and Slave (NoiseSensorI2CSlave).
 */

// I2C Commands
enum I2CCommand {
  CMD_GET_DATA = 0x01,       // Request all sensor data
  CMD_GET_AVG = 0x02,        // Request average
  CMD_GET_PEAK = 0x03,       // Request peak
  CMD_GET_MIN = 0x04,        // Request minimum
  CMD_GET_LEGAL = 0x05,      // Request legal average
  CMD_GET_LEGAL_MAX = 0x06,  // Request maximum legal
  CMD_GET_STATUS = 0x07,     // Request status
  CMD_RESET = 0x08,          // Reset cycle
  CMD_IDENTIFY = 0x09,       // Identify sensor type and version
  CMD_PING = 0x09,           // Alias for identify
  CMD_GET_READY = 0x0A       // Check if data is ready
};

// Sensor data structure (32 bytes)
struct SensorData {
  float noise;             // Current noise level (mV)
  float noiseAvg;          // Average noise
  float noisePeak;         // Peak noise
  float noiseMin;          // Minimum noise
  float noiseAvgLegal;     // Legal average
  float noiseAvgLegalMax;  // Maximum legal average
  uint16_t lowNoiseLevel;  // Dynamic base noise level
  uint32_t cycles;         // Number of cycles completed
};

// Sensor identity structure (5 bytes)
struct SensorIdentity {
  uint8_t sensorType;    // 0x01 = Noise Sensor
  uint8_t versionMajor;  // Firmware version major
  uint8_t versionMinor;  // Firmware version minor
  uint8_t status;        // Bitmask: bit0=initialized, bit1=adc_active, bit2=data_ready
  uint8_t i2cAddress;    // Current I2C address
} __attribute__((packed));

#ifndef SENSOR_TYPE_NOISE
static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;
#endif

#endif
