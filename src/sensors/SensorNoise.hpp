#ifndef SENSOR_NOISE_HPP
#define SENSOR_NOISE_HPP

#include "ISensor.hpp"
#include <Wire.h>

/**
 * @brief Noise Sensor I2C Master Client
 * 
 * This sensor communicates with an ESP32-C3 slave device running NoiseSensorI2CSlave.
 * The slave device reads a microphone connected to GPIO4 (ADC) and sends noise measurements via I2C.
 * 
 * Hardware Setup:
 *   - ESP32-C3 Slave: Runs NoiseSensorI2CSlave firmware with microphone on GPIO4
 *   - Master Board: Runs this SensorNoise class to read data via I2C
 * 
 * I2C Protocol:
 *   - Default Address: 0x08 (configurable on slave)
 *   - Command: CMD_GET_DATA (0x01) returns SensorData struct
 *   - Command: CMD_IDENTIFY (0x09) returns SensorIdentity struct
 */

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <NoiseSensorI2CSlave.h>
#else
#include "noise/NoiseProtocol.h"
#endif

class SensorNoise : public ISensor {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x08;
    static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
    static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;
    static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;

    SensorNoise(TwoWire* wire = &Wire, uint8_t address = DEFAULT_I2C_ADDRESS);
    virtual ~SensorNoise() {}

    // ISensor interface
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SNOISE; }
    const char* getName() const override { return "NoiseSensor"; }

    // Data getters
    float getNoise() const { return _noise; }
    float getNoiseAvg() const { return _noiseAvg; }
    float getNoisePeak() const { return _noisePeak; }
    float getNoiseMin() const { return _noiseMin; }
    float getNoiseAvgLegal() const { return _noiseAvgLegal; }

    // Get detected I2C address
    uint8_t getAddress() const { return _address; }

private:
    TwoWire* _wire;
    uint8_t _address;
    
    // Cached readings
    float _noise = 0.0f;
    float _noiseAvg = 0.0f;
    float _noisePeak = 0.0f;
    float _noiseMin = 0.0f;
    float _noiseAvgLegal = 0.0f;
    bool _available = false;

    // Helper methods
    bool autoDetect();
    bool checkDevicePresent(uint8_t addr);
    bool readIdentity(uint8_t addr, SensorIdentity& identity);
};

#endif
