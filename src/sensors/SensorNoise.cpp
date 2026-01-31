#include "SensorNoise.hpp"
#include <Arduino.h>

SensorNoise::SensorNoise(TwoWire* wire, uint8_t address) 
    : _wire(wire), _address(address) {
}

bool SensorNoise::init() {
    // If address is 0 or default, try auto-detection
    if (_address == 0 || _address == DEFAULT_I2C_ADDRESS) {
        return autoDetect();
    }
    
    // Otherwise verify the provided address
    if (!checkDevicePresent(_address)) {
        return false;
    }
    
    SensorIdentity identity{};
    if (!readIdentity(_address, identity)) {
        return false;
    }
    
    _available = (identity.sensorType == SENSOR_TYPE_NOISE);
    return _available;
}

bool SensorNoise::read() {
    if (_address == 0) return false;
    
    // Send CMD_GET_DATA command
    _wire->beginTransmission(_address);
    _wire->write(CMD_GET_DATA);
    if (_wire->endTransmission() != 0) {
        return false;
    }
    
    // Request sensor data
    size_t bytesReceived = _wire->requestFrom(_address, (uint8_t)sizeof(SensorData));
    if (bytesReceived != sizeof(SensorData)) {
        return false;
    }
    
    // Read data structure
    SensorData data;
    uint8_t* ptr = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(SensorData); i++) {
        ptr[i] = _wire->read();
    }
    
    // Update cached values
    _noise = data.noise;
    _noiseAvg = data.noiseAvg;
    _noisePeak = data.noisePeak;
    _noiseMin = data.noiseMin;
    _noiseAvgLegal = data.noiseAvgLegal;
    
    return true;
}

bool SensorNoise::autoDetect() {
    // Scan I2C address range for noise sensor
    for (uint8_t addr = MIN_I2C_ADDRESS; addr <= MAX_I2C_ADDRESS; addr++) {
        if (!checkDevicePresent(addr)) {
            continue;
        }
        
        SensorIdentity identity{};
        if (!readIdentity(addr, identity)) {
            continue;
        }
        
        if (identity.sensorType == SENSOR_TYPE_NOISE) {
            _address = addr;
            return true;
        }
    }
    
    return false;
}

bool SensorNoise::checkDevicePresent(uint8_t addr) {
    _wire->beginTransmission(addr);
    return (_wire->endTransmission() == 0);
}

bool SensorNoise::readIdentity(uint8_t addr, SensorIdentity& identity) {
    // Send CMD_IDENTIFY command
    _wire->beginTransmission(addr);
    _wire->write(CMD_IDENTIFY);
    if (_wire->endTransmission() != 0) {
        return false;
    }

    // Request identity data
    size_t bytesReceived = _wire->requestFrom(addr, (uint8_t)sizeof(SensorIdentity));
    if (bytesReceived != sizeof(SensorIdentity)) {
        return false;
    }

    // Read identity structure
    uint8_t* ptr = (uint8_t*)&identity;
    for (size_t i = 0; i < sizeof(SensorIdentity); i++) {
        ptr[i] = _wire->read();
    }
    
    return true;
}
