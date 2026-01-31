#include "SensorSCD4x.hpp"
#include <Arduino.h>

SensorSCD4x::SensorSCD4x(TwoWire* wire) : _wire(wire) {
}

bool SensorSCD4x::init() {
    _scd4x.begin(*_wire);
    
    // Stop potentially running periodic measurement to allow config
    uint16_t error = _scd4x.stopPeriodicMeasurement();
    if (error) return false;
    
    // Config logic usually happens here or is called externally.
    // The original code calculates offsets based on global params. 
    // We'll leave that to the caller (Sensors.cpp) via the setters provided.

    error = _scd4x.startPeriodicMeasurement();
    _available = (error == 0);
    return _available;
}

bool SensorSCD4x::read() {
    uint16_t tCO2 = 0;
    float tTemp = 0.0f;
    float tHumi = 0.0f;
    
    // bool getDataReadyFlag(bool& dataReady) exists but original code used 
    // readMeasurement directly which blocks? or checks if ready?
    // Sensirion library readMeasurement blocks if I recall correctly or returns error if not ready.
    // However, original code simply calls readMeasurement.
    
    uint16_t error = _scd4x.readMeasurement(tCO2, tTemp, tHumi);
    if (error) return false;
    
    if (tCO2 == 0) return false; // Basic filter
    
    _co2 = tCO2;
    _temp = tTemp;
    _humi = tHumi;
    
    return true;
}

void SensorSCD4x::setTemperatureOffset(float offset) {
    _scd4x.stopPeriodicMeasurement();
    delay(500); // Spec requires 500ms wait
    _scd4x.setTemperatureOffset(offset);
    _scd4x.startPeriodicMeasurement();
}

float SensorSCD4x::getTemperatureOffset() {
    float offset = 0.0;
    _scd4x.stopPeriodicMeasurement();
    uint16_t error = _scd4x.getTemperatureOffset(offset);
    _scd4x.startPeriodicMeasurement();
    if (error) return 0.0;
    return offset;
}

void SensorSCD4x::setSensorAltitude(uint16_t altitude) {
    _scd4x.stopPeriodicMeasurement();
    delay(500);
    _scd4x.setSensorAltitude(altitude);
    _scd4x.startPeriodicMeasurement();
}

uint16_t SensorSCD4x::performForcedRecalibration(uint16_t targetCO2, uint16_t &frcCorrection) {
    _scd4x.stopPeriodicMeasurement();
    delay(500);
    uint16_t error = _scd4x.performForcedRecalibration(targetCO2, frcCorrection);
    delay(50);
    _scd4x.startPeriodicMeasurement();
    return error;
}
