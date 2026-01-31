#include "SensorSEN5x.hpp"
#include <Arduino.h>

SensorSEN5x::SensorSEN5x(TwoWire* wire) : _wire(wire) {
}

bool SensorSEN5x::init() {
    _sen5x.begin(*_wire);
    
    uint16_t error = _sen5x.deviceReset();
    if (error) return false;
    
    // Offset logic handled externally calling setTemperatureOffset if needed
    
    error = _sen5x.startMeasurement();
    _available = (error == 0);
    return _available;
}

bool SensorSEN5x::read() {
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;

    uint16_t error = _sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

    if (error) return false;

    _pm1p0 = massConcentrationPm1p0;
    _pm2p5 = massConcentrationPm2p5;
    _pm4p0 = massConcentrationPm4p0;
    _pm10p0 = massConcentrationPm10p0;
    _humi = ambientHumidity;
    _temp = ambientTemperature;
    _vocIndex = vocIndex;
    _noxIndex = noxIndex;

    return true;
}

void SensorSEN5x::setTemperatureOffset(float offset) {
     _sen5x.stopMeasurement();
     delay(100);
     _sen5x.setTemperatureOffsetSimple(offset);
     delay(100); 
     _sen5x.startMeasurement();
}
