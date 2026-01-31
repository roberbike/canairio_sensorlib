#include "SensorSCD30.hpp"

SensorSCD30::SensorSCD30(float* toffset_ptr, float* altoffset_ptr) 
    : _global_toffset(toffset_ptr), _global_altoffset(altoffset_ptr) {}

bool SensorSCD30::init() {
#ifndef Wire1
    if (!_scd30.begin()) return false;
#else
    if (!_scd30.begin() && !_scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID)) return false;
#endif
    
    // Sync offsets from global config
    if (_global_toffset) {
        setTemperatureOffset(*_global_toffset * 100); 
    }
    if (_global_altoffset) {
        setAltitudeOffset((uint16_t)*_global_altoffset);
    }
    
    _available = true;
    return true;
}

bool SensorSCD30::read() {
    if (!_scd30.dataReady() || !_scd30.read()) return false;
    
    uint16_t tCO2 = _scd30.CO2;
    if (tCO2 > 0) {
        _co2 = tCO2;
        _humi = _scd30.relative_humidity;
        _temp = _scd30.temperature;
        return true;
    }
    return false;
}

void SensorSCD30::setTemperatureOffset(float offset) {
    _scd30.setTemperatureOffset(uint16_t(offset * 100));
}

void SensorSCD30::setAltitudeOffset(uint16_t altitude) {
    _scd30.setAltitudeOffset(altitude);
}
