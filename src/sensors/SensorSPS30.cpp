#include "SensorSPS30.hpp"

SensorSPS30::SensorSPS30() {}

SensorSPS30::SensorSPS30(Stream& serial) : _serial(&serial) {}

bool SensorSPS30::init() {
    if (_serial) {
        if (!_driver.begin(*_serial)) return false;
    } else {
        if (_driver.begin(&Wire) == false) return false;
    }

    if (!_driver.probe()) return false;
    if (!_driver.reset()) return false;
    
    if (!_driver.start()) return false;
    
    _available = true;
    return true;
}

bool SensorSPS30::read() {
    struct sps_values val;
    if (_driver.GetValues(&val) != SPS30_ERR_OK) return false;
    
    _pm1 = (uint16_t)round(val.MassPM1);
    _pm25 = (uint16_t)round(val.MassPM2);
    _pm4 = (uint16_t)round(val.MassPM4);
    _pm10 = (uint16_t)round(val.MassPM10);
    
    return true;
}
