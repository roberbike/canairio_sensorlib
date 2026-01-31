#include "SensorAM2320.hpp"

SensorAM2320::SensorAM2320(TwoWire* wire) : _wire(wire) {
    _driver = AM232X(wire);
}

bool SensorAM2320::init() {
    if (!_driver.begin()) return false;
    _driver.wakeUp();
    _available = true;
    return true;
}

bool SensorAM2320::read() {
    if (!_driver.isConnected()) return false;
    if (_driver.read() != AM232X_OK) return false;
    
    _temp = _driver.getTemperature();
    _humi = _driver.getHumidity();
    return true;
}
