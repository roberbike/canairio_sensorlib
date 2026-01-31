#include "SensorMHZ19.hpp"

SensorMHZ19::SensorMHZ19(Stream& serial) : _serial(serial) {}

bool SensorMHZ19::init() {
    _mhz19.begin(_serial);
    _mhz19.autoCalibration(false);
    delay(100);
    // Try a quick read to verify presence
    int co2 = _mhz19.getCO2();
    _available = (co2 > 0);
    return _available;
}

bool SensorMHZ19::read() {
    int co2 = _mhz19.getCO2();
    if (co2 > 0) {
        _co2 = (uint16_t)co2;
        _temp = _mhz19.getTemperature();
        return true;
    }
    return false;
}
