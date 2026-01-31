#include "SensorSHT31.hpp"

SensorSHT31::SensorSHT31(uint8_t address, TwoWire* wire) : _sht31(wire), _address(address) {
}

bool SensorSHT31::init() {
    // SHT31 address is usually 0x44 or 0x45
    _available = _sht31.begin(_address);
    return _available;
}

bool SensorSHT31::read() {
    float t = _sht31.readTemperature();
    float h = _sht31.readHumidity();

    if (!isnan(t) && !isnan(h)) {
        _temp = t;
        _humi = h;
        return true;
    }
    return false;
}
