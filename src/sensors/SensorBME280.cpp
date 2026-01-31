#include "SensorBME280.hpp"

SensorBME280::SensorBME280(uint8_t address, TwoWire* wire) : _bme280(), _wire(wire), _address(address) {
}

bool SensorBME280::init() {
    // Adafruit BME280 begin takes (address, wire)
    _available = _bme280.begin(_address, _wire);
    return _available;
}

bool SensorBME280::read() {
    float t = _bme280.readTemperature();
    float h = _bme280.readHumidity();
    float p = _bme280.readPressure(); // in Pa
    
    if (!isnan(t) && !isnan(h) && !isnan(p)) {
        _temp = t;
        _humi = h;
        _pres = p / 100.0F; // Convert to hPa
        // Approx altitude calculation if needed, or we can just expose pressure
        // _alt = _bme280.readAltitude(SEALEVELPRESSURE_HPA); // We need sea level pressure
        return true;
    }
    return false;
}
