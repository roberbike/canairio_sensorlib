#include "SensorBME680.hpp"

SensorBME680::SensorBME680(uint8_t address, TwoWire* wire) : _bme680(wire), _address(address), _wire(wire) {
}

bool SensorBME680::init() {
    if (!_bme680.begin(_address)) return false;

    // Config from original Sensors.cpp
    _bme680.setTemperatureOversampling(BME680_OS_8X);
    _bme680.setHumidityOversampling(BME680_OS_2X);
    _bme680.setPressureOversampling(BME680_OS_4X);
    _bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme680.setGasHeater(320, 150);  // 320*C for 150 ms

    return true;
}

bool SensorBME680::read() {
    if (!_bme680.performReading()) return false;

    _temp = _bme680.temperature;
    _humi = _bme680.humidity;
    _pres = _bme680.pressure / 100.0;
    _gas = _bme680.gas_resistance / 1000.0;
    
    // Altitude calculation
    // Similar to others, using standard formula with 1013.25 as sealevel default
    // logic or just exposing pressure for main class to calculate if needed.
    // Original: alt = bme680.readAltitude(sealevel);
    _alt = _bme680.readAltitude(1013.25); 

    return true;
}
