#include "SensorBMP280.hpp"

SensorBMP280::SensorBMP280(uint8_t address, TwoWire* wire) : _bmp280(wire), _address(address), _wire(wire) {
}

bool SensorBMP280::init() {
    if (!_bmp280.begin(_address)) return false;

    // Default configuration from original Sensors.cpp
    _bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Operating Mode.
                        Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                        Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                        Adafruit_BMP280::FILTER_X16,       // Filtering.
                        Adafruit_BMP280::STANDBY_MS_500);  // Standby time.
    return true;
}

bool SensorBMP280::read() {
    float t = _bmp280.readTemperature();
    float p = _bmp280.readPressure();
    
    // Check for valid readings? Adafruit lib returns 0 on error? or NAN?
    // Original code checked for press1 == 0 || isnan(temp1)
    if (p == 0 || isnan(t)) return false;

    _temp = t;
    _pres = p / 100.0F; // Convert to hPa
    
    // Altitude calculation not provided by BMP280 directly with stored sea level?
    // Adafruit BMP280 has readAltitude(seaLevelhPa)
    // We can expose a method to set sea level or just calculate it like BME280.
    // However, existing code used: alt1 = bmp280.readAltitude(sealevel);
    // where sealevel is a member of Sensors class.
    // For now we will return 0 or calculate standard.
    // The ISensor interface doesn't pass sealevel.
    // We will calculate standard altitude based on 1013.25 for now or add setter.
    _alt = 44330.0 * (1.0 - pow(_pres / 1013.25, 0.1903)); 
    
    return true;
}
