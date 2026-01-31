#ifndef SENSOR_BMP280_HPP
#define SENSOR_BMP280_HPP

#include "ISensor.hpp"
#include <Adafruit_BMP280.h>

class SensorBMP280 : public ISensor {
public:
    SensorBMP280(uint8_t address = 0x77, TwoWire* wire = &Wire);
    virtual ~SensorBMP280() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SBMP280; }
    const char* getName() const override { return "BMP280"; }

    // Specific methods
    float getTemperature() const { return _temp; }
    float getPressure() const { return _pres; }
    float getAltitude() const { return _alt; }

private:
   Adafruit_BMP280 _bmp280;
   uint8_t _address;
   TwoWire* _wire;
   
   float _temp = 0.0f;
   float _pres = 0.0f;
   float _alt = 0.0f;
   bool _available = false;
};

#endif
