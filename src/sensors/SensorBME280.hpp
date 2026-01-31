#ifndef SENSOR_BME280_HPP
#define SENSOR_BME280_HPP

#include "ISensor.hpp"
#include <Adafruit_BME280.h>

class SensorBME280 : public ISensor {
public:
    SensorBME280(uint8_t address = 0x76, TwoWire* wire = &Wire);
    virtual ~SensorBME280() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SBME280; }
    const char* getName() const override { return "BME280"; }

    // Specific methods
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }
    float getPressure() const { return _pres; }
    float getAltitude() const { return _alt; }

private:
   Adafruit_BME280 _bme280;
   TwoWire* _wire;
   uint8_t _address;
   
   float _temp = 0.0f;
   float _humi = 0.0f;
   float _pres = 0.0f;
   float _alt = 0.0f;
   bool _available = false;
};

#endif
