#ifndef SENSOR_SHT31_HPP
#define SENSOR_SHT31_HPP

#include "ISensor.hpp"
#include <Adafruit_SHT31.h>

class SensorSHT31 : public ISensor {
public:
    SensorSHT31(uint8_t address = 0x44, TwoWire* wire = &Wire);
    virtual ~SensorSHT31() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSHT31; }
    const char* getName() const override { return "SHT31"; }

    // Specific methods
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }

private:
    Adafruit_SHT31 _sht31;
    float _temp = 0.0f;
    float _humi = 0.0f;
    uint8_t _address;
    bool _available = false;
};

#endif
