#ifndef SENSOR_AM2320_HPP
#define SENSOR_AM2320_HPP

#include "ISensor.hpp"
#include <AM232X.h>

class SensorAM2320 : public ISensor {
public:
    SensorAM2320(TwoWire* wire = &Wire);
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SAM232X; }
    const char* getName() const override { return "AM2320"; }

    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }

private:
    AM232X _driver;
    TwoWire* _wire;
    float _temp = 0.0f;
    float _humi = 0.0f;
    bool _available = false;
};

#endif
