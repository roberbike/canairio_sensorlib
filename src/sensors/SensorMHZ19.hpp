#ifndef SENSOR_MHZ19_HPP
#define SENSOR_MHZ19_HPP

#include "ISensor.hpp"
#include <MHZ19.h>

class SensorMHZ19 : public ISensor {
public:
    SensorMHZ19(Stream& serial);
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SMHZ19; }
    const char* getName() const override { return "MHZ19"; }

    uint16_t getCO2() const { return _co2; }
    float getTemperature() const { return _temp; }

    void calibrate() { _mhz19.calibrate(); }

private:
    MHZ19 _mhz19;
    Stream& _serial;
    uint16_t _co2 = 0;
    float _temp = 0.0f;
    bool _available = false;
};

#endif
