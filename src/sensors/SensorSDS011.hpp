#ifndef SENSOR_SDS011_HPP
#define SENSOR_SDS011_HPP

#include "ISensor.hpp"

class SensorSDS011 : public ISensor {
public:
    SensorSDS011(Stream& serial);
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SDS011; }
    const char* getName() const override { return "SDS011"; }

    uint16_t getPM25() const { return _pm25; }
    uint16_t getPM10() const { return _pm10; }

private:
    Stream& _serial;
    uint16_t _pm25 = 0;
    uint16_t _pm10 = 0;
    bool _available = false;
};

#endif
