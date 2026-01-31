#ifndef SENSOR_SPS30_HPP
#define SENSOR_SPS30_HPP

#include "ISensor.hpp"
#include <sps30.h>

class SensorSPS30 : public ISensor {
public:
    SensorSPS30(); // Default I2C
    SensorSPS30(Stream& serial); // UART
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSPS30; }
    const char* getName() const override { return "SPS30"; }

    uint16_t getPM1() const { return _pm1; }
    uint16_t getPM25() const { return _pm25; }
    uint16_t getPM4() const { return _pm4; }
    uint16_t getPM10() const { return _pm10; }

private:
    SPS30 _driver;
    Stream* _serial = nullptr;
    uint16_t _pm1 = 0, _pm25 = 0, _pm4 = 0, _pm10 = 0;
    
    bool _available = false;
    bool probe();
};

#endif
