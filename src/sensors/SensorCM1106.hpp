#ifndef SENSOR_CM1106_HPP
#define SENSOR_CM1106_HPP

#include "ISensor.hpp"
#include <cm1106_uart.h>

class SensorCM1106 : public ISensor {
public:
    SensorCM1106(Stream& serial);
    ~SensorCM1106();
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SCM1106; }
    const char* getName() const override { return "CM1106"; }

    uint16_t getCO2() const { return _co2; }
    
    void start_calibration(int ppm) { _driver->start_calibration(ppm); }
    bool get_ABC(CM1106_ABC* abc) { return _driver->get_ABC(abc); }

private:
    CM1106_UART* _driver;
    uint16_t _co2 = 0;
    bool _available = false;
};

#endif
