#ifndef SENSOR_SENSEAIR_S8_HPP
#define SENSOR_SENSEAIR_S8_HPP

#include "ISensor.hpp"
#include <s8_uart.h>

class SensorSenseAirS8 : public ISensor {
public:
    SensorSenseAirS8(Stream& serial);
    ~SensorSenseAirS8();
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SAIRS8; }
    const char* getName() const override { return "SenseAir S8"; }

    uint16_t getCO2() const { return _co2; }
    
    bool manual_calibration() { return _driver->manual_calibration(); }

private:
    S8_UART* _driver;
    uint16_t _co2 = 0;
    bool _available = false;
};

#endif
