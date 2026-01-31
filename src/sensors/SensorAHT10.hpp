#ifndef SENSOR_AHT10_HPP
#define SENSOR_AHT10_HPP

#include "ISensor.hpp"
#include <AHTxx.h>

class SensorAHT10 : public ISensor {
public:
    SensorAHT10(uint8_t address = AHTXX_ADDRESS_X38, TwoWire* wire = &Wire);
    virtual ~SensorAHT10() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SAHTXX; }
    const char* getName() const override { return "AHT10"; }

    // Specific methods
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }

private:
   AHTxx _aht10;
   uint8_t _address;
   TwoWire* _wire;
   
   float _temp = 0.0f;
   float _humi = 0.0f;
   bool _available = false;
};

#endif
