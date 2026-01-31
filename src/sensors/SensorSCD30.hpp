#ifndef SENSOR_SCD30_HPP
#define SENSOR_SCD30_HPP

#include "ISensor.hpp"
#include <Adafruit_SCD30.h>

class SensorSCD30 : public ISensor {
public:
    SensorSCD30(float* toffset_ptr, float* altoffset_ptr);
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSCD30; }
    const char* getName() const override { return "SCD30"; }

    // Specific methods for this sensor
    void setTemperatureOffset(float offset);
    void setAltitudeOffset(uint16_t altitude);
    uint16_t getTemperatureOffset() { return _scd30.getTemperatureOffset(); }
    uint16_t getAltitudeOffset() { return _scd30.getAltitudeOffset(); }
    uint16_t getCO2() const { return _co2; }
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }

private:
    Adafruit_SCD30 _scd30;
    float* _global_toffset;   // Pointer to global config
    float* _global_altoffset; // Pointer to global config
    
    bool _available = false;
    uint16_t _co2 = 0;
    float _temp = 0.0f;
    float _humi = 0.0f;
};

#endif
