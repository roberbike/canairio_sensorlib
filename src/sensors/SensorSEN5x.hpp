#ifndef SENSOR_SEN5X_HPP
#define SENSOR_SEN5X_HPP

#include "ISensor.hpp"
#include <SensirionI2CSen5x.h>

class SensorSEN5x : public ISensor {
public:
    SensorSEN5x(TwoWire* wire = &Wire);
    virtual ~SensorSEN5x() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSEN5X; }
    const char* getName() const override { return "SEN5x"; }

    // Specific methods
    float getPm1p0() const { return _pm1p0; }
    float getPm2p5() const { return _pm2p5; }
    float getPm4p0() const { return _pm4p0; }
    float getPm10p0() const { return _pm10p0; }
    float getHumidity() const { return _humi; }
    float getTemperature() const { return _temp; }
    float getVocIndex() const { return _vocIndex; }
    float getNoxIndex() const { return _noxIndex; }

    // Configuration
    void setTemperatureOffset(float offset);

private:
   SensirionI2CSen5x _sen5x;
   TwoWire* _wire;
   
   float _pm1p0 = 0.0f;
   float _pm2p5 = 0.0f;
   float _pm4p0 = 0.0f;
   float _pm10p0 = 0.0f;
   float _humi = 0.0f;
   float _temp = 0.0f;
   float _vocIndex = 0.0f;
   float _noxIndex = 0.0f;
   bool _available = false;
};

#endif
