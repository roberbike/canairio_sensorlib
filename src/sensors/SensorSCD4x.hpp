#ifndef SENSOR_SCD4X_HPP
#define SENSOR_SCD4X_HPP

#include "ISensor.hpp"
#include <SensirionI2CScd4x.h>

class SensorSCD4x : public ISensor {
public:
    SensorSCD4x(TwoWire* wire = &Wire);
    virtual ~SensorSCD4x() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSCD4X; }
    const char* getName() const override { return "SCD4X"; }

    // Specific methods
    float getCO2() const { return _co2; }
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }
    
    // Configuration methods specific to SCD4x
    void setTemperatureOffset(float offset);
    float getTemperatureOffset();
    void setSensorAltitude(uint16_t altitude);
    uint16_t performForcedRecalibration(uint16_t targetCO2, uint16_t &frcCorrection);

private:
   SensirionI2CScd4x _scd4x;
   TwoWire* _wire;
   
   float _co2 = 0.0f;
   float _temp = 0.0f;
   float _humi = 0.0f;
   bool _available = false;
};

#endif
