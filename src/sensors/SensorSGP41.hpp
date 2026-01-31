#ifndef SENSOR_SGP41_HPP
#define SENSOR_SGP41_HPP

#include "ISensor.hpp"
#include <SensirionI2CSgp41.h>

class SensorSGP41 : public ISensor {
public:
    SensorSGP41(TwoWire* wire = &Wire);
    virtual ~SensorSGP41() {}

    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SSGP41; }
    const char* getName() const override { return "SGP41"; }

    // Specific methods
    uint16_t getVocRaw() const { return _vocRaw; }
    uint16_t getNoxRaw() const { return _noxRaw; }
    
    // Logic to handle conditioning
    void setConditioningSeconds(uint8_t seconds) { _conditioning_s = seconds; }

private:
   SensirionI2CSgp41 _sgp41;
   TwoWire* _wire;
   
   uint16_t _vocRaw = 0;
   uint16_t _noxRaw = 0;
   uint8_t _conditioning_s = 10; // Default conditioning time
   bool _available = false;
};

#endif
