#ifndef SENSOR_DFROBOT_GAS_HPP
#define SENSOR_DFROBOT_GAS_HPP

#include "ISensor.hpp"
#include <DFRobot_MultiGasSensor.h>

class SensorDFRobotGas : public ISensor {
public:
    SensorDFRobotGas(uint8_t address, SENSORS type, const char* name, TwoWire* wire = &Wire);
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return _type; }
    const char* getName() const override { return _name; }

    float getConcentration() const { return _concentration; }

private:
    DFRobot_GAS_I2C _driver;
    SENSORS _type;
    const char* _name;
    float _concentration = 0.0f;
    bool _available = false;
};

#endif
