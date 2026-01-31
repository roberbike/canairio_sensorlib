#include "SensorDFRobotGas.hpp"

SensorDFRobotGas::SensorDFRobotGas(uint8_t address, SENSORS type, const char* name, TwoWire* wire) 
    : _driver(wire, address), _type(type), _name(name) {}

bool SensorDFRobotGas::init() {
    if (!_driver.begin()) return false;
    _driver.changeAcquireMode(_driver.PASSIVITY);
    _driver.setTempCompensation(_driver.ON);
    _available = true;
    return true;
}

bool SensorDFRobotGas::read() {
    // Note: DFRobot driver might not check for presence inside readGasConcentrationPPM
    _concentration = _driver.readGasConcentrationPPM();
    return true;
}
