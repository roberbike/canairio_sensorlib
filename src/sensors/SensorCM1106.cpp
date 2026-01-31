#include "SensorCM1106.hpp"

SensorCM1106::SensorCM1106(Stream& serial) {
    _driver = new CM1106_UART(serial);
}

SensorCM1106::~SensorCM1106() {
    delete _driver;
}

bool SensorCM1106::init() {
    char softver[32];
    _driver->get_software_version(softver);
    if (strlen(softver) == 0) return false;

    // Setup ABC parameters (defaults from original code)
    _driver->set_ABC(CM1106_ABC_OPEN, 7, 415);
    _driver->set_working_status(1);
    
    _available = true;
    return true;
}

bool SensorCM1106::read() {
    int co2 = _driver->get_co2();
    if (co2 > 0) {
        _co2 = (uint16_t)co2;
        return true;
    }
    return false;
}
