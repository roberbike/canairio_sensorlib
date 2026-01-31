#include "SensorSenseAirS8.hpp"

SensorSenseAirS8::SensorSenseAirS8(Stream& serial) {
    _driver = new S8_UART(serial);
}

SensorSenseAirS8::~SensorSenseAirS8() {
    delete _driver;
}

bool SensorSenseAirS8::init() {
    char firm_version[32];
    _driver->get_firmware_version(firm_version);
    if (strlen(firm_version) == 0) return false;

    // Default ABC config from original
    _driver->set_ABC_period(180);
    
    _available = true;
    return true;
}

bool SensorSenseAirS8::read() {
    int co2 = _driver->get_co2();
    if (co2 > 0) {
        _co2 = (uint16_t)co2;
        return true;
    }
    return false;
}
