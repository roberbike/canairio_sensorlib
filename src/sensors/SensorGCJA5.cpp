#include "SensorGCJA5.hpp"

SensorGCJA5::SensorGCJA5() {}

SensorGCJA5::SensorGCJA5(Stream& serial) : _serial(&serial) {}

bool SensorGCJA5::init() {
    if (_serial) {
        // Unfortunately SFE_PARTICLE_SENSOR library seems primarily I2C based in its public API
        // But monolithic code uses pmGCJA5Read() which reads UART manually.
        // If it's UART mode, we'll use the manual read logic.
        _available = true;
        return true;
    } else {
        if (!_driver.begin()) {
#ifdef Wire1
            if (!_driver.begin(Wire1)) {
                _available = false;
                return false;
            }
#else
            _available = false;
            return false;
#endif
        }
        _available = true;
        return true;
    }
}

bool SensorGCJA5::read() {
    if (_serial) {
        uint8_t buffer[32];
        // Minimal UART read logic from monolithic
        if (_serial->available() >= 32) {
            _serial->readBytes(buffer, 32);
            if (buffer[0] == 02) {
                _pm1 = buffer[2] * 256 + buffer[1];
                _pm25 = buffer[6] * 256 + buffer[5];
                _pm10 = buffer[10] * 256 + buffer[9];
                return true;
            }
        }
        return false;
    } else {
        if (!_driver.isConnected()) return false;
        _pm1 = _driver.getPM1_0();
        _pm25 = _driver.getPM2_5();
        _pm10 = _driver.getPM10();
        return true;
    }
}
