#include "SensorSDS011.hpp"

SensorSDS011::SensorSDS011(Stream& serial) : _serial(serial) {}

bool SensorSDS011::init() {
    // SDS011 is passive in the sense it just sends data
    _available = read();
    return _available;
}

bool SensorSDS011::read() {
    uint8_t buffer[10];
    // This is a bit tricky as we need to wait for the sync byte
    // For now, mirroring the logic from monolithic read
    if (_serial.available() < 10) return false;
    
    if (_serial.read() == 170) { // 0xAA
        if (_serial.read() == 192) { // 0xC0
            _serial.readBytes(&buffer[2], 8);
            _pm25 = (buffer[3] * 256 + buffer[2]) / 10;
            _pm10 = (buffer[5] * 256 + buffer[4]) / 10;
            return true;
        }
    }
    return false;
}
