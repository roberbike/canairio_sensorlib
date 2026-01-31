#include "SensorPMS5003T.hpp"

SensorPMS5003T::SensorPMS5003T(Stream& serial) { 
    this->_serial = &serial; 
}

bool SensorPMS5003T::init() {
    if (this->_isBegin) {
        return true;
    }

    if (pms.begin(this->_serial) == false) {
#if defined(ESP32)
        log_e("PMS failed");
#endif
        return false;
    }

    this->_available = true;
    this->_isBegin = true;
    return true;
}

bool SensorPMS5003T::read() {
    if (!isBegin()) return false;
    
    pms.handle();
    _pm1 = pms.getPM0_1();
    _pm25 = pms.getPM2_5();
    _pm10 = pms.getPM10();
    _temp = pms.getTemp() / 10.0f;
    _humi = pms.getHum() / 10.0f;
    
    return true;
}

// Legacy methods for compatibility
int SensorPMS5003T::getPm01Ae() { return pms.getPM0_1(); }

int SensorPMS5003T::getPm25Ae() { return pms.getPM2_5(); }

int SensorPMS5003T::getPm10Ae() { return pms.getPM10(); }

int SensorPMS5003T::getPm03ParticleCount() { return pms.getCount0_3(); }

int SensorPMS5003T::convertPm25ToUsAqi(int pm25) { return pms.pm25ToAQI(pm25); }

float SensorPMS5003T::getRelativeHumidity() { return pms.getHum() / 10.0f; }

bool SensorPMS5003T::isBegin() {
    if (this->_isBegin == false) {
#if defined(ESP32)
        log_d("Not-initialized");
#endif
        return false;
    }
    return true;
}

void SensorPMS5003T::end() {
    if (_isBegin == false) {
        return;
    }
    _isBegin = false;
    _serial = nullptr;
#if defined(ESP32)
    log_d("De-initialize");
#endif
}

void SensorPMS5003T::handle() { pms.handle(); }

bool SensorPMS5003T::isFailed() { return pms.isFailed(); }
