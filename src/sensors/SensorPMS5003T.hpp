#ifndef SENSOR_PMS5003T_HPP
#define SENSOR_PMS5003T_HPP

#include "ISensor.hpp"
#include <HardwareSerial.h>
#include "pms/PMS.h"
#include "pms/PMS5003TBase.h"
#include <Stream.h>

/**
 * @brief Sensor class for PMS5003T (Plantower PM sensor with T&H)
 * Inherits from ISensor and uses PMSBase for protocol handling
 */
class SensorPMS5003T : public ISensor, public PMS5003TBase {
public:
    explicit SensorPMS5003T(Stream& serial);
    
    // ISensor interface implementation
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::P5003T; }
    const char* getName() const override { return "PMS5003T"; }
    
    // Specific getters
    uint16_t getPM1() const { return _pm1; }
    uint16_t getPM25() const { return _pm25; }
    uint16_t getPM10() const { return _pm10; }
    float getTemperature() const { return _temp; }
    float getHumidity() const { return _humi; }
    
    // Legacy methods for compatibility
    void end();
    void handle();
    bool isFailed();
    int getPm01Ae();
    int getPm25Ae();
    int getPm10Ae();
    int getPm03ParticleCount();
    int convertPm25ToUsAqi(int pm25);
    float getRelativeHumidity();

private:
    bool _isBegin = false;
    bool _isSleep = false;
    Stream* _serial;
    PMSBase pms;
    
    uint16_t _pm1 = 0;
    uint16_t _pm25 = 0;
    uint16_t _pm10 = 0;
    float _temp = 0.0f;
    float _humi = 0.0f;
    bool _available = false;
    
    bool isBegin();
};

#endif /** SENSOR_PMS5003T_HPP */
