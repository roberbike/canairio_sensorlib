#ifndef SENSOR_PM1006_HPP
#define SENSOR_PM1006_HPP

#include "ISensor.hpp"
#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum { PM1006_HEADER, PM1006_LENGTH, PM1006_DATA, PM1006_CHECK } pm1006_state_t;

class SensorPM1006 : public ISensor {
private:
    Stream* _serial;
    bool _debug;

    pm1006_state_t _state;
    size_t _rxlen;
    size_t _index;
    uint8_t _txbuf[16];
    uint8_t _rxbuf[20];
    uint8_t _checksum;

    uint16_t _pm25 = 0;

    bool send_command(size_t cmd_len, const uint8_t* cmd_data);
    int build_tx(size_t cmd_len, const uint8_t* cmd_data);
    bool process_rx(uint8_t c);
    bool read_pm25_internal(uint16_t* pm);

public:
    static const int BIT_RATE = 9600;

    /**
     * Constructor.
     * @param serial the serial port, NOTE: the serial port has to be configured for a bit rate of
     * SensorPM1006::BIT_RATE !
     */
    explicit SensorPM1006(Stream& serial, bool debug = false);

    // ISensor interface implementation
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::IKEAVK; }
    const char* getName() const override { return "PM1006"; }

    // Specific getters
    uint16_t getPM25() const { return _pm25; }
    
private:
    bool _available = false;
};

#endif
