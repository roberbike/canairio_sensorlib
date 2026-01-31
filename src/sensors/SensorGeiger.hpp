#ifndef SENSOR_GEIGER_HPP
#define SENSOR_GEIGER_HPP

#include "ISensor.hpp"
#include <Arduino.h>
#include "utils/MovingSum.h"

/**************************************************************
 *                          GEIGER
 * ************************************************************/

#define GEIGER_TIMER 1     // timer0 is already used (at least on TTGO-TDisplay) somewhere ???
#define GEIGER_BUFSIZE 60  // moving sum buffer size (1 sample every 1s * 60 samples = 60s)
#define J305_CONV_FACTOR \
  0.008120370  // conversion factor used for conversion from CPM to uSv/h units (J305 tube)

class SensorGeiger : public ISensor {
private:
    bool devmode = false;
    int _gpio;
    uint32_t tics_cpm = 0U;  // tics in last 60s
    float uSvh = 0.0f;
    bool _available = false;

public:
    /**
     * @brief Constructor
     * @param gpio attached pin
     * @param debug debug mode enable/disable
     */
    explicit SensorGeiger(int gpio = -1, bool debug = false);
    
    // ISensor interface implementation
    bool init() override;
    bool read() override;
    bool isAvailable() const override { return _available; }
    SENSORS getSensorType() const override { return SENSORS::SCAJOE; }
    const char* getName() const override { return "CAJOE Geiger"; }
    
    // Specific getters
    uint32_t getCPM() const { return tics_cpm; }
    float getuSvh() const { return uSvh; }
    uint32_t getTics() { return this->tics_cpm; }
    float getUSvh() { return float(this->tics_cpm) * J305_CONV_FACTOR; }
    
    void clear();
};

#endif
