#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include <Arduino.h>
#include "SensorTypes.hpp"

// Forward declaration if possible, but enums are likely needed here and they are in Sensors.hpp currently.
// Ideally SENSORS and UNIT enums should be moved to a separate Types.hpp file.

class ISensor {
public:
    virtual ~ISensor() {}

    /**
     * @brief Initialize the sensor
     * @return true if initialization was successful
     */
    virtual bool init() = 0;

    /**
     * @brief Perform periodic loop tasks (optional)
     */
    virtual void loop() {}

    /**
     * @brief Read data from the sensor
     * @return true if new data was read
     */
    virtual bool read() = 0;

    /**
     * @brief Check if the sensor is available and functional
     * @return true if the sensor is detected and initialized
     */
    virtual bool isAvailable() const = 0;

    /**
     * @brief Get the sensor type ID
     */
    virtual SENSORS getSensorType() const = 0;

    /**
     * @brief Get the sensor name
     */
    virtual const char* getName() const = 0;

    // Management interface (optional implementations)
    virtual void setSampleTime(int seconds) { (void)seconds; }
    virtual void setAltitude(float altitude) { (void)altitude; }
    virtual void setTemperatureOffset(float offset) { (void)offset; }
    virtual void setSeaLevelPressure(float hpa) { (void)hpa; }
    virtual bool calibrate(int value = 0) { (void)value; return false; }
    virtual void setAmbientData(float temp, float hum, float pres) { (void)temp; (void)hum; (void)pres; }
};

#endif // ISENSOR_HPP
