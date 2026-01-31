#ifndef NOISE_SENSOR_I2C_SLAVE_H
#define NOISE_SENSOR_I2C_SLAVE_H

#if !defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ARCH_ESP32C3) && !defined(ARDUINO_ARCH_ESP32S2) && !defined(ARDUINO_ARCH_ESP32S3) && !defined(ESP32)

#include <Arduino.h>
#include <Wire.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// I2C Constants
static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x08;
static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;
static constexpr unsigned long MIN_UPDATE_INTERVAL = 10;
static constexpr unsigned long DEFAULT_UPDATE_INTERVAL = 1000;

#include "NoiseProtocol.h"

/**
 * @brief Integrated Noise Sensor as I2C Slave
 */
class NoiseSensorI2CSlave {
 public:
  enum LogLevel {
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_INFO = 3,
    LOG_DEBUG = 4,
    LOG_VERBOSE = 5
  };

  static constexpr uint8_t VERSION_MAJOR = 1;
  static constexpr uint8_t VERSION_MINOR = 1;
  static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;

  // Combined Configuration
  struct Config {
    // I2C parameters
    uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;
#if defined(ARDUINO_ARCH_ESP8266)
    uint8_t sdaPin = 4;
    uint8_t sclPin = 5;
#elif defined(ARDUINO_ARCH_SAM)
    uint8_t sdaPin = SDA;
    uint8_t sclPin = SCL;
#else
    uint8_t sdaPin = 8;
    uint8_t sclPin = 10;
#endif  // pin defaults

    // ADC and sensor parameters
    uint8_t adcPin = 4;
    unsigned long dutyCycle = 120000;  // Main cycle period (ms)
    unsigned long legalPeriod = 5000;  // Legal average period (ms)
    int lowNoiseLevel = 36;            // Initial base noise level
    int outlierThreshold = 4095;       // ADC outlier threshold
    unsigned long updateInterval = DEFAULT_UPDATE_INTERVAL;
    LogLevel logLevel = LOG_INFO;
  };

  NoiseSensorI2CSlave(const Config& config);

  void begin();
  void update();

  const SensorData& getData() const { return sensorData; }
  bool isDataReady() const { return dataReady; }
  bool isValid() const { return validateConfig(config); }
  bool isInitialized() const { return initialized; }
  bool isReady() const { return initialized && adcActive; }
  bool isADCActive() const { return adcActive; }

 private:
  Config config;
  SensorData sensorData;

  bool dataReady;
  bool initialized;
  bool adcActive;
  unsigned long lastUpdate;
  bool instanceOwner;
  volatile uint8_t lastCommand;
  volatile bool pendingReset;

  // Internal measurement state
  unsigned long tmpIni;
  unsigned long countStart;
  unsigned long legalStart;
  int loops;
  int loopsLegal;
  unsigned long noiseSum;
  unsigned long noiseSumLegal;
  float icycles;
  bool cycleComplete;

  // Measurement methods
  unsigned int readADC_mV();
  void calculateLegalAverage();
  void processMainCycle();
  void resetCycle();
  bool checkADCSignal();

  // Logging helpers
  void log(LogLevel level, const char* message) const;
  void log(LogLevel level, const char* prefix, float value) const;
  inline bool shouldLog(LogLevel level) const { return level <= config.logLevel; }

  // I2C Static stuff
  static NoiseSensorI2CSlave* instance;
  static void IRAM_ATTR onRequestStatic();
  static void IRAM_ATTR onReceiveStatic(int numBytes);
  void onRequest();
  void onReceive(int numBytes);

  static bool validateConfig(const Config& cfg);
  static bool isValidGpioPin(uint8_t pin);
  static bool isValidAdcPin(uint8_t pin);
};

#endif  // !ESP32 family

#endif  // NOISE_SENSOR_I2C_SLAVE_H
