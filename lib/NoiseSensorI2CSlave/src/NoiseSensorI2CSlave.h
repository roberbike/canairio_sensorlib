#ifndef NOISE_SENSOR_I2C_SLAVE_H
#define NOISE_SENSOR_I2C_SLAVE_H

#include <Arduino.h>
#include <Wire.h>
#include "NoiseSensor.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Constantes para configuracion I2C
static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x08;  // 0x08
static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;
static constexpr unsigned long MIN_UPDATE_INTERVAL = 10;     // ms
static constexpr unsigned long DEFAULT_UPDATE_INTERVAL = 1000;  // ms 1000

// Estructura de datos del sensor
struct SensorData {
  float noise;
  float noiseAvg;
  float noisePeak;
  float noiseMin;
  float noiseAvgLegal;
  float noiseAvgLegalMax;
  uint16_t lowNoiseLevel;
  uint32_t cycles;
};

// Comandos I2C
enum I2CCommand {
  CMD_GET_DATA = 0x01,      // Solicitar todos los datos
  CMD_GET_AVG = 0x02,       // Solicitar promedio
  CMD_GET_PEAK = 0x03,      // Solicitar pico
  CMD_GET_MIN = 0x04,       // Solicitar minimo
  CMD_GET_LEGAL = 0x05,     // Solicitar promedio legal
  CMD_GET_LEGAL_MAX = 0x06, // Solicitar maximo legal
  CMD_GET_STATUS = 0x07,    // Solicitar estado
  CMD_RESET = 0x08,         // Resetear ciclo
  CMD_PING = 0x09,          // Identificacion/deteccion del sensor (alias de CMD_IDENTIFY)
  CMD_IDENTIFY = 0x09,      // Identificacion y deteccion del sensor
  CMD_GET_READY = 0x0A      // Verificar si esta listo para enviar datos
};

// Estructura de identificacion del sensor
struct SensorIdentity {
  uint8_t sensorType;   // Tipo de sensor (0x01 = Noise Sensor)
  uint8_t versionMajor; // Version mayor
  uint8_t versionMinor; // Version menor
  uint8_t status;       // Estado: bit 0 = inicializado, bit 1 = ADC activo, bit 2 = datos listos
  uint8_t i2cAddress;   // Direccion I2C del sensor
} __attribute__((packed));

/**
 * Clase para manejar un sensor de ruido como esclavo I2C
 */
class NoiseSensorI2CSlave {
 public:
  // Constantes de version (publicas para facil acceso)
  static constexpr uint8_t VERSION_MAJOR = 1;
  static constexpr uint8_t VERSION_MINOR = 1;
  static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;

  /**
   * Configuracion del esclavo I2C
   */
  struct Config {
    uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;  // Direccion I2C del esclavo
#if defined(ARDUINO_ARCH_ESP8266)
    uint8_t sdaPin = 4;   // SDA por defecto en ESP8266
    uint8_t sclPin = 5;   // SCL por defecto en ESP8266
#elif defined(ARDUINO_ARCH_SAM)
    uint8_t sdaPin = SDA; // SDA por defecto en SAM
    uint8_t sclPin = SCL; // SCL por defecto en SAM
#else
    uint8_t sdaPin = 8;   // Pin SDA
    uint8_t sclPin = 10;  // Pin SCL
#endif
    uint8_t adcPin = 4;   // Pin ADC para el sensor
    unsigned long updateInterval = DEFAULT_UPDATE_INTERVAL;  // Intervalo de actualizacion en ms
    NoiseSensor::LogLevel logLevel = NoiseSensor::LOG_INFO;
  };

  /**
   * Constructor
   * @param config Configuracion del esclavo I2C
   *
   * NOTA: Solo se puede crear una instancia de NoiseSensorI2CSlave por programa
   * debido a las limitaciones de los callbacks I2C estaticos de Arduino Wire.
   */
  NoiseSensorI2CSlave(const Config& config);

  /**
   * Inicializar el esclavo I2C y el sensor
   */
  void begin();

  /**
   * Actualizar el sensor y los datos I2C (debe llamarse en loop())
   */
  void update();

  /**
   * Obtener los datos actuales del sensor
   * @return Referencia a la estructura SensorData
   */
  const SensorData& getData() const { return sensorData; }

  /**
   * Verificar si hay datos listos
   * @return true si hay datos disponibles
   */
  bool isDataReady() const { return dataReady; }

  /**
   * Obtener el objeto NoiseSensor para configuracion avanzada
   * @return Referencia al objeto NoiseSensor
   */
  NoiseSensor& getNoiseSensor() { return noiseSensor; }

  /**
   * Validar la configuracion actual
   * @return true si la configuracion es valida
   */
  bool isValid() const { return validateConfig(config); }

  /**
   * Actualizar configuracion antes de begin()
   * @param newConfig Nueva configuracion
   * @return true si la configuracion es valida y se aplico
   */
  bool setConfig(const Config& newConfig);

  /**
   * Verificar si el sensor esta inicializado correctamente
   * @return true si el sensor esta inicializado y listo para usar
   */
  bool isInitialized() const { return initialized; }

  /**
   * Verificar si el sensor esta listo para enviar datos
   * Verifica que este inicializado y que el ADC este recibiendo senal
   * @return true si esta listo para enviar datos
   */
  bool isReady() const;

  /**
   * Verificar si el ADC esta recibiendo senal del microfono
   * @return true si hay senal activa en el ADC
   */
  bool isADCActive() const { return adcActive; }

 private:
  Config config;
  NoiseSensor noiseSensor;
  SensorData sensorData;
  bool dataReady;
  bool initialized;
  bool adcActive;
  unsigned long lastUpdate;
  bool instanceOwner;
  volatile uint8_t lastCommand;
  volatile bool pendingReset;

  // Callbacks I2C (deben ser estaticos o usar punteros)
  static NoiseSensorI2CSlave* instance;
  static void IRAM_ATTR onRequestStatic();
  static void IRAM_ATTR onReceiveStatic(int numBytes);

  void onRequest();
  void onReceive(int numBytes);

  // Metodo privado para verificar senal ADC
  bool checkADCSignal();
  static bool validateConfig(const Config& cfg);
  static bool isValidGpioPin(uint8_t pin);
  static bool isValidAdcPin(uint8_t pin);
};

#endif  // NOISE_SENSOR_I2C_SLAVE_H
