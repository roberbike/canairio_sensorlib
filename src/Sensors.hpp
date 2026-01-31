#ifndef Sensors_hpp
#define Sensors_hpp

#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>
#include <sps30.h>

#include "sensors/SensorAHT10.hpp"
#include "sensors/SensorAM2320.hpp"
#include "sensors/SensorBME280.hpp"
#include "sensors/SensorBME680.hpp"
#include "sensors/SensorBMP280.hpp"
#include "sensors/SensorCM1106.hpp"
#include "sensors/SensorDFRobotGas.hpp"
#include "sensors/SensorGCJA5.hpp"
#include "sensors/SensorGeiger.hpp"
#include "sensors/SensorMHZ19.hpp"
#include "sensors/SensorNoise.hpp"
#include "sensors/SensorPM1006.hpp"
#include "sensors/SensorPMS5003T.hpp"
#include "sensors/SensorSCD30.hpp"
#include "sensors/SensorSCD4x.hpp"
#include "sensors/SensorSDS011.hpp"
#include "sensors/SensorSEN5x.hpp"
#include "sensors/SensorSGP41.hpp"
#include "sensors/SensorSHT31.hpp"
#include "sensors/SensorSPS30.hpp"
#include "sensors/SensorSenseAirS8.hpp"

#if (defined(ARDUINO_ARCH_ESP32) &&                                               \
     (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
      defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32C3_DEV) ||       \
      defined(ARDUINO_ESP32S2_DEV) || defined(ARDUINO_ESP32S3_DEV) ||             \
      defined(ARDUINO_LOLIN_C3_MINI) || defined(ARDUINO_LOLIN_S2_MINI) ||         \
      defined(ARDUINO_LOLIN_S3_MINI) || defined(ESP32C3) || defined(ESP32S2) ||   \
      defined(ESP32S3))) ||                                                       \
    defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
#define CSL_NOISE_SENSOR_SUPPORTED 1
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define CSL_NOISE_SENSOR_USE_EXTERNAL 1
#endif

#ifdef CSL_NOISE_SENSOR_SUPPORTED
#if defined(CSL_NOISE_SENSOR_USE_EXTERNAL)
#include <NoiseSensorI2CSlave.h>
#else
#if defined(__has_include)
#if __has_include("sensors/noise/NoiseSensorI2CSlave.h")
#include "sensors/noise/NoiseSensorI2CSlave.h"
#else
#undef CSL_NOISE_SENSOR_SUPPORTED
#endif
#else
#include "sensors/noise/NoiseSensorI2CSlave.h"
#endif
#endif
#endif

#ifdef CSL_NOISE_SENSOR_SUPPORTED
#ifndef SENSOR_TYPE_NOISE
#define SENSOR_TYPE_NOISE 0x01
#endif
#endif

#ifdef DHT11_ENABLED
#include <dht_nonblocking.h>
#endif

#define CSL_VERSION "0.7.6"
#define CSL_REVISION 385

/***************************************************************
 * S E T U P   E S P 3 2   B O A R D S   A N D   F I E L D S
 ***************************************************************/

// provisional pins. Config those via CLI
#ifdef WEMOSOLED
#define PMS_RX 13  // config for Wemos board & TTGO18650
#define PMS_TX 15  // some old TTGO18650 have PMS_RX 18 & PMS_TX 17
#elif HELTEC
#define PMS_RX 17
#define PMS_TX 18
#elif TTGO_TQ
#define PMS_RX 13
#define PMS_TX 18
#elif M5COREINK
#define PMS_RX 13
#define PMS_TX 14
#elif TTGO_TDISPLAY
#define PMS_RX 13
#define PMS_TX 12
#elif ESP32PICOD4
#define PMS_RX 19
#define PMS_TX 18
#elif ESP32GENERIC
#define PMS_RX RX
#define PMS_TX TX
#elif M5STICKCPLUS
#define PMS_RX 36
#define PMS_TX 0
#elif M5COREINK
#define PMS_RX 13
#define PMS_TX 14
#elif M5ATOM
#define PMS_RX 23
#define PMS_TX 33
#elif M5PICOD4
#define PMS_RX 3
#define PMS_TX 1
#elif AG_OPENAIR
#define PMS_RX 0
#define PMS_TX 1
#define AIRG_SDA 7
#define AIRG_SCL 6
#elif TTGO_T7
#define PMS_RX 17
#define PMS_TX 16
#elif ESP32DEVKIT
#define PMS_RX 17
#define PMS_TX 16
#else
#define PMS_RX -1  // DEFAULTS for other boards. Please setup it via CLI
#define PMS_TX -1
#endif

// I2C pins for M5COREINK and M5STICKCPLUS
#define HAT_I2C_SDA 0
#define HAT_I2C_SCL 26
#define EXT_I2C_SDA 32
#define EXT_I2C_SCL 33

#ifdef M5AIRQ
#define GROVE_SDA 13
#define GROVE_SCL 15
#define I2C1_SDA_PIN 11
#define I2C1_SCL_PIN 12
#endif

#ifdef TTGO_T7S3
#define GROVE_SDA 13
#define GROVE_SCL 14
#define I2C1_SDA_PIN 8
#define I2C1_SCL_PIN 9
#endif

// Read UART sensor retry.
#define SENSOR_RETRY 1000  // Max Serial characters

// UART defualt port
#define SENSOR_COMMS SERIALPORT2

#include "SensorTypes.hpp"

typedef void (*errorCbFn)(const char* msg);
typedef void (*voidCbFn)();

/**
 * @brief CanAirIO Sensors Manager main class.
 * @authors \@hpsaturn and CanAir.IO contributers
 */
class Sensors {
 public:
  /// SPS30 values. Only for Sensirion SPS30 sensor.
  struct sps_values val;

  /// Debug mode for increase verbose.
  bool devmode;

  /// Initial sample time for all sensors
  int sample_time = 10;

  /// Temperature offset (for final temp output)
  float toffset = 0.0;

  /// Altitud compensation variable
  float altoffset = 0.0;

  /// Sea level pressure (hPa)
  float sealevel = 1013.25;

  /// Altitud hpa calculation
  float hpa = 0.0;

  /// Sensirion dust SPS30 library (legacy path)
  SPS30 sps30;

  virtual ~Sensors();

  void init(unsigned int pms_type = 0, int pms_rx = PMS_RX, int pms_tx = PMS_TX);

  void loop();

  bool readAllSensors();

  bool isDataReady();

  void setSampleTime(int seconds);

  void setOnDataCallBack(voidCbFn cb);

  void setOnErrorCallBack(errorCbFn cb);

  void setTemperatureUnit(TEMPUNIT tunit);

  void setDebugMode(bool enable);

  bool isUARTSensorConfigured();

  int getUARTDeviceTypeSelected();

  uint16_t getPM1();

  uint16_t getPM25();

  uint16_t getPM4();

  uint16_t getPM10();

  uint16_t getCO2();

  float getCO2humi();

  float getCO2temp();

  float getTemperature();

  float getHumidity();

  float getPressure();

  float getAltitude();

  float getGas();

  float getNH3();

  float getCO();

  float getNO2();

  float getO3();

  void enableGeigerSensor(int gpio);

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  float getNoise();

  float getNoiseAverage();

  float getNoisePeak();

  float getNoiseMin();

  float getNoiseLegalAverage();

  float getNoiseLegalMaximum();
#endif

  uint32_t getGeigerCPM(void);

  float getGeigerMicroSievertHour(void);

  void initTOffset(float offset);

  float getTOffset();

  void setTempOffset(float offset);

  float getTempOffset();

  void setCO2AltitudeOffset(float altitude);

  void setSeaLevelPressure(float hpa);

  void setCO2RecalibrationFactor(int ppmValue);

  void detectI2COnly(bool enable);

  String getLibraryVersion();

  int16_t getLibraryRevision();

  bool isSensorRegistered(SENSORS sensor);

  uint8_t* getSensorsRegistered();

  uint8_t getSensorsRegisteredCount();

  String getSensorName(SENSORS sensor);

  SensorGroup getSensorGroup(SENSORS sensor);

  uint8_t getUnitsRegisteredCount();

  bool isUnitRegistered(UNIT unit);

  String getUnitName(UNIT unit);

  String getUnitSymbol(UNIT unit);

  UNIT getNextUnit();

  void resetUnitsRegister();

  void resetSensorsRegister();

  void resetNextUnit();

  void resetAllVariables();

  float getUnitValue(UNIT unit);

  void printUnitsRegistered(bool debug = false);

  void printSensorsRegistered(bool debug = false);

  void startI2C();

  // Modularized sensors
  SensorSCD30* _sensorSCD30 = nullptr;
  SensorSHT31* _sensorSHT31 = nullptr;
  SensorBME280* _sensorBME280 = nullptr;
  SensorBMP280* _sensorBMP280 = nullptr;
  SensorBME680* _sensorBME680 = nullptr;
  SensorAHT10* _sensorAHT10 = nullptr;
  SensorSCD4x* _sensorSCD4x = nullptr;
  SensorSEN5x* _sensorSEN5x = nullptr;
  SensorSGP41* _sensorSGP41 = nullptr;
  SensorNoise* _sensorNoise = nullptr;
  SensorMHZ19* _sensorMHZ19 = nullptr;
  SensorCM1106* _sensorCM1106 = nullptr;
  SensorSenseAirS8* _sensorS8 = nullptr;
  SensorPM1006* _sensorPM1006 = nullptr;
  SensorPMS5003T* _sensorPMS5003T = nullptr;
  SensorSDS011* _sensorSDS011 = nullptr;
  SensorSPS30* _sensorSPS30 = nullptr;
  SensorGCJA5* _sensorGCJA5 = nullptr;
  SensorAM2320* _sensorAM2320 = nullptr;
  SensorDFRobotGas* _sensorDFRCO = nullptr;
  SensorDFRobotGas* _sensorDFRNH3 = nullptr;
  SensorDFRobotGas* _sensorDFRNO2 = nullptr;
  SensorDFRobotGas* _sensorDFRO3 = nullptr;
  SensorGeiger* _sensorGeiger = nullptr;

 private:
  ISensor* _active_sensors[SCOUNT] = {nullptr};
  uint8_t _active_sensors_count = 0;
  void registerSensor(ISensor* s);
#ifdef DHT11_ENABLED
  /// DHT library
  uint32_t delayMS;
#endif
  /// For UART sensors (autodetected available serial)
  Stream* _serial;
  /// Callback on some sensors error.
  errorCbFn _onErrorCb = nullptr;
  /// Callback when sensor data is ready.
  voidCbFn _onDataCb = nullptr;

  int dev_uart_type = -1;
  bool i2conly = false;

  bool dataReady;

  bool readAllComplete = false;

  uint8_t sensors_registered_count;

  uint8_t units_registered_count;

  uint8_t current_unit = 0;

  uint16_t pm1;   // PM1
  uint16_t pm25;  // PM2.5
  uint16_t pm4;   // PM4
  uint16_t pm10;  // PM10

  float humi = 0.0;  // % Relative humidity
  float temp = 0.0;  // Temperature (°C)
  float pres = 0.0;  // Pressure
  float alt = 0.0;
  float gas = 0.0;
  float voci = 0.0;
  float noxi = 0.0;
  uint16_t voc = 0;
  uint16_t nox = 0;

  // temperature unit (C,K,F)
  TEMPUNIT temp_unit = TEMPUNIT::CELSIUS;

  uint16_t CO2Val;      // CO2 in ppm
  float CO2humi = 0.0;  // humidity of CO2 sensor
  float CO2temp = 0.0;  // temperature of CO2 sensor

  float nh3;  // Amonium in ppm
  float co;   // Carbon monoxide in ppm
  float no2;  // Nitrogen dioxide in ppm
  float o3;   // Ozone in ppm

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  TwoWire* noiseWire = nullptr;
  SensorData noiseSensorData{};
  bool noiseWireReady = false;
  uint8_t noiseSensorAddress = 0;
#if __cplusplus >= 201103L
  static_assert(sizeof(SensorData) == 32, "SensorData size mismatch");
#endif
#endif
  bool noiseSensorEnabled = false;
  float noiseInstant = 0.0;
  float noiseAvgValue = 0.0;
  float noisePeakValue = 0.0;
  float noiseMinValue = 0.0;
  float noiseAvgLegalValue = 0.0;
  float noiseAvgLegalMaxValue = 0.0;
  bool noiseScanDone = false;

  void am2320Init();
  void am2320Read();

  void bme280Init();
  void bme280Read();

  void bmp280Init();
  void bmp280Read();

  void bme680Init();
  void bme680Read();

  void aht10Init();
  void aht10Read();

  void sht31Init();
  void sht31Read();

  void CO2scd30Init();
  void CO2scd30Read();
  void setSCD30TempOffset(float offset);
  float getSCD30TempOffset();
  void setSCD30AltitudeOffset(float offset);
  void CO2correctionAlt();
  float hpaCalculation(float altitude);

  void CO2scd4xInit();
  void CO2scd4xRead();
  void setSCD4xTempOffset(float offset);
  float getSCD4xTempOffset();
  void setSCD4xAltitudeOffset(float offset);

  void sen5xInit();
  void sen5xRead();
  void setsen5xTempOffset(float offset);

  void sgp41Init();
  void sgp41Read();

  void GCJA5Init();
  void GCJA5Read();

#ifdef DHT11_ENABLED
  void dhtInit();
  void dhtRead();
  bool dhtIsReady(float* temperature, float* humidity);
#endif

  void DFRobotNH3Init();
  void DFRobotNH3Read();
  void DFRobotCOInit();
  void DFRobotCORead();
  void DFRobotNO2Init();
  void DFRobotNO2Read();
  void DFRobotO3Init();
  void DFRobotO3Read();

  // UART sensors methods:

  bool sensorSerialInit(unsigned int pms_type, int rx, int tx);
  bool pmSensorAutoDetect(unsigned int pms_type);
  bool pmSensorRead();
  bool pmGenericRead();
  bool pmGCJA5Read();
  bool pmSDS011Read();
  bool pm1006Read();
  bool pm5003TRead();
  bool CO2Mhz19Read();
  bool CO2CM1106Read();
  bool CO2Mhz19Init();
  bool CO2CM1106Init();
  bool senseAirS8Init();
  bool senseAirS8Read();
  bool PM1006Init();
  bool PM5003TInit();

  bool sps30I2CInit();
  bool sps30UARTInit();
  bool sps30Read();
  bool sps30tests();
  void sps30ErrToMess(char* mess, uint8_t r);
  void sps30Errorloop(char* mess, uint8_t r);
  void sps30DeviceInfo();

  void geigerRead();

  void onSensorError(const char* msg);

  void enableWire1();

  void disableWire1();

  bool serialInit(unsigned int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx);

  size_t hwSerialRead(uint8_t* buffer, size_t length);

  void restart();  // restart serial (it isn't works sometimes)

  void DEBUG(const char* text, const char* textb = "");

  void printValues();

  void printHumTemp();

  void tempRegister(bool isCO2temp);

  void sensorRegister(SENSORS sensor);

  void sensorAnnounce(SENSORS sensor);

  void unitRegister(UNIT unit);

  uint8_t* getUnitsRegistered();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  bool noiseSensorAutoDetect();
  void noiseSensorService();
  void noiseSensorCollect();
  bool noiseSensorReadIdentity(TwoWire& wire, uint8_t address, SensorIdentity& out);
  bool noiseSensorReadData(TwoWire& wire, uint8_t address, SensorData& out);
  bool noiseSensorDevicePresent(TwoWire& wire, uint8_t address);
  void noiseSensorInitWire();
#endif

// @todo use DEBUG_ESP_PORT ?
#ifdef WM_DEBUG_PORT
  Stream& _debugPort = WM_DEBUG_PORT;
#else
  Stream& _debugPort = Serial;  // debug output stream ref
#endif
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
extern Sensors sensors;
#endif

#endif
