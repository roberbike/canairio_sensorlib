#include "Sensors.hpp"

#include <math.h>

#include "utils/SerialHelper.hpp"

// Units and sensors registers

#define X(unit, symbol, name) symbol,
const char *unit_symbol[] = {SENSOR_UNITS};
#undef X

#define X(unit, symbol, name) name,
char const *unit_name[] = {SENSOR_UNITS};
#undef X

uint8_t units_registered[UCOUNT];

#define X(utype, uname, umaintype) uname,
char const *sensors_device_names[] = {SENSORS_TYPES};
#undef X

#define X(utype, uname, umaintype) umaintype,
int sensors_device_types[] = {SENSORS_TYPES};
#undef X

uint8_t sensors_registered[SCOUNT];

/***********************************************************************************
 *  P U B L I C   M E T H O D S
 * *********************************************************************************/

Sensors::~Sensors() {
  for (int i = 0; i < _active_sensors_count; i++) {
    if (_active_sensors[i]) {
      delete _active_sensors[i];
      _active_sensors[i] = nullptr;
    }
  }
}

void Sensors::registerSensor(ISensor *s) {
  if (s == nullptr || _active_sensors_count >= SCOUNT) return;
  _active_sensors[_active_sensors_count++] = s;
  sensorRegister(s->getSensorType());
}

/**
 * @brief Main sensors loop.
 * All sensors are read here, please call it on main loop.
 */
void Sensors::loop() {
  // Allow sensors to perform background tasks
  for (int i = 0; i < _active_sensors_count; i++) {
    if (_active_sensors[i] && _active_sensors[i]->isAvailable()) {
      _active_sensors[i]->loop();
    }
  }

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorService();
#endif
  static uint32_t pmLoopTimeStamp = 0;  // timestamp for sensor loop check data
  if ((millis() - pmLoopTimeStamp >
       sample_time * (uint32_t)1000)) {  // sample time for each capture
    pmLoopTimeStamp = millis();
    readAllSensors();

    if (!dataReady) DEBUG("-->[SLIB] any data from sensors\t: ? check your wirings!");

    if (dataReady && (_onDataCb != nullptr)) {
      _onDataCb();  // if any sensor reached any data, dataReady is true.
    } else if (!dataReady && (_onErrorCb != nullptr))
      _onErrorCb("[W][SLIB] sensorslib error msg\t: No data from any sensor!");
  }

#ifdef DHT11_ENABLED
  dhtRead();  // DHT2x sensors need check fastest
#endif
}

void Sensors::printHumTemp() {
  Serial.printf("-->[SLIB] sensorlib \t\t: T:%02.1f, H:%02.1f\r\n", humi, temp);
}

/**
 * @brief Read all sensors but use only one time or use loop() instead.
 * All sensors are read here. Use it carefully, better use sensors.loop()
 */
bool Sensors::readAllSensors() {
  readAllComplete = false;
  if (!i2conly && dev_uart_type >= 0) {
    dataReady = pmSensorRead();
    DEBUG("-->[SLIB] UART data ready \t:", dataReady ? "true" : "false");
  }
  enableWire1();

  // New modular reading loop
  // Note: We still call original Read methods for now as they contain
  // specific logic for mapping values to member variables like CO2Val, temp, etc.
  // This will be consolidated in a follow-up step.
  CO2scd30Read();
  GCJA5Read();
  CO2scd4xRead();
  if (!sps30Read()) {
    sen5xRead();
  }
  am2320Read();
  sht31Read();
  bme280Read();
  bmp280Read();
  bme680Read();
  aht10Read();
  DFRobotCORead();
  DFRobotNH3Read();
  DFRobotNO2Read();
  DFRobotO3Read();
  geigerRead();
  sgp41Read();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorCollect();
#endif

#ifdef DHT11_ENABLED
  dhtRead();
#endif

  disableWire1();

  printValues();
  printSensorsRegistered(devmode);
  printUnitsRegistered(devmode);

  readAllComplete = dataReady;
  return dataReady;
}

/**
 * @brief All sensors init.
 * @param pms_type (optional) UART PMS type, please see DEVICE_TYPE enum.
 * @param pms_rx (optional) UART PMS RX pin.
 * @param pms_tx (optional) UART PMS TX pin.
 */
void Sensors::init(unsigned int pms_type, int pms_rx, int pms_tx) {
// override with debug INFO level (>=3)
#ifdef CORE_DEBUG_LEVEL
  if (CORE_DEBUG_LEVEL >= 3) devmode = true;
#endif
  if (devmode) {
    Serial.printf("-->[SLIB] CanAirIO SensorsLib\t: v%sr%d\r\n", CSL_VERSION, CSL_REVISION);
    Serial.printf("-->[SLIB] sensorslib devmod\t: %s\r\n", devmode ? "true" : "false");
  }

  Serial.println("-->[SLIB] temperature offset\t: " + String(toffset));
  Serial.println("-->[SLIB] altitude offset   \t: " + String(altoffset));
  Serial.println("-->[SLIB] sea level pressure\t: " + String(sealevel) + " hPa");
  Serial.printf("-->[SLIB] only i2c sensors  \t: %s\r\n", i2conly ? "true" : "false");

  if (!i2conly && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
    DEBUG("-->[SLIB] UART sensors detected\t:", "0");
  }

  startI2C();
  CO2scd30Init();
  GCJA5Init();
  CO2scd4xInit();
  if (!sps30I2CInit()) {
    sen5xInit();
  }
  bmp280Init();
  bme280Init();
  bme680Init();
  am2320Init();
  sht31Init();
  aht10Init();
  DFRobotCOInit();
  DFRobotNH3Init();
  DFRobotNO2Init();
  DFRobotO3Init();
  sgp41Init();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorAutoDetect();
#endif

#ifdef DHT11_ENABLED
  dhtInit();
#endif

  printSensorsRegistered(true);
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds) {
  sample_time = seconds;
  if (devmode) Serial.printf("-->[SLIB] new sample time\t: %d\r\n", seconds);

  for (int i = 0; i < _active_sensors_count; i++) {
    if (_active_sensors[i]) {
      _active_sensors[i]->setSampleTime(seconds);
    }
  }
}

/**
 * @brief set CO2 recalibration PPM value (400 to 2000)
 * @param ppmValue the ppm value to set, normally 400.
 *
 * This method is used to set the CO2 recalibration value, please use it only on outdoor conditions.
 * Please see the documentation of each sensor for more information.
 */
void Sensors::setCO2RecalibrationFactor(int ppmValue) {
  if (devmode) Serial.printf("-->[SLIB] global calibration to\t: %d\r\n", ppmValue);

  for (int i = 0; i < _active_sensors_count; i++) {
    if (_active_sensors[i]) {
      _active_sensors[i]->calibrate(ppmValue);
    }
  }
}

/**
 * @brief set CO2 altitude offset (m)
 * @param altitude (m).
 *
 * This method is used to compensate the CO2 value with the altitude. Recommended on high altitude.
 */
void Sensors::setCO2AltitudeOffset(float altitude) {
  this->altoffset = altitude;
  this->hpa = hpaCalculation(altitude);  // hPa hectopascal calculation based on altitude

  for (int i = 0; i < _active_sensors_count; i++) {
    if (_active_sensors[i]) {
      _active_sensors[i]->setAltitude(altitude);
      _active_sensors[i]->setSeaLevelPressure(this->sealevel);  // Provide current sealevel too
    }
  }
}

/**
 * @brief set the sea level pressure (hPa)
 * @param hpa (hPa).
 *
 * This method is used to set the sea level pressure for some sensors that need it.
 */
void Sensors::setSeaLevelPressure(float hpa) { sealevel = hpa; }

/// restart and re-init all sensors (not recommended)
void Sensors::restart() {
  _serial->flush();
  init();
  delay(100);
}

/**
 * @brief Get sensor data.
 * @param cb (mandatory) callback function to be called when data is ready.
 */
void Sensors::setOnDataCallBack(voidCbFn cb) { _onDataCb = cb; }

/**
 * @brief Optional callback for get the sensors errors
 * @param cb callback function to be called when any warnning or error happens.
 */
void Sensors::setOnErrorCallBack(errorCbFn cb) { _onErrorCb = cb; }

/**
 * @brief Optional for increase the debug level
 * @param enable true to enable debug mode, false to disable debug mode.
 */
void Sensors::setDebugMode(bool enable) { devmode = enable; }

/// get the sensor status
bool Sensors::isDataReady() { return readAllComplete; }

/// get PM1.0 ug/m3 value
uint16_t Sensors::getPM1() { return pm1; }

/// get PM2.5 ug/m3 value
uint16_t Sensors::getPM25() { return pm25; }

/// get PM4 ug/m3 value
uint16_t Sensors::getPM4() { return pm4; }

/// get PM10 ug/m3 value
uint16_t Sensors::getPM10() { return pm10; }

/// get CO2 ppm value
uint16_t Sensors::getCO2() { return CO2Val; }

/// get humidity % value of CO2 sensor device
float Sensors::getCO2humi() { return CO2humi; }

/// get humidity % value of environment sensor
float Sensors::getHumidity() { return humi; }

/**
 * @brief set the temperature type unit
 * @param tunit celciuse, kelvin or fahrenheit.
 */
void Sensors::setTemperatureUnit(TEMPUNIT tunit) {
  temp_unit = tunit;
  String tunit_symbol;
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      tunit_symbol = getUnitSymbol(TEMP);
      break;
    case TEMPUNIT::KELVIN:
      tunit_symbol = getUnitSymbol(TEMPK);
      break;
    case TEMPUNIT::FAHRENHEIT:
      tunit_symbol = getUnitSymbol(TEMPF);
      break;
    default:
      tunit_symbol = getUnitSymbol(TEMP);
  }
  Serial.printf("-->[SLIB] temperature unit\t: %s\r\n", tunit_symbol.c_str());
}

/// get temperature value from the CO2 sensor device
float Sensors::getCO2temp() {
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      return CO2temp;
    case TEMPUNIT::KELVIN:
      return CO2temp + 273.15;
    case TEMPUNIT::FAHRENHEIT:
      return CO2temp * 1.8 + 32;
  }
  return CO2temp;
}

/// get temperature value from environment sensor
float Sensors::getTemperature() {
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      return temp;
    case TEMPUNIT::KELVIN:
      return temp + 273.15;
    case TEMPUNIT::FAHRENHEIT:
      return temp * 1.8 + 32;
  }
  return temp;
}

/**
 * @brief Temperature unit register (auto)
 * @param isCO2temp temperature unit register for CO2 sensors.
 *
 * This method should register the right unit regarding setTemperatureUnit() method.
 */
void Sensors::tempRegister(bool isCO2temp) {
  switch (temp_unit) {
    case (TEMPUNIT::CELSIUS):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMP);
      else
        unitRegister(UNIT::TEMP);
      break;
    case (TEMPUNIT::KELVIN):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMPK);
      else
        unitRegister(UNIT::TEMPK);
      break;
    case (TEMPUNIT::FAHRENHEIT):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMPF);
      else
        unitRegister(UNIT::TEMPF);
      break;
  }
}

/**
 * @brief Initialize internal temperature offset to be used on startup
 *
 * Positive value for offset to be subtracetd to the temperature.
 * Mush be called before the initialization of the sensors.
 */
void Sensors::initTOffset(float offset) { toffset = offset; }

/**
 * @brief Get sensorlib actual internal temperature offset
 * @return float with the temperature offset.
 * Positive value for offset to be subtracetd to the temperature.
 */
float Sensors::getTOffset() { return toffset; }

/**
 * @brief Set temperature offset for all temperature sensors
 *
 * Positive value for offset to be subtracetd to the temperature.
 */
void Sensors::setTempOffset(float offset) {
  toffset = offset;
  setSCD30TempOffset(toffset * 100);
  setSCD4xTempOffset(toffset);
  setsen5xTempOffset(toffset);
}

/**
 * @brief Get temperature offset for Sensirion sensors (from internal sensor in SCD4x and SCD30)
 * @return float with the temperature offset.
 * Positive value for offset to be subtracetd to the temperature.
 */
float Sensors::getTempOffset() {
  float toffset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD30)) {
    toffset = getSCD30TempOffset();
  }
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    toffset = getSCD4xTempOffset();
  }
  return toffset;
}

/// get Gas resistance value of BMP680 sensor
float Sensors::getGas() { return gas; }

/// get Altitude value in meters
float Sensors::getAltitude() { return alt; }

/// get Pressure value in hPa
float Sensors::getPressure() { return pres; }

/// get NH3 value in ppm
float Sensors::getNH3() { return nh3; }

/// get CO value in ppm
float Sensors::getCO() { return co; }

/// get NO2 value in ppm
float Sensors::getNO2() { return no2; }

/// get O3 value in ppm
float Sensors::getO3() { return o3; }

#ifdef CSL_NOISE_SENSOR_SUPPORTED
float Sensors::getNoise() { return noiseInstant; }

float Sensors::getNoiseAverage() { return noiseAvgValue; }

float Sensors::getNoisePeak() { return noisePeakValue; }

float Sensors::getNoiseMin() { return noiseMinValue; }

float Sensors::getNoiseLegalAverage() { return noiseAvgLegalValue; }

float Sensors::getNoiseLegalMaximum() { return noiseAvgLegalMaxValue; }
#endif

/**
 * @brief UART only: check if the UART sensor is registered
 * @return bool true if the UART sensor is registered, false otherwise.
 */
bool Sensors::isUARTSensorConfigured() { return dev_uart_type >= 0; }

/**
 * @brief UART only: get the UART sensor type. See SENSORS enum. Also getDeviceName()
 * @return SENSORS enum value.
 */
int Sensors::getUARTDeviceTypeSelected() { return dev_uart_type; }

/**
 * @brief Forced to enable I2C sensors only.
 * Recommended to use only if you are using a I2C sensor and improve the performance.
 */
void Sensors::detectI2COnly(bool enable) { i2conly = enable; }

/// returns the CanAirIO Sensorslib version
String Sensors::getLibraryVersion() { return String(CSL_VERSION); }

/// return the current revision code number
int16_t Sensors::getLibraryRevision() { return CSL_REVISION; }

/// get device sensors detected count
uint8_t Sensors::getSensorsRegisteredCount() { return sensors_registered_count; }

/**
 * @brief Read and check the sensors status on initialization
 * @param sensor (mandatory) SENSORS enum value.
 * @return True if the sensor is registered, false otherwise.
 */
bool Sensors::isSensorRegistered(SENSORS sensor) {
  for (unsigned int i = 0; i < SCOUNT; i++) {
    if (sensors_registered[i] == sensor) return true;
  }
  return false;
}

/**
 * @brief get the sensor name
 * @param sensor (mandatory) SENSORS enum value.
 * @return String with the sensor name.
 */
String Sensors::getSensorName(SENSORS sensor) {
  if (sensor < 0 || sensor > SCOUNT) return "";
  return String(sensors_device_names[sensor]);
}

/**
 * @brief get the sensor group type
 * @param sensor (mandatory) SENSORS enum value.
 * @return Sensor group int with the sensor type.
 *
 * if the sensor is not in a group, return 0.
 * if the sensor is in a group, return 1 (PM), 2 (CO2), 3 (ENV).
 */
SensorGroup Sensors::getSensorGroup(SENSORS sensor) {
  return (SensorGroup)sensors_device_types[sensor];
}

/**
 * @brief get the sensor registry for retrieve the sensor names
 * @return pointer to the sensor registry.
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
uint8_t *Sensors::getSensorsRegistered() { return sensors_registered; }

/**
 * @brief get the sensor unit status on the registry
 * @return True if the sensor unit is available, false otherwise.
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
bool Sensors::isUnitRegistered(UNIT unit) {
  if (unit == UNIT::NUNIT) return false;
  for (unsigned int i = 0; i < UCOUNT; i++) {
    if (units_registered[i] == unit) return true;
  }
  return false;
}

/**
 * @brief get the sensor units registry for retrieve the unit name, unit type and symbol. See
 * getNextUnit()
 * @return pointer to the sensor units registry
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
uint8_t *Sensors::getUnitsRegistered() { return units_registered; }

/// get device sensors units detected count
uint8_t Sensors::getUnitsRegisteredCount() { return units_registered_count; }

/**
 * @brief get the sensor unit name
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit name.
 */
String Sensors::getUnitName(UNIT unit) {
  if (unit < 0 || unit > UCOUNT) return "";
  return String(unit_name[unit]);
}

/**
 * @brief get the sensor unit symbol
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit symbol.
 */
String Sensors::getUnitSymbol(UNIT unit) { return String(unit_symbol[unit]); }

/**
 * @brief get the next sensor unit available
 * @return UNIT enum value.
 */
UNIT Sensors::getNextUnit() {
  for (unsigned int i = current_unit; i < UCOUNT; i++) {
    if (units_registered[i] != 0) {
      current_unit = i + 1;
      return (UNIT)units_registered[i];
    }
  }
  current_unit = 0;
  return (UNIT)0;
}

/**
 * @brief reset the sensor units registry.
 *
 * This function is useful to reset the units registry after a sensor unit is removed.
 * but it is **Not necessary** to call this function.
 */
void Sensors::resetUnitsRegister() {
  units_registered_count = 0;
  for (unsigned int i = 0; i < UCOUNT; i++) {
    units_registered[i] = 0;
  }
}
/**
 * @brief reset the sensor registry.
 *
 * This function is useful to reset the sensors registry after a sensor is removed.
 * It should be called before the initialization of the sensors but
 * it is **Not necessary** to call this function.
 */
void Sensors::resetSensorsRegister() {
  sensors_registered_count = 0;
  for (unsigned int i = 0; i < SCOUNT; i++) {
    sensors_registered[i] = 0;
  }
}

/**
 * @brief reset the next sensor unit counter.
 *
 * This function is useful to reset the counter to review the sensor units again.
 * but it is not necessary to call this function.
 */
void Sensors::resetNextUnit() { current_unit = 0; }

/**
 * @brief get the sensor unit value (float)
 * @param unit (mandatory) UNIT enum value.
 * @return float value of the each unit (RAW).
 *
 * Also you can use the specific primitive like getTemperature(),
 * getHumidity(), getGas(), getAltitude(), getPressure()
 */
float Sensors::getUnitValue(UNIT unit) {
  switch (unit) {
    case PM1:
      return pm1;
    case PM25:
      return pm25;
    case PM4:
      return pm4;
    case PM10:
      return pm10;
    case TEMP:
      return temp;
    case TEMPK:
      return temp + 273.15;
    case TEMPF:
      return temp * 1.8 + 32;
    case HUM:
      return humi;
    case CO2:
      return CO2Val;
    case CO2TEMP:
      return CO2temp;
    case CO2TEMPK:
      return CO2temp + 273.15;
    case CO2TEMPF:
      return CO2temp * 1.8 + 32;
    case CO2HUM:
      return CO2humi;
    case PRESS:
      return pres;
    case ALT:
      return alt;
    case GAS:
      return gas;
    case CPM:
      return getGeigerCPM();
    case RAD:
      return getGeigerMicroSievertHour();
    case NH3:
      return nh3;
    case CO:
      return co;
    case NO2:
      return no2;
    case O3:
      return o3;
    case NOISE:
      return noiseInstant;
    case NOISEAVG:
      return noiseAvgValue;
    case NOISEPEAK:
      return noisePeakValue;
    case NOISEMIN:
      return noiseMinValue;
    case NOISEAVGLEGAL:
      return noiseAvgLegalValue;
    case NOISEAVGLEGALMAX:
      return noiseAvgLegalMaxValue;
    default:
      return 0.0;
  }
}

/**
 * @brief print the sensor units names available
 * @param debug optional boolean to set the debug mode flag.
 */
void Sensors::printUnitsRegistered(bool debug) {
  if (!debug) return;
  Serial.printf("-->[SLIB] sensors units count\t: %i (", units_registered_count);
  int i = 0;
  while (units_registered[i++] != 0) {
    Serial.print(unit_name[units_registered[i - 1]]);
    Serial.print(",");
  }
  Serial.println(")");
}

/**
 * @brief print the sensor names detected
 * @param debug optional boolean to set the debug mode flag.
 */
void Sensors::printSensorsRegistered(bool debug) {
  if (!debug) return;
  int i = 0;
  Serial.printf("-->[SLIB] sensors count  \t: %i (", sensors_registered_count);
  if (sensors_registered_count > 0 && sensors_registered[0] == SENSORS::Auto) {
    Serial.printf("%s,", sensors_device_names[sensors_registered[0]]);
    i = 1;
  }
  while (sensors_registered[i++] != 0) {
    Serial.printf("%s,", sensors_device_names[sensors_registered[i - 1]]);
  }
  Serial.println(")");
}

/// Print preview of the current variables detected by the sensors
void Sensors::printValues() {
  if (!devmode) return;
  Serial.print("-->[SLIB] sensors values  \t: ");
  for (unsigned int i = 0; i < UCOUNT; i++) {
    if (units_registered[i] != 0) {
      Serial.print(getUnitName((UNIT)units_registered[i]));
      Serial.print(":");
      Serial.printf("%02.1f ", getUnitValue((UNIT)units_registered[i]));
    }
  }
  Serial.println();
}

/******************************************************************************
 *  S E N S O R   P R I V A T E   M E T H O D S
 ******************************************************************************/

/**
 *  @brief PMS sensor generic read. Supported: Honeywell & Plantower sensors
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmGenericRead() {
  uint8_t buffer[32];
  if (hwSerialRead(buffer, 32) > 0) {
    if (buffer[0] == 66) {
      if (buffer[1] == 77) {
        DEBUG("-->[SLIB] UART PMGENERIC read!\t: :D");
        pm25 = buffer[6] * 256 + buffer[7];
        pm10 = buffer[8] * 256 + buffer[9];

        unitRegister(UNIT::PM25);
        unitRegister(UNIT::PM10);

        if (pm25 > 1000 && pm10 > 1000) {
          onSensorError("[E][SLIB] UART PMGENERIC error\t: out of range pm25 > 1000");
        } else
          return true;
      } else {
        onSensorError("[E][SLIB] UART PMGENERIC error\t: invalid header");
      }
    }
  }
  return false;
}

/**
 *  @brief Panasonic GCJA5 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmGCJA5Read() {
  if (_sensorGCJA5 && _sensorGCJA5->read()) {
    pm1 = _sensorGCJA5->getPM1();
    pm25 = _sensorGCJA5->getPM25();
    pm10 = _sensorGCJA5->getPM10();

    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM10);

    DEBUG("-->[SLIB] UART GCJA5 read\t: done!");
    return true;
  }
  return false;
}

/**
 *  @brief Nova SDS011 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmSDS011Read() {
  if (_sensorSDS011 && _sensorSDS011->read()) {
    pm25 = _sensorSDS011->getPM25();
    pm10 = _sensorSDS011->getPM10();

    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM10);

    DEBUG("-->[SLIB] SDS011 read \t\t: done!");
    return true;
  }
  return false;
}

/**
 *  @brief IKEA Vindriktning particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */

bool Sensors::pm1006Read() {
  if (_sensorPM1006 && _sensorPM1006->read()) {
    pm25 = _sensorPM1006->getPM25();
    unitRegister(UNIT::PM25);
    return true;
  }
  return false;
}

/**
 *  @brief PMS5003T particulate meter, T&H, sensors read.
 *  @return true if header and sensor data is right.
 */

bool Sensors::pm5003TRead() {
  if (!isSensorRegistered(SENSORS::P5003T) || !_sensorPMS5003T) return false;
  if (_sensorPMS5003T->read()) {
    pm1 = _sensorPMS5003T->getPM1();
    pm25 = _sensorPMS5003T->getPM25();
    pm10 = _sensorPMS5003T->getPM10();
    temp = _sensorPMS5003T->getTemperature() - toffset;
    humi = _sensorPMS5003T->getHumidity();
    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM10);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::TEMP);
    return true;
  }
  return false;
}

/**
 * @brief PMSensor Serial read to buffer
 * @param buffer pointer to store data
 * @param length max length to read
 * @return size_t bytes read
 **/
size_t Sensors::hwSerialRead(uint8_t *buffer, size_t length) {
  if (length > 256) length = 256;  // logical cap if needed, though caller owns buffer

  // Use SerialHelper to safely read bytes with timeout
  size_t readCount = SerialHelper::readBytes(*_serial, buffer, length, SENSOR_RETRY);

  if (readCount == 0) {
    DEBUG("-->[SLIB] UART detection msg\t: no data");
    return 0;
  }
  return readCount;
}

/**
 *  @brief Sensirion SPS30 particulate meter sensor read.
 *  @return true if reads succes.
 */
bool Sensors::sps30Read() {
  if (!isSensorRegistered(SENSORS::SSPS30) || !_sensorSPS30) return false;

  if (_sensorSPS30->read()) {
    pm1 = _sensorSPS30->getPM1();
    pm25 = _sensorSPS30->getPM25();
    pm4 = _sensorSPS30->getPM4();
    pm10 = _sensorSPS30->getPM10();

    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM4);
    unitRegister(UNIT::PM10);

    DEBUG("-->[SLIB] SPS30 read \t\t: done!");
    dataReady = true;
    return true;
  }
  return false;
}

bool Sensors::CO2Mhz19Read() {
  if (_sensorMHZ19 && _sensorMHZ19->read()) {
    CO2Val = _sensorMHZ19->getCO2();
    CO2temp = _sensorMHZ19->getTemperature() - toffset;
    if (altoffset != 0) CO2correctionAlt();
    dataReady = true;
    DEBUG("-->[SLIB] MHZ14-9 read  \t: done!");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool Sensors::CO2CM1106Read() {
  if (_sensorCM1106 && _sensorCM1106->read()) {
    CO2Val = _sensorCM1106->getCO2();
    dataReady = true;
    if (altoffset != 0) CO2correctionAlt();
    DEBUG("-->[SLIB] CM1106 read   \t: done!");
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool Sensors::senseAirS8Read() {
  if (_sensorS8 && _sensorS8->read()) {
    CO2Val = _sensorS8->getCO2();
    if (altoffset != 0) CO2correctionAlt();
    dataReady = true;
    DEBUG("-->[SLIB] SENSEAIRS8 read   \t: done!");
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

/**
 * @brief read sensor data. Sensor selected.
 * @return true if data is loaded from sensor.
 */
bool Sensors::pmSensorRead() {
  switch (dev_uart_type) {
    case Auto:
      return pmGenericRead();
      break;

    case SGCJA5:
      return pmGCJA5Read();
      break;

    case SDS011:
      return pmSDS011Read();
      break;

    case IKEAVK:
      return pm1006Read();
      break;

    case SMHZ19:
      return CO2Mhz19Read();
      break;

    case SCM1106:
      return CO2CM1106Read();
      break;

    case SAIRS8:
      return senseAirS8Read();
      break;

    case P5003T:
      return pm5003TRead();
      break;

    default:
      return false;
      break;
  }
}

/******************************************************************************
 *  I 2 C   S E N S O R   R E A D   M E T H O D S
 ******************************************************************************/

void Sensors::am2320Read() {
  if (!isSensorRegistered(SENSORS::SAM232X) || !_sensorAM2320) return;
  if (_sensorAM2320->read()) {
    humi = _sensorAM2320->getHumidity();
    temp = _sensorAM2320->getTemperature() - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AM2320 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::bme280Read() {
  if (!isSensorRegistered(SENSORS::SBME280) || !_sensorBME280) return;

  if (_sensorBME280->read()) {
    humi = _sensorBME280->getHumidity();
    temp = _sensorBME280->getTemperature() - toffset;
    pres = _sensorBME280->getPressure();
    alt = _sensorBME280->getAltitude();
    // approximate altitude calculation if sensor doesn't provide it using sea level pressure
    // The wrapper currently doesn't calculate alt, but returns 0.
    // We should probably help it or move calculation here?
    // Existing code: alt = bme280.readAltitude(sealevel);
    // My wrapper: returns _alt which is 0.
    // I should fix wrapper or calculate here.
    // Adafruit_BME280 has readAltitude(sealevel).
    // I should update wrapper to take sealevel? Or calculate here.
    // Let's calculate here:
    alt = 44330.0 * (1.0 - pow(pres / sealevel, 0.1903));  // Standard formula
    // Or simpler: access wrapper public method if I exposed readAltitude?
    // I didn't expose readAltitude that takes sealevel. I exposed getAltitude() which returns
    // member. I will use formula here or update wrapper. Formula is fine BUT Adafruit lib does it.
    // Let's assume for now I will use formula here or just update wrapper later. Use formula.

    dataReady = true;
    DEBUG("-->[SLIB] BME280 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::ALT);
  }
}

void Sensors::bmp280Read() {
  if (!isSensorRegistered(SENSORS::SBMP280) || !_sensorBMP280) return;

  if (_sensorBMP280->read()) {
    temp = _sensorBMP280->getTemperature() - toffset;
    pres = _sensorBMP280->getPressure();
    alt = _sensorBMP280->getAltitude();
    dataReady = true;
    DEBUG("-->[SLIB] BMP280 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::PRESS);
    unitRegister(UNIT::ALT);
  }
}

void Sensors::bme680Read() {
  if (!isSensorRegistered(SENSORS::SBME680) || !_sensorBME680) return;

  if (_sensorBME680->read()) {
    temp = _sensorBME680->getTemperature() - toffset;
    humi = _sensorBME680->getHumidity();
    pres = _sensorBME680->getPressure();
    gas = _sensorBME680->getGasResistance();
    alt = _sensorBME680->getAltitude();
    dataReady = true;
    DEBUG("-->[SLIB] BME680 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::PRESS);
    unitRegister(UNIT::GAS);
    unitRegister(UNIT::ALT);
  }
}

void Sensors::aht10Read() {
  if (!isSensorRegistered(SENSORS::SAHTXX) || !_sensorAHT10) return;

  if (_sensorAHT10->read()) {
    humi = _sensorAHT10->getHumidity();
    temp = _sensorAHT10->getTemperature() - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AHT10 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::sht31Read() {
  if (!isSensorRegistered(SENSORS::SSHT31) || !_sensorSHT31) return;

  if (_sensorSHT31->read()) {
    humi = _sensorSHT31->getHumidity();
    temp = _sensorSHT31->getTemperature() - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] SHT31 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::CO2scd30Read() {
  if (!isSensorRegistered(SENSORS::SSCD30)) return;

  if (_sensorSCD30 && _sensorSCD30->read()) {
    CO2Val = _sensorSCD30->getCO2();
    CO2humi = _sensorSCD30->getHumidity();
    CO2temp = _sensorSCD30->getTemperature();

    dataReady = true;
    DEBUG("-->[SLIB] SCD30 read\t\t: done! (modular)");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    unitRegister(UNIT::CO2HUM);
  }
}

void Sensors::sgp41Read() {
  if (!isSensorRegistered(SENSORS::SSGP41) || !_sensorSGP41) return;

  if (_sensorSGP41->read()) {
    voci = _sensorSGP41->getVocRaw();
    noxi = _sensorSGP41->getNoxRaw();
    unitRegister(UNIT::VOC);
    unitRegister(UNIT::NOX);
  } else {
    // Error handling or just logging if needed, similar to original but minimal
    DEBUG("-->[SLIB] sgp41 read error");
  }
}

void Sensors::CO2scd4xRead() {
  if (!isSensorRegistered(SENSORS::SSCD4X) || !_sensorSCD4x) return;

  if (_sensorSCD4x->read()) {
    CO2Val = _sensorSCD4x->getCO2();
    CO2temp = _sensorSCD4x->getTemperature();
    CO2humi = _sensorSCD4x->getHumidity();
    dataReady = true;
    DEBUG("-->[SLIB] SCD4x read\t\t: done!");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    unitRegister(UNIT::CO2HUM);
  }
}

void Sensors::sen5xRead() {
  if (!isSensorRegistered(SENSORS::SSEN5X) || !_sensorSEN5x) return;

  if (_sensorSEN5x->read()) {
    pm1 = (uint16_t)_sensorSEN5x->getPm1p0();
    pm25 = (uint16_t)_sensorSEN5x->getPm2p5();
    pm4 = (uint16_t)_sensorSEN5x->getPm4p0();
    pm10 = (uint16_t)_sensorSEN5x->getPm10p0();
    temp = _sensorSEN5x->getTemperature() - toffset;
    humi = _sensorSEN5x->getHumidity();
    voci = _sensorSEN5x->getVocIndex();
    noxi = _sensorSEN5x->getNoxIndex();

    dataReady = true;
    DEBUG("-->[SLIB] SEN5x read\t\t: done!");
    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM4);
    unitRegister(UNIT::PM10);
    unitRegister(UNIT::TEMP);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::VOCI);
    unitRegister(UNIT::NOXI);
  }
}

void Sensors::GCJA5Read() {
  if (dev_uart_type == SENSORS::SGCJA5) return;
  if (!isSensorRegistered(SENSORS::SGCJA5) || !_sensorGCJA5) return;

  if (_sensorGCJA5->read()) {
    pm1 = _sensorGCJA5->getPM1();
    pm25 = _sensorGCJA5->getPM25();
    pm10 = _sensorGCJA5->getPM10();
    dataReady = true;
    DEBUG("-->[SLIB] GCJA5 read\t\t: done!");
    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM10);
  }
}

void Sensors::DFRobotNH3Read() {
  if (!isSensorRegistered(SENSORS::SDFRNH3) || !_sensorDFRNH3) return;
  if (_sensorDFRNH3->read()) {
    nh3 = _sensorDFRNH3->getConcentration();
    unitRegister(UNIT::NH3);
  }
}

void Sensors::DFRobotCORead() {
  if (!isSensorRegistered(SENSORS::SDFRCO) || !_sensorDFRCO) return;
  if (_sensorDFRCO->read()) {
    co = _sensorDFRCO->getConcentration();
    unitRegister(UNIT::CO);
  }
}

void Sensors::DFRobotNO2Read() {
  if (!isSensorRegistered(SENSORS::SDFRNO2) || !_sensorDFRNO2) return;
  if (_sensorDFRNO2->read()) {
    no2 = _sensorDFRNO2->getConcentration();
    unitRegister(UNIT::NO2);
  }
}

void Sensors::DFRobotO3Read() {
  if (!isSensorRegistered(SENSORS::SDFRO3) || !_sensorDFRO3) return;
  if (_sensorDFRO3->read()) {
    o3 = _sensorDFRO3->getConcentration();
    unitRegister(UNIT::O3);
  }
}

#ifdef CSL_NOISE_SENSOR_SUPPORTED
bool Sensors::noiseSensorAutoDetect() {
  if (noiseSensorEnabled) return true;
  if (noiseScanDone) return false;
  noiseScanDone = true;

  noiseSensorInitWire();
  if (noiseWire == nullptr) {
    DEBUG("-->[SLIB] Noise sensor detect:\t", "no i2c");
    return false;
  }

  for (uint8_t addr = MIN_I2C_ADDRESS; addr <= MAX_I2C_ADDRESS; addr++) {
    if (!noiseSensorDevicePresent(*noiseWire, addr)) continue;

    SensorIdentity identity{};
    if (!noiseSensorReadIdentity(*noiseWire, addr, identity)) continue;
    if (identity.sensorType != SENSOR_TYPE_NOISE) continue;

    noiseSensorEnabled = true;
    noiseSensorAddress = addr;
    sensorAnnounce(SENSORS::SNOISE);
    sensorRegister(SENSORS::SNOISE);
    DEBUG("-->[SLIB] Noise sensor detect:\t", "ok");
    return true;
  }

  DEBUG("-->[SLIB] Noise sensor detect:\t", "not found");
  return false;
}

void Sensors::noiseSensorService() {
  if (noiseSensorEnabled || noiseScanDone) return;
  noiseSensorAutoDetect();
}

void Sensors::noiseSensorCollect() {
  if (!noiseSensorEnabled) return;

  if (noiseWire == nullptr || !noiseSensorEnabled) return;

  if (noiseSensorAddress == 0) return;
  if (!noiseSensorDevicePresent(*noiseWire, noiseSensorAddress)) {
    noiseSensorEnabled = false;
    noiseSensorAddress = 0;
    return;
  }
  if (!noiseSensorReadData(*noiseWire, noiseSensorAddress, noiseSensorData)) return;
  noiseInstant = noiseSensorData.noise;
  noiseAvgValue = noiseSensorData.noiseAvg;
  noisePeakValue = noiseSensorData.noisePeak;
  noiseMinValue = noiseSensorData.noiseMin;
  noiseAvgLegalValue = noiseSensorData.noiseAvgLegal;
  noiseAvgLegalMaxValue = noiseSensorData.noiseAvgLegalMax;

  unitRegister(UNIT::NOISE);
  dataReady = true;

  unitRegister(UNIT::NOISEAVG);
  unitRegister(UNIT::NOISEPEAK);
  unitRegister(UNIT::NOISEMIN);
  unitRegister(UNIT::NOISEAVGLEGAL);
  unitRegister(UNIT::NOISEAVGLEGALMAX);
}

bool Sensors::noiseSensorReadIdentity(TwoWire &wire, uint8_t address, SensorIdentity &out) {
  wire.beginTransmission(address);
  wire.write(CMD_PING);
  if (wire.endTransmission() != 0) return false;
  delayMicroseconds(200);

  uint8_t got = wire.requestFrom(address, (uint8_t)sizeof(SensorIdentity));
  if (got != sizeof(SensorIdentity)) return false;

  wire.readBytes(reinterpret_cast<uint8_t *>(&out), sizeof(SensorIdentity));
  return true;
}

bool Sensors::noiseSensorReadData(TwoWire &wire, uint8_t address, SensorData &out) {
  wire.beginTransmission(address);
  wire.write(CMD_GET_DATA);
  if (wire.endTransmission() != 0) return false;
  delayMicroseconds(200);

  uint8_t got = wire.requestFrom(address, (uint8_t)sizeof(SensorData));
  if (got != sizeof(SensorData)) return false;

  uint8_t buffer[sizeof(SensorData)] = {0};
  wire.readBytes(buffer, sizeof(SensorData));
  memcpy(&out, buffer, sizeof(SensorData));
  return true;
}

bool Sensors::noiseSensorDevicePresent(TwoWire &wire, uint8_t address) {
  wire.beginTransmission(address);
  return (wire.endTransmission() == 0);
}

void Sensors::noiseSensorInitWire() {
  if (noiseWireReady) return;

  noiseWire = &Wire;
  noiseWireReady = true;
}
#endif

#ifdef DHT11_ENABLED
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
/**
 * @deprecated Please don't use this sensor anymore
 */
bool Sensors::dhtIsReady(float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > sample_time * (uint32_t)1000) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return (true);
    }
  }
  return (false);
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtInit() {
  sensorAnnounce(SENSORS::SDHTX);
  dhtRead();
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtRead() {
  if (dhtIsReady(&dhttemp, &dhthumi) != true) return;
  temp = dhttemp - toffset;
  humi = dhthumi;
  dataReady = true;
  sensorRegister(SENSORS::SDHTX);
  DEBUG("-->[SLIB] DHTXX read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
}
#endif

void Sensors::onSensorError(const char *msg) {
  DEBUG(msg);
  if (_onErrorCb != nullptr) _onErrorCb(msg);
}

void Sensors::sps30ErrToMess(char *mess, uint8_t r) {
  char buf[80];
  sps30.GetErrDescription(r, buf, 80);
  DEBUG("[E][SLIB] SPS30 error msg\t:", buf);
}

void Sensors::sps30Errorloop(char *mess, uint8_t r) { DEBUG(mess); }

/**
 * Particule meter sensor (PMS) init.
 *
 * Hardware serial init for multiple PM sensors, like
 * Honeywell, Plantower, Panasonic, Sensirion, etc.
 *
 * @param pms_type PMS type, please see DEVICE_TYPE enum.
 * @param pms_rx PMS RX pin.
 * @param pms_tx PMS TX pin.
 **/
bool Sensors::sensorSerialInit(unsigned int pms_type, int pms_rx, int pms_tx) {
  // set UART for autodetection sensors (Honeywell, Plantower)
  if (pms_type == SENSORS::Auto) {
    DEBUG("-->[SLIB] UART detecting type\t: Auto");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }
  // set UART for custom sensors
  else if (pms_type == SENSORS::SGCJA5) {
    DEBUG("-->[SLIB] UART detecting type\t: GCJA5");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SSPS30) {
    DEBUG("-->[SLIB] UART detecting type\t: SSPS30");
    if (!serialInit(pms_type, 115200, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SDS011) {
    DEBUG("-->[SLIB] UART detecting type\t: SDS011");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SMHZ19) {
    DEBUG("-->[SLIB] UART detecting type\t: Mhz19");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SCM1106) {
    DEBUG("-->[SLIB] UART detecting type\t: CM1106");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SAIRS8) {
    DEBUG("-->[SLIB] UART detecting type\t: SENSEAIRS8");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::IKEAVK) {
    DEBUG("-->[SLIB] UART detecting type\t: IKEAVK");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::P5003T) {
    DEBUG("-->[SLIB] UART detecting type\t: PMS5003T");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }

  // starting auto detection loop
  int try_sensor_init = 0;
  while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2) {
  }

  // get device selected..
  if (dev_uart_type >= 0) {
    DEBUG("-->[SLIB] UART sensor detected \t:", getSensorName((SENSORS)dev_uart_type).c_str());
    sensorRegister((SENSORS)dev_uart_type);
    return true;
  }

  return false;
}
/**
 * @brief Generic PM sensor auto detection.
 *
 * In order UART config, this method looking up for
 * special header on Serial stream
 **/
bool Sensors::pmSensorAutoDetect(unsigned int pms_type) {
  delay(1000);  // sync serial

  if (pms_type == SENSORS::SSPS30) {
    if (sps30UARTInit()) {
      dev_uart_type = SENSORS::SSPS30;
      return true;
    }
  }

  if (pms_type == SENSORS::SDS011) {
    if (pmSDS011Read()) {
      dev_uart_type = SENSORS::SDS011;
      return true;
    }
  }

  if (pms_type == SENSORS::IKEAVK) {
    if (PM1006Init()) {
      dev_uart_type = SENSORS::IKEAVK;
      return true;
    }
  }

  if (pms_type == SENSORS::P5003T) {
    if (PM5003TInit()) {
      dev_uart_type = SENSORS::P5003T;
      return true;
    }
  }

  if (pms_type == SENSORS::SMHZ19) {
    if (CO2Mhz19Init()) {
      dev_uart_type = SENSORS::SMHZ19;
      return true;
    }
  }

  if (pms_type == SENSORS::SCM1106) {
    if (CO2CM1106Init()) {
      dev_uart_type = SENSORS::SCM1106;
      return true;
    }
  }

  if (pms_type == SENSORS::SAIRS8) {
    if (senseAirS8Init()) {
      dev_uart_type = SENSORS::SAIRS8;
      return true;
    }
  }

  if (pms_type <= SENSORS::SGCJA5) {
    if (pmGenericRead()) {
      dev_uart_type = SENSORS::Auto;
      return true;
    }
    delay(1000);  // sync serial
    // For GCJA5 UART, we need to try init
    if (_sensorGCJA5) delete _sensorGCJA5;
    _sensorGCJA5 = new SensorGCJA5(*_serial);
    if (_sensorGCJA5->init() && _sensorGCJA5->read()) {
      dev_uart_type = SENSORS::SGCJA5;
      sensorRegister(SENSORS::SGCJA5);
      return true;
    }
    delete _sensorGCJA5;
    _sensorGCJA5 = nullptr;
  }

  return false;
}

bool Sensors::CO2Mhz19Init() {
  if (_sensorMHZ19) delete _sensorMHZ19;
  _sensorMHZ19 = new SensorMHZ19(*_serial);
  if (_sensorMHZ19->init()) {
    registerSensor(_sensorMHZ19);
    return true;
  }
  delete _sensorMHZ19;
  _sensorMHZ19 = nullptr;
  return false;
}

bool Sensors::PM1006Init() {
  if (_sensorPM1006) delete _sensorPM1006;
  _sensorPM1006 = new SensorPM1006(*_serial);
  if (_sensorPM1006->init()) {
    registerSensor(_sensorPM1006);
    return true;
  }
  delete _sensorPM1006;
  _sensorPM1006 = nullptr;
  return false;
}

bool Sensors::PM5003TInit() {
  if (_sensorPMS5003T) delete _sensorPMS5003T;
  _sensorPMS5003T = new SensorPMS5003T(*_serial);
  if (_sensorPMS5003T->init()) {
    registerSensor(_sensorPMS5003T);
    return true;
  }
  delete _sensorPMS5003T;
  _sensorPMS5003T = nullptr;
  return false;
}

bool Sensors::CO2CM1106Init() {
  DEBUG("-->[SLIB] try to enable sensor\t: CM1106..");
  if (_sensorCM1106) delete _sensorCM1106;
  _sensorCM1106 = new SensorCM1106(*_serial);
  if (_sensorCM1106->init()) {
    registerSensor(_sensorCM1106);
    return true;
  }
  delete _sensorCM1106;
  _sensorCM1106 = nullptr;
  return false;
}

bool Sensors::senseAirS8Init() {
  if (_sensorS8) delete _sensorS8;
  _sensorS8 = new SensorSenseAirS8(*_serial);
  if (_sensorS8->init()) {
    registerSensor(_sensorS8);
    return true;
  }
  delete _sensorS8;
  _sensorS8 = nullptr;
  return false;
}

bool Sensors::sps30UARTInit() {
  sensorAnnounce(SENSORS::SSPS30);
  if (_sensorSPS30) delete _sensorSPS30;
  _sensorSPS30 = new SensorSPS30(*_serial);
  if (_sensorSPS30->init()) {
    registerSensor(_sensorSPS30);
    return true;
  }
  delete _sensorSPS30;
  _sensorSPS30 = nullptr;
  return false;
}

bool Sensors::sps30I2CInit() {
  if (dev_uart_type == SENSORS::SSPS30) return false;
  sensorAnnounce(SENSORS::SSPS30);

  if (_sensorSPS30) delete _sensorSPS30;
  _sensorSPS30 = new SensorSPS30();  // I2C
  if (_sensorSPS30->init()) {
    registerSensor(_sensorSPS30);
    return true;
  }
  delete _sensorSPS30;
  _sensorSPS30 = nullptr;
  return false;
}

bool Sensors::sps30tests() {
  // check for SPS30 connection
  if (!sps30.probe()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not probe.", 0);
    return false;
  } else {
    sps30DeviceInfo();
  }
  // reset SPS30 connection
  if (!sps30.reset()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not reset.", 0);
    return false;
  }
  return true;
}

/**
 * @brief : read and display Sensirion device info.
 */
// SPS30 Device info removed

void Sensors::am2320Init() {
  sensorAnnounce(SENSORS::SAM232X);
  if (_sensorAM2320) delete _sensorAM2320;

  _sensorAM2320 = new SensorAM2320();
  if (_sensorAM2320->init()) {
    registerSensor(_sensorAM2320);
    return;
  }

#ifdef Wire1
  delete _sensorAM2320;
  _sensorAM2320 = new SensorAM2320(&Wire1);
  if (_sensorAM2320->init()) {
    registerSensor(_sensorAM2320);
    return;
  }
#endif

  delete _sensorAM2320;
  _sensorAM2320 = nullptr;
}

void Sensors::sht31Init() {
  sensorAnnounce(SENSORS::SSHT31);

  if (_sensorSHT31) delete _sensorSHT31;

  // Try default
  _sensorSHT31 = new SensorSHT31();
  if (_sensorSHT31->init()) {
    registerSensor(_sensorSHT31);
    return;
  }

#ifdef Wire1
  delete _sensorSHT31;
  _sensorSHT31 = new SensorSHT31(0x44, &Wire1);
  if (_sensorSHT31->init()) {
    registerSensor(_sensorSHT31);
    return;
  }
#endif

  delete _sensorSHT31;
  _sensorSHT31 = nullptr;
}

void Sensors::bme280Init() {
  sensorAnnounce(SENSORS::SBME280);

  if (_sensorBME280) delete _sensorBME280;

  // Try 1: Default address (usually 0x77), default Wire
  _sensorBME280 = new SensorBME280(BME280_ADDRESS);
  if (_sensorBME280->init()) {
    registerSensor(_sensorBME280);
    return;
  }
  delete _sensorBME280;

  // Try 2: Only 0x77 failed, try 0x76 (Alternate)
  _sensorBME280 = new SensorBME280(BME280_ADDRESS_ALTERNATE);
  if (_sensorBME280->init()) {
    registerSensor(_sensorBME280);
    return;
  }

#ifdef Wire1
  delete _sensorBME280;
  // Try 3: Default, Wire1
  _sensorBME280 = new SensorBME280(BME280_ADDRESS, &Wire1);
  if (_sensorBME280->init()) {
    registerSensor(_sensorBME280);
    return;
  }
  delete _sensorBME280;

  // Try 4: Alternate, Wire1
  _sensorBME280 = new SensorBME280(BME280_ADDRESS_ALTERNATE, &Wire1);
  if (_sensorBME280->init()) {
    registerSensor(_sensorBME280);
    return;
  }
#endif

  delete _sensorBME280;
  _sensorBME280 = nullptr;
}

/// Environment BMP280 sensor init
void Sensors::bmp280Init() {
  sensorAnnounce(SENSORS::SBMP280);

  if (_sensorBMP280) delete _sensorBMP280;

  // Try default (0x77)
  _sensorBMP280 = new SensorBMP280(BMP280_ADDRESS);
  if (_sensorBMP280->init()) {
    registerSensor(_sensorBMP280);
    return;
  }
  delete _sensorBMP280;

  // Try alternate (0x76)
  _sensorBMP280 = new SensorBMP280(BMP280_ADDRESS_ALT);
  if (_sensorBMP280->init()) {
    registerSensor(_sensorBMP280);
    return;
  }

#ifdef Wire1
  delete _sensorBMP280;
  _sensorBMP280 = new SensorBMP280(BMP280_ADDRESS, &Wire1);
  if (_sensorBMP280->init()) {
    registerSensor(_sensorBMP280);
    return;
  }
  delete _sensorBMP280;

  _sensorBMP280 = new SensorBMP280(BMP280_ADDRESS_ALT, &Wire1);
  if (_sensorBMP280->init()) {
    registerSensor(_sensorBMP280);
    return;
  }
#endif

  delete _sensorBMP280;
  _sensorBMP280 = nullptr;
}

/// Bosch BME680 sensor init
void Sensors::bme680Init() {
  sensorAnnounce(SENSORS::SBME680);

  if (_sensorBME680) delete _sensorBME680;

  _sensorBME680 = new SensorBME680();
  if (_sensorBME680->init()) {
    registerSensor(_sensorBME680);
    return;
  }

#ifdef Wire1
  delete _sensorBME680;
  _sensorBME680 = new SensorBME680(0x77, &Wire1);  // BME680 usually 0x77
  if (_sensorBME680->init()) {
    registerSensor(_sensorBME680);
    return;
  }
#endif

  delete _sensorBME680;
  _sensorBME680 = nullptr;
}

/// AHTXX sensors init
void Sensors::aht10Init() {
  sensorAnnounce(SENSORS::SAHTXX);

  if (_sensorAHT10) delete _sensorAHT10;

  _sensorAHT10 = new SensorAHT10();
  // Note: Previous code handled M5StickC specific pins here.
  // We're simplifying to standard Wire for now.
  if (_sensorAHT10->init()) {
    registerSensor(_sensorAHT10);
    return;
  }

  delete _sensorAHT10;
  _sensorAHT10 = nullptr;
}

/// Sensirion SCD30 CO2/T/H sensor init
void Sensors::CO2scd30Init() {
  sensorAnnounce(SENSORS::SSCD30);

  // Initialize modular sensor wrapper
  if (_sensorSCD30 == nullptr) {
    _sensorSCD30 = new SensorSCD30(&toffset, &altoffset);
  }

  if (!_sensorSCD30->init()) return;

  delay(10);

  registerSensor(_sensorSCD30);

  DEBUG("-->[SLIB] SCD30 Temp offset\t:", String(_sensorSCD30->getTemperatureOffset()).c_str());
  DEBUG("-->[SLIB] SCD30 Altitude offset\t:", String(_sensorSCD30->getAltitudeOffset()).c_str());

  if (_sensorSCD30->getAltitudeOffset() != uint16_t(altoffset)) {
    DEBUG("-->[SLIB] SCD30 altitude offset to\t:", String(altoffset).c_str());
    setSCD30AltitudeOffset(altoffset);
    delay(10);
  }

  if (uint16_t((_sensorSCD30->getTemperatureOffset())) != (uint16_t(toffset * 100))) {
    DEBUG("-->[SLIB] SCD30 Temp offset to\t:", String(toffset).c_str());
    setSCD30TempOffset(toffset);
    delay(10);
  }
}

void Sensors::setSCD30TempOffset(float offset) {
  if (_sensorSCD30) {
    Serial.println("-->[SLIB] SCD30 new temp offset\t: " + String(offset));
    _sensorSCD30->setTemperatureOffset(offset);
  }
}

float Sensors::getSCD30TempOffset() {
  float offset = 0.0;
  if (_sensorSCD30) {
    offset = _sensorSCD30->getTemperatureOffset() / 100.0;
    Serial.println("-->[SLIB] SCD30 get temp offset\t: " + String(offset));
  }
  return offset;
}

void Sensors::setSCD30AltitudeOffset(float offset) {
  if (_sensorSCD30) {
    Serial.println("-->[SLIB] SCD30 new altitude offset\t: " + String(offset));
    _sensorSCD30->setAltitudeOffset(uint16_t(offset));
  }
}

void Sensors::sgp41Init() {
  sensorAnnounce(SENSORS::SSGP41);

  if (_sensorSGP41) delete _sensorSGP41;
  _sensorSGP41 = new SensorSGP41();

  if (_sensorSGP41->init()) {
    registerSensor(_sensorSGP41);
  } else {
    delete _sensorSGP41;
    _sensorSGP41 = nullptr;
  }
}

/// Sensirion SCD4X CO2 sensor init
void Sensors::CO2scd4xInit() {
  sensorAnnounce(SENSORS::SSCD4X);

  if (_sensorSCD4x) delete _sensorSCD4x;
  _sensorSCD4x = new SensorSCD4x();

  if (!_sensorSCD4x->init()) {
    delete _sensorSCD4x;
    _sensorSCD4x = nullptr;
    return;
  }

  registerSensor(_sensorSCD4x);

  float tTemperatureOffset = _sensorSCD4x->getTemperatureOffset();
  DEBUG("-->[SLIB] SCD4x Temp offset\t:", String(tTemperatureOffset).c_str());

  // Logic for setting offsets if needed...
  // Replicating original logic:
  // if (tSensorAltitude != uint16_t(altoffset)) setSCD4xAltitudeOffset(altoffset);

  if (altoffset != 0) {  // Should we check against sensor current?
    // For simplicity calling setter, which does the check internally via read/write delay
    setSCD4xAltitudeOffset(altoffset);
  }

  float offsetDifference = abs((toffset * 100) - (tTemperatureOffset * 100));
  if (offsetDifference > 0.5) {
    Serial.println("-->[SLIB] SCD4x new offset\t: Temp offset to " + String(toffset));
    setSCD4xTempOffset(toffset);
  }
}

/// set SCD4x temperature compensation
void Sensors::setSCD4xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X) && _sensorSCD4x) {
    Serial.println("-->[SLIB] SCD4x new temperature offset\t: " + String(offset));
    _sensorSCD4x->setTemperatureOffset(offset);
  }
}

/// get SCD4x temperature compensation
float Sensors::getSCD4xTempOffset() {
  if (isSensorRegistered(SENSORS::SSCD4X) && _sensorSCD4x) {
    return _sensorSCD4x->getTemperatureOffset();
  }
  return 0.0;
}

/// set SCD4x altitude compensation
void Sensors::setSCD4xAltitudeOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X) && _sensorSCD4x) {
    Serial.println("-->[SLIB] SCD4x new altitude offset\t: " + String(offset));
    _sensorSCD4x->setSensorAltitude((uint16_t)offset);
  }
}

/// Panasonic SEN5X sensor init
void Sensors::sen5xInit() {
  sensorAnnounce(SENSORS::SSEN5X);

  if (_sensorSEN5x) delete _sensorSEN5x;
  _sensorSEN5x = new SensorSEN5x();

  if (!_sensorSEN5x->init()) {
    delete _sensorSEN5x;
    _sensorSEN5x = nullptr;
    return;
  }

  registerSensor(_sensorSEN5x);

  // Temp offset handling
  // _sensorSEN5x does not expose getTemperatureOffsetSimple, only setter in wrapper?
  // Original extracted it.
  // Since we don't have get in wrapper yet, we might skip reading current offset
  // and just enforce if mismatch with global toffset?
  // But we don't know the current offset.
  // For now, trusting that if we set it, it adheres.
  if (toffset != 0) {
    _sensorSEN5x->setTemperatureOffset(toffset);
  }
}

/// set SEN5X temperature compensation
void Sensors::setsen5xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSEN5X) && _sensorSEN5x) {
    Serial.println("-->[SLIB] SEN5x new temperature offset\t: " + String(offset));
    _sensorSEN5x->setTemperatureOffset(offset);
  }
}

/// Panasonic GCJA5 sensor init
void Sensors::GCJA5Init() {
  sensorAnnounce(SENSORS::SGCJA5);
  if (_sensorGCJA5) delete _sensorGCJA5;
  _sensorGCJA5 = new SensorGCJA5();  // I2C
  if (_sensorGCJA5->init()) {
    registerSensor(_sensorGCJA5);
    return;
  }

#ifdef Wire1
  delete _sensorGCJA5;
  _sensorGCJA5 = new SensorGCJA5();  // Constructor doesn't take Wire yet in my impl, needs update
                                     // or handle inside init
  // Actually my SensorGCJA5.cpp init handles default and Wire1
  if (_sensorGCJA5->init()) {
    registerSensor(_sensorGCJA5);
    return;
  }
#endif
  delete _sensorGCJA5;
  _sensorGCJA5 = nullptr;
}

/// DFRobot GAS (CO) sensors init
void Sensors::DFRobotCOInit() {
  sensorAnnounce(SENSORS::SDFRCO);
  if (_sensorDFRCO) delete _sensorDFRCO;
  _sensorDFRCO = new SensorDFRobotGas(0x78, SENSORS::SDFRCO, "DFRobot CO");
  if (_sensorDFRCO->init()) {
    registerSensor(_sensorDFRCO);
  } else {
    delete _sensorDFRCO;
    _sensorDFRCO = nullptr;
  }
}

void Sensors::DFRobotNH3Init() {
  sensorAnnounce(SENSORS::SDFRNH3);
  if (_sensorDFRNH3) delete _sensorDFRNH3;
  _sensorDFRNH3 = new SensorDFRobotGas(0x7A, SENSORS::SDFRNH3, "DFRobot NH3");
  if (_sensorDFRNH3->init()) {
    registerSensor(_sensorDFRNH3);
  } else {
    delete _sensorDFRNH3;
    _sensorDFRNH3 = nullptr;
  }
}

void Sensors::DFRobotNO2Init() {
  sensorAnnounce(SENSORS::SDFRNO2);
  if (_sensorDFRNO2) delete _sensorDFRNO2;
  _sensorDFRNO2 = new SensorDFRobotGas(0x7B, SENSORS::SDFRNO2, "DFRobot NO2");
  if (_sensorDFRNO2->init()) {
    registerSensor(_sensorDFRNO2);
  } else {
    delete _sensorDFRNO2;
    _sensorDFRNO2 = nullptr;
  }
}

void Sensors::DFRobotO3Init() {
  sensorAnnounce(SENSORS::SDFRO3);
  if (_sensorDFRO3) delete _sensorDFRO3;
  _sensorDFRO3 = new SensorDFRobotGas(0x79, SENSORS::SDFRO3, "DFRobot O3");
  if (_sensorDFRO3->init()) {
    registerSensor(_sensorDFRO3);
  } else {
    delete _sensorDFRO3;
    _sensorDFRO3 = nullptr;
  }
}

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation
void Sensors::CO2correctionAlt() {
  DEBUG("-->[SLIB] CO2 altitud original\t:", String(CO2Val).c_str());
  float CO2cor = (0.016 * ((1013.25 - hpa) / 10) * (CO2Val - 400)) +
                 CO2Val;  // Increment of 1.6% for every hpa of difference at sea level
  CO2Val = round(CO2cor);
  DEBUG("-->[SLIB] CO2 compensated\t:", String(CO2Val).c_str());
}

/// hPa hectopascal calculation based on the altitude. See CO2AltitudeOffset setter
float Sensors::hpaCalculation(float altitude) {
  DEBUG("-->[SLIB] CO2 altitude offset\t:", String(altitude).c_str());
  float hpa =
      1012 - 0.118 * altitude +
      0.00000473 * altitude *
          altitude;  // Cuadratic regresion formula obtained PA (hpa) from high above the sea
  DEBUG("-->[SLIB] CO2 pressure (hPa)\t:", String(hpa).c_str());
  return hpa;
}

/// utility to notify on the debug output a possible sensor.
void Sensors::sensorAnnounce(SENSORS sensor) {
  DEBUG("-->[SLIB] attempt enable sensor\t:", getSensorName(sensor).c_str());
}

/**
 * @brief register the sensor type.
 * @param receive SENSORS enum param.
 *
 * Each sensor should be registered and also its units. With that we will able to have
 * dynamic calls of the sensors and its units on the GUI or implementation.
 */
void Sensors::sensorRegister(SENSORS sensor) {
  if (isSensorRegistered(sensor) && sensor != SENSORS::Auto) {
    return;
  }
  Serial.printf("-->[SLIB] sensor registered\t: %s  \t:D\r\n", getSensorName(sensor).c_str());
  sensors_registered[sensors_registered_count++] = sensor;
}

/**
 * @brief register the unit type.
 * @param receive UNIT enum param.
 *
 * Each sensor unit should be registered. For temperature sensors
 * please use tempRegister() method.
 */
void Sensors::unitRegister(UNIT unit) {
  if (isUnitRegistered(unit)) return;
  if (unit == UNIT::NUNIT) return;
  units_registered[units_registered_count++] = unit;
}

/// reset all library variables (generic sensors units)
void Sensors::resetAllVariables() {
  pm1 = 0;
  pm25 = 0;
  pm10 = 0;
  CO2Val = 0;
  CO2humi = 0.0;
  CO2temp = 0.0;
  humi = 0.0;
  temp = 0.0;
  alt = 0.0;
  gas = 0.0;
  pres = 0.0;
  nh3 = 0.0;
  co = 0;
  no2 = 0.0;
  o3 = 0.0;
#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseInstant = 0.0;
  noiseAvgValue = 0.0;
  noisePeakValue = 0.0;
  noiseMinValue = 0.0;
  noiseAvgLegalValue = 0.0;
  noiseAvgLegalMaxValue = 0.0;
#endif
  if (_sensorGeiger != nullptr) _sensorGeiger->clear();
}

// #########################################################################

void Sensors::geigerRead() {
  if (_sensorGeiger && _sensorGeiger->read()) {
    unitRegister(UNIT::CPM);
    unitRegister(UNIT::RAD);
  }
}

void Sensors::enableGeigerSensor(int gpio) {
  sensorAnnounce(SENSORS::SCAJOE);
  if (gpio < 0) {
    if (devmode) Serial.printf("[W][SLIB] undefined Geiger pin\t: %i\r\n", gpio);
    return;
  }
  if (_sensorGeiger) delete _sensorGeiger;
  _sensorGeiger = new SensorGeiger(gpio, devmode);
  if (_sensorGeiger->init()) {
    registerSensor(_sensorGeiger);
  } else {
    delete _sensorGeiger;
    _sensorGeiger = nullptr;
  }
}

/**
 * @brief get Geiger count. Tics in the last 60secs
 * @return CPM
 */
uint32_t Sensors::getGeigerCPM(void) {
  if (_sensorGeiger == nullptr)
    return 0;
  else
    return _sensorGeiger->getCPM();
}

/**
 * @brief get Geiger count in uSv/h units
 * @return CPM * J305 conversion factor
 */
float Sensors::getGeigerMicroSievertHour(void) {
  if (_sensorGeiger == nullptr)
    return 0;
  else
    return _sensorGeiger->getuSvh();
}

// #########################################################################

void Sensors::DEBUG(const char *text, const char *textb) {
  if (devmode) {
    _debugPort.print(text);
    if (textb) {
      _debugPort.print(" ");
      _debugPort.print(textb);
    }
    _debugPort.println();
  }
}

//***********************************************************************************//

void Sensors::startI2C() {
#if defined(M5STICKCPLUS) || defined(M5COREINK)
  Wire.begin(EXT_I2C_SDA, EXT_I2C_SCL);  // M5CoreInk Ext port (default for all sensors)
  enableWire1();
#endif
#ifdef M5ATOM
  Wire.begin();
  enableWire1();
#endif
#ifdef ESP32C3
  Wire.begin(19, 18);
#endif
#ifdef ESP32S2
  Wire.begin(33, 35);
#endif
#ifdef TTGO_T7S3
  Wire.begin(GROVE_SDA, GROVE_SCL);
  enableWire1();
#endif
#ifdef M5AIRQ
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
  enableWire1();
#endif
#ifdef AG_OPENAIR
  Wire.begin(AIRG_SDA, AIRG_SCL);
  delay(1000);
#endif
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_SAM)
  Wire.begin();
#endif
}

void Sensors::enableWire1() {
#ifdef M5STICKCPLUS
  Wire1.flush();
  Wire1.begin(HAT_I2C_SDA, HAT_I2C_SCL);  // M5CoreInk hat pines (header on top)
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(25, 26);  // M5CoreInk hat pines (header on top)
#endif
#ifdef M5ATOM
  Wire1.flush();
  Wire1.begin(26, 32);  // M5CoreInk Ext port (default for all sensors)
#endif
#ifdef M5AIRQ
  Wire1.flush();
  Wire1.begin(GROVE_SDA, GROVE_SCL);
#endif
#ifdef TTGO_T7S3
  Wire1.flush();
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);  // Alternative I2C port
#endif
}

void Sensors::disableWire1() {
#ifdef M5STICKCPLUS
  Wire1.flush();
  Wire1.begin(21, 22);  // Restore AXP192 I2C pins (failed after some time)
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(21, 22);  // M5CoreInk hat pines (header on top)
#endif
}

bool Sensors::serialInit(unsigned int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx) {
  if (devmode)
    Serial.printf("-->[SLIB] UART init with speed\t: %lu TX:%i RX:%i\r\n", speed_baud, pms_tx,
                  pms_rx);
#if ARDUINO_USB_CDC_ON_BOOT  // Serial used for USB CDC
  Serial0.begin(9600, SERIAL_8N1);
  _serial = &Serial0;
  return true;
#endif
  switch (SENSOR_COMMS) {
    case SERIALPORT:
      Serial.begin(speed_baud);
      _serial = &Serial;
      break;
#if defined(ARDUINO_ARCH_ESP32)
    // on a Sparkfun ESP32 Thing the default pins for serial1 are used for acccessing flash memory
    // you have to define different pins upfront in order to use serial1 port.
    case SERIALPORT1:
      DEBUG("-->[SLIB] UART COMM port \t: Serial1");
      if (pms_rx == 0 || pms_tx == 0) {
        DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      _serial = &Serial1;
      break;
    case SERIALPORT2:
#if SOC_UART_NUM > 2
      DEBUG("-->[SLIB] UART COMM port \t: Serial2");
      if (pms_type == SENSORS::SSPS30)
        Serial2.begin(speed_baud);
      else
        Serial2.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      _serial = &Serial2;
#else
      DEBUG("-->[SLIB] Force UART port \t: Serial1");
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx);
      _serial = &Serial1;
#endif
      break;
#endif
    default:

      if (pms_rx == 0 || pms_tx == 0) {
        DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      // In case RX and TX are both pin 8, try Serial1 anyway.
      // A way to force-enable Serial1 on some boards.
      if (pms_rx == 8 && pms_tx == 8) {
        Serial1.begin(speed_baud);
        _serial = &Serial1;
      }

      else {
#if defined(INCLUDE_SOFTWARE_SERIAL)
        DEBUG("-->[SLIB] swSerial init on pin\t:", String(pms_rx).c_str());
        static SoftwareSerial swSerial(pms_rx, pms_tx);
        if (pms_type == SENSORS::SSPS30)
          swSerial.begin(speed_baud);
        else if (pms_type == SENSORS::SGCJA5)
          swSerial.begin(speed_baud, SWSERIAL_8E1, pms_rx, pms_tx, false);
        else
          swSerial.begin(speed_baud, SWSERIAL_8N1, pms_rx, pms_tx, false);
        _serial = &swSerial;
#else
        DEBUG("-->[SLIB] UART SoftwareSerial \t: disable");
        return (false);
#endif  // INCLUDE_SOFTWARE_SERIAL
      }
      break;
  }

  delay(10);
  return true;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
Sensors sensors;
#endif
