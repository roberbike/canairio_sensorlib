#if !defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ARCH_ESP32C3) && !defined(ARDUINO_ARCH_ESP32S2) && !defined(ARDUINO_ARCH_ESP32S3) && !defined(ESP32)

#include "NoiseSensorI2CSlave.h"

#include <cstring>

NoiseSensorI2CSlave* NoiseSensorI2CSlave::instance = nullptr;

NoiseSensorI2CSlave::NoiseSensorI2CSlave(const Config& config)
    : config(config),
      dataReady(false),
      initialized(false),
      adcActive(false),
      lastUpdate(0),
      instanceOwner(false),
      lastCommand(CMD_GET_STATUS),
      pendingReset(false),
      cycleComplete(false) {
  memset(&sensorData, 0, sizeof(sensorData));
  sensorData.cycles = 50;

  if (instance == nullptr) {
    instance = this;
    instanceOwner = true;
  }
}

void NoiseSensorI2CSlave::begin() {
  if (!instanceOwner) return;

  if (!isValid()) return;

#if defined(ARDUINO_ARCH_ESP32)
  Wire.setBufferSize(64);
  if (!Wire.begin(config.i2cAddress, config.sdaPin, config.sclPin, 100000)) {
    return;
  }
#elif defined(ARDUINO_ARCH_ESP8266)
  Wire.begin(config.sdaPin, config.sclPin, config.i2cAddress);
  Wire.setClock(100000);
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
  Wire.begin(config.i2cAddress);
  Wire.setClock(100000);
#else
  Wire.begin(config.i2cAddress);
  Wire.setClock(100000);
#endif

  Wire.onRequest(onRequestStatic);
  Wire.onReceive(onReceiveStatic);

  // Initialize measurement state
  tmpIni = millis();
  countStart = millis();
  legalStart = millis();

  sensorData.noiseAvg = config.lowNoiseLevel;
  sensorData.noiseAvgLegal = config.lowNoiseLevel;
  sensorData.noiseAvgLegalMax = config.lowNoiseLevel;
  sensorData.noiseMin = 1000;
  sensorData.lowNoiseLevel = config.lowNoiseLevel;

  noiseSum = 0;
  noiseSumLegal = 0;
  loops = 0;
  loopsLegal = 0;
  icycles = 1;

  adcActive = checkADCSignal();
  if (adcActive) {
    initialized = true;
  }
}

void NoiseSensorI2CSlave::update() {
  if (!initialized) return;

  if (pendingReset) {
    pendingReset = false;
    resetCycle();
  }

  // noise measurement logic
  unsigned int currentNoise = readADC_mV();
  sensorData.noise = (float)currentNoise;

  if (currentNoise <= (unsigned int)config.outlierThreshold) {
    if (currentNoise < sensorData.lowNoiseLevel) {
      sensorData.lowNoiseLevel = currentNoise;
    }

    if (millis() - tmpIni > 1000) {
      noiseSum += currentNoise;
      loops++;
      tmpIni = millis();
    }

    loopsLegal++;
    noiseSumLegal += currentNoise;
    if (millis() - legalStart > config.legalPeriod) {
      calculateLegalAverage();
      legalStart = millis();
      loopsLegal = 0;
      noiseSumLegal = 0;
    }

    if (sensorData.noise > sensorData.noisePeak) {
      sensorData.noisePeak = sensorData.noise;
    }
    if (sensorData.noise < sensorData.noiseMin && loops > 5) {
      sensorData.noiseMin = sensorData.noise;
    }
  }

  if (millis() - countStart > config.dutyCycle) {
    processMainCycle();
  }

  // Periodic ADC check
  static unsigned long lastUpdateCheck = 0;
  if (millis() - lastUpdateCheck >= config.updateInterval) {
    lastUpdateCheck = millis();
    dataReady = true;
    if (cycleComplete) {
      resetCycle();
    }
  }
}

unsigned int NoiseSensorI2CSlave::readADC_mV() {
#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
  return analogReadMilliVolts(config.adcPin);
#else
  int rawValue = analogRead(config.adcPin);
  return map(rawValue, 0, 1023, 0, 3300);
#endif
}

void NoiseSensorI2CSlave::calculateLegalAverage() {
  if (loopsLegal > 0) {
    sensorData.noiseAvgLegal = (float)(noiseSumLegal / loopsLegal);
    if (sensorData.noiseAvgLegal > sensorData.noiseAvgLegalMax) {
      sensorData.noiseAvgLegalMax = sensorData.noiseAvgLegal;
    }
  }
}

void NoiseSensorI2CSlave::processMainCycle() {
  countStart = millis();
  if (loops > 0) {
    sensorData.noiseAvg = (float)(noiseSum / loops);
  }

  if (sensorData.cycles > 99) {
    icycles = -1;
  } else if (sensorData.cycles < 1) {
    sensorData.lowNoiseLevel = (uint16_t)sensorData.noiseMin;
    icycles = +1;
  }

  sensorData.cycles += (uint32_t)icycles;
  noiseSum = 0;
  loops = 0;
  cycleComplete = true;
}

void NoiseSensorI2CSlave::resetCycle() { cycleComplete = false; }

bool NoiseSensorI2CSlave::checkADCSignal() {
  const int numSamples = 5;
  for (int i = 0; i < numSamples; i++) {
    int adcValue = analogRead(config.adcPin);
    if (adcValue > 0 && adcValue < 4095) return true;
    delay(5);
  }
  return false;
}

void NoiseSensorI2CSlave::log(LogLevel level, const char* message) const {
  if (shouldLog(level)) Serial.println(message);
}

void NoiseSensorI2CSlave::log(LogLevel level, const char* prefix, float value) const {
  if (shouldLog(level)) {
    Serial.print(prefix);
    Serial.println(value);
  }
}

void NoiseSensorI2CSlave::onRequest() {
  if (!initialized) {
    uint8_t zero = 0x00;
    Wire.write(&zero, 1);
    return;
  }

  switch (lastCommand) {
    case CMD_GET_DATA:
      Wire.write((uint8_t*)&sensorData, sizeof(sensorData));
      break;
    case CMD_GET_AVG:
      Wire.write((uint8_t*)&sensorData.noiseAvg, sizeof(float));
      break;
    case CMD_GET_PEAK:
      Wire.write((uint8_t*)&sensorData.noisePeak, sizeof(float));
      break;
    case CMD_GET_MIN:
      Wire.write((uint8_t*)&sensorData.noiseMin, sizeof(float));
      break;
    case CMD_GET_LEGAL:
      Wire.write((uint8_t*)&sensorData.noiseAvgLegal, sizeof(float));
      break;
    case CMD_GET_LEGAL_MAX:
      Wire.write((uint8_t*)&sensorData.noiseAvgLegalMax, sizeof(float));
      break;
    case CMD_GET_STATUS: {
      uint8_t status = dataReady ? 0x01 : 0x00;
      Wire.write(&status, 1);
      break;
    }
    case CMD_GET_READY: {
      uint8_t ready = isReady() ? 0x01 : 0x00;
      Wire.write(&ready, 1);
      break;
    }
    case CMD_IDENTIFY: {
      SensorIdentity identity;
      identity.sensorType = SENSOR_TYPE_NOISE;
      identity.versionMajor = VERSION_MAJOR;
      identity.versionMinor = VERSION_MINOR;
      identity.status = 0;
      if (initialized) identity.status |= 0x01;
      if (adcActive) identity.status |= 0x02;
      if (dataReady) identity.status |= 0x04;
      identity.i2cAddress = config.i2cAddress;
      Wire.write((uint8_t*)&identity, sizeof(identity));
      break;
    }
    default: {
      uint8_t zero = 0x00;
      Wire.write(&zero, 1);
      break;
    }
  }
}

void NoiseSensorI2CSlave::onReceive(int numBytes) {
  if (numBytes > 0) {
    lastCommand = (uint8_t)Wire.read();
    while (Wire.available()) Wire.read();
    if (lastCommand == CMD_RESET) pendingReset = true;
  }
}

void IRAM_ATTR NoiseSensorI2CSlave::onRequestStatic() {
  if (instance) instance->onRequest();
}
void IRAM_ATTR NoiseSensorI2CSlave::onReceiveStatic(int numBytes) {
  if (instance) instance->onReceive(numBytes);
}

bool NoiseSensorI2CSlave::validateConfig(const Config& cfg) {
  if (cfg.i2cAddress < MIN_I2C_ADDRESS || cfg.i2cAddress > MAX_I2C_ADDRESS) return false;
  if (cfg.sdaPin == cfg.sclPin) return false;
  if (cfg.updateInterval < MIN_UPDATE_INTERVAL) return false;
  if (!isValidGpioPin(cfg.sdaPin) || !isValidGpioPin(cfg.sclPin)) return false;
  if (!isValidAdcPin(cfg.adcPin)) return false;
  return true;
}

bool NoiseSensorI2CSlave::isValidGpioPin(uint8_t pin) {
  if (pin == 255) return false;
#if defined(ARDUINO_ARCH_ESP8266)
  return pin <= 16;
#else
  return true;
#endif
}

bool NoiseSensorI2CSlave::isValidAdcPin(uint8_t pin) {
  if (pin == 255) return false;
#if defined(ARDUINO_ARCH_ESP8266)
#ifdef A0
  return pin == static_cast<uint8_t>(A0) || pin == 0;
#else
  return pin == 0;
#endif
#else
  return true;
#endif
}

#endif  // !ESP32 family
