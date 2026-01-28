#include "NoiseSensor.h"

// Implementacion de metodos de logging
void NoiseSensor::log(LogLevel level, const char* message) const {
  if (shouldLog(level)) {
    Serial.println(message);
  }
}

void NoiseSensor::log(LogLevel level, const char* prefix, unsigned long value) const {
  if (shouldLog(level)) {
    Serial.print(prefix);
    Serial.println(value);
  }
}

void NoiseSensor::log(LogLevel level, const char* prefix, int value) const {
  if (shouldLog(level)) {
    Serial.print(prefix);
    Serial.println(value);
  }
}

void NoiseSensor::log(LogLevel level, const char* prefix, float value) const {
  if (shouldLog(level)) {
    Serial.print(prefix);
    Serial.println(value);
  }
}

void NoiseSensor::log(LogLevel level, const char* prefix, unsigned int value) const {
  if (shouldLog(level)) {
    Serial.print(prefix);
    Serial.println(value);
  }
}

NoiseSensor::NoiseSensor()
    : cycleComplete(false) {
  measurements.cycles = 50;
}

NoiseSensor::NoiseSensor(const Config& config)
    : config(config), cycleComplete(false) {
  measurements.cycles = 50;
}

void NoiseSensor::begin() {
  // Configurar pin ADC (no necesario en ESP32, pero dejamos el comentario)
  // pinMode no se usa para ADC en ESP32

  // Inicializar variables
  tmpIni = millis();
  countStart = millis();
  legalStart = millis();

  measurements.noiseAvg = config.lowNoiseLevel;
  measurements.noiseAvgLegal = config.lowNoiseLevel;
  measurements.noiseAvgPre = config.lowNoiseLevel;
  measurements.noiseAvgLegalMax = config.lowNoiseLevel;
  measurements.noisePeak = 0;
  measurements.noiseMin = 1000;
  measurements.lowNoiseLevel = config.lowNoiseLevel;
  measurements.noise = 0;

  noiseSum = 0;
  noiseSumLegal = 0;
  loops = 0;
  loopsLegal = 0;
  icycles = 1;

  log(LOG_INFO, "NoiseSensor initialized");
}

void NoiseSensor::update() {
  // Leer ruido cada milisegundo
  measurements.noise = readADC_mV();

  // ++++++++++++++  Eliminar outlier ++++++++++++++++
  if (measurements.noise > config.outlierThreshold) {
    if (shouldLog(LOG_WARN)) {
      Serial.print("Outlier removed: ");
      Serial.println(measurements.noise);
    }
  } else {
    // Recalcular el LowNoiseLevel
    if (measurements.noise < measurements.lowNoiseLevel) {
      measurements.lowNoiseLevel = measurements.noise;
    }

    // ++++++++++++++  LAeq basado en muestras cada segundo ++++++++++++++++
    if (millis() - tmpIni > 1000) {
      noiseSum += measurements.noise;
      loops++;

      if (shouldLog(LOG_VERBOSE)) {
        Serial.print("Noise: ");
        Serial.print(measurements.noise);
        Serial.print(" loop: ");
        Serial.print(loops);
        Serial.print(" cycle: ");
        Serial.print(measurements.cycles);
        Serial.print(" loops_legal: ");
        Serial.println(loopsLegal);
      }
      tmpIni = millis();
    }

    // ++++++++++++++  Calculos de ruido legal ++++++++++++++++
    loopsLegal++;
    noiseSumLegal += measurements.noise;
    if (millis() - legalStart > config.legalPeriod) {
      calculateLegalAverage();
      legalStart = millis();
      loopsLegal = 0;
      noiseSumLegal = 0;
    }

    // ++++++++++++++  Deteccion de maximo y minimo ++++++++++++++++
    if (measurements.noise > measurements.noisePeak) {
      measurements.noisePeak = measurements.noise;
      log(LOG_DEBUG, "Noise peak: ", measurements.noisePeak);
    }
    if (measurements.noise < measurements.noiseMin && loops > 5) {
      measurements.noiseMin = measurements.noise;
      log(LOG_DEBUG, "Noise min: ", measurements.noiseMin);
    }
  }

  // Procesar ciclo principal
  if (millis() - countStart > config.dutyCycle) {
    processMainCycle();
  }
}

unsigned int NoiseSensor::readADC_mV() {
  // Leer ADC en milivoltios
  // Compatible con ESP32-C3, ESP32-S2, ESP32-S3

#if defined(ESP32) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3)
  // Usar analogReadMilliVolts() que devuelve directamente en mV
  return analogReadMilliVolts(config.adcPin);
#else
  // Fallback para otros microcontroladores
  int rawValue = analogRead(config.adcPin);
  // Asumir 10-bit ADC y referencia de 3.3V
  return map(rawValue, 0, 1023, 0, 3300);
#endif
}

void NoiseSensor::calculateLegalAverage() {
  log(LOG_DEBUG, " Legal time: ", millis() - legalStart);

  measurements.noiseAvgLegal = int(noiseSumLegal / loopsLegal);
  if (measurements.noiseAvgLegal > measurements.noiseAvgLegalMax) {
    measurements.noiseAvgLegalMax = measurements.noiseAvgLegal;
    log(LOG_INFO, "  Noise legal current maximum: ", measurements.noiseAvgLegalMax);
  }

  if (shouldLog(LOG_DEBUG)) {
    Serial.print("   (Legal) noise_avg_legal: ");
    Serial.print(measurements.noiseAvgLegal);
    Serial.print(" noise_avg_legal_max: ");
    Serial.print(measurements.noiseAvgLegalMax);
    Serial.print(" samples: ");
    Serial.println(loopsLegal);
  }
}

void NoiseSensor::processMainCycle() {
  log(LOG_INFO, " DutyCycle time: ", millis() - countStart);
  countStart = millis();

  // Calculos de ruido
  measurements.noiseAvg = int(noiseSum / loops);

  if (shouldLog(LOG_INFO)) {
    Serial.print("  Noise average: ");
    Serial.println(measurements.noiseAvg);
    Serial.print("  Noise peak: ");
    Serial.println(measurements.noisePeak);
    Serial.print("  Noise min: ");
    Serial.println(measurements.noiseMin);
    Serial.print("  Samples: ");
    Serial.println(loops);
  }

  if (shouldLog(LOG_DEBUG)) {
    Serial.print("  Noise sum: ");
    Serial.println(noiseSum);
  }

  // Control de ciclos
  if (measurements.cycles > 99) {
    icycles = -1;
  } else if (measurements.cycles < 1) {
    // Reiniciar el LowNoiseLevel para evitar niveles bajos como 0
    measurements.lowNoiseLevel = measurements.noiseMin;
    icycles = +1;
  }

  measurements.cycles += icycles;
  noiseSum = 0;
  loops = 0;
  measurements.noiseAvgPre = measurements.noiseAvg;
  measurements.noiseAvgLegal = measurements.noiseAvgLegalMax;

  // Comprobar si es momento de enviar datos
  cycleComplete = true;
}

void NoiseSensor::resetCycle() {
  cycleComplete = false;
}
