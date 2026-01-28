#ifndef NOISE_SENSOR_H
#define NOISE_SENSOR_H

#include <Arduino.h>

class NoiseSensor {
 public:
  // Niveles de logging
  enum LogLevel {
    LOG_NONE = 0,   // Sin logs
    LOG_ERROR = 1,  // Solo errores
    LOG_WARN = 2,   // Warnings y errores
    LOG_INFO = 3,   // Informacion importante
    LOG_DEBUG = 4,  // Debug detallado
    LOG_VERBOSE = 5 // Todo (muy verboso)
  };

  // Configuracion
  struct Config {
    int adcPin = 4;                       // GPIO para ADC
    unsigned long dutyCycle = 120000;     // Periodo de transmision (ms)
    unsigned long legalPeriod = 5000;     // Periodo de medicion legal (ms)
    int lowNoiseLevel = 36;               // Nivel base de ruido bajo
    int noiseDiffSleep = 0;               // Diferencias para modo sleep
    unsigned long sleep4NoNoise = 300000; // Tiempo de sleep cuando hay poco ruido
    bool indoor = false;                  // Si true, nunca entra en sleep
    int outlierThreshold = 4095;          // Umbral para descartar valores anomalos
    LogLevel logLevel = LOG_NONE;         // Nivel de logging (por defecto: INFO)
  };

  // Resultados de mediciones
  struct Measurements {
    unsigned int noise;      // Valor actual de ruido
    float noiseAvg;          // Promedio de ruido
    float noiseAvgPre;       // Promedio previo
    unsigned int noisePeak;  // Valor pico de ruido
    unsigned int noiseMin;   // Valor minimo de ruido
    int lowNoiseLevel;       // Nivel base dinamico
    float noiseAvgLegal;     // Promedio legal actual
    float noiseAvgLegalMax;  // Maximo promedio legal
    unsigned int cycles;     // Contador de ciclos
  };

  // Constructor
  NoiseSensor();
  NoiseSensor(const Config& config);

  // Inicializacion
  void begin();

  // Actualizar mediciones (llamar desde loop)
  void update();

  // Obtener mediciones actuales
  const Measurements& getMeasurements() const { return measurements; }

  // Verificar si es momento de enviar datos (ciclo completado)
  bool isCycleComplete() const { return cycleComplete; }

  // Preparar para nuevo ciclo (llamar despues de procesar datos)
  void resetCycle();

 private:
  Config config;
  Measurements measurements;

  // Control de tiempo
  unsigned long tmpIni;
  unsigned long countStart;
  unsigned long legalStart;

  // Contadores y sumas
  int loops;
  int loopsLegal;
  unsigned long noiseSum;
  unsigned long noiseSumLegal;

  // Control de ciclos
  float icycles;
  bool cycleComplete;

  // Leer ADC en milivoltios (compatible con ESP32-C3/S2/S3)
  unsigned int readADC_mV();

  // Calcular promedio legal
  void calculateLegalAverage();

  // Procesar ciclo principal
  void processMainCycle();

  // Metodos de logging optimizados
  void log(LogLevel level, const char* message) const;
  void log(LogLevel level, const char* prefix, unsigned long value) const;
  void log(LogLevel level, const char* prefix, int value) const;
  void log(LogLevel level, const char* prefix, float value) const;
  void log(LogLevel level, const char* prefix, unsigned int value) const;
  inline bool shouldLog(LogLevel level) const { return level <= config.logLevel; }
};

#endif  // NOISE_SENSOR_H
