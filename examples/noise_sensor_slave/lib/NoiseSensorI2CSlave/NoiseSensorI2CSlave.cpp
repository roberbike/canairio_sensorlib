#include "NoiseSensorI2CSlave.h"
#include <cstring>
#if defined(ARDUINO_ARCH_ESP32)
#include "soc/soc_caps.h"
#endif

// Instancia estática para los callbacks
NoiseSensorI2CSlave* NoiseSensorI2CSlave::instance = nullptr;

NoiseSensorI2CSlave::NoiseSensorI2CSlave(const Config& config) 
    : config(config),
      dataReady(false),
      initialized(false),
      adcActive(false),
      lastUpdate(0),
      instanceOwner(false),
      lastCommand(CMD_GET_STATUS),
      pendingReset(false) {
    // Configurar NoiseSensor
    NoiseSensor::Config noiseConfig;
    noiseConfig.adcPin = config.adcPin;
    noiseConfig.logLevel = config.logLevel;
    noiseSensor = NoiseSensor(noiseConfig);
    
    // Inicializar estructura de datos
    memset(&sensorData, 0, sizeof(sensorData));
    
    // Establecer instancia para callbacks estáticos (solo una instancia permitida)
    if (instance == nullptr) {
        instance = this;
        instanceOwner = true;
    }
}

void NoiseSensorI2CSlave::begin() {
    if (!instanceOwner) {
        if (config.logLevel >= NoiseSensor::LOG_ERROR) {
            Serial.println("ERROR: Solo se permite una instancia de NoiseSensorI2CSlave por programa.");
        }
        return;
    }

    // Validar configuración usando el método isValid()
    if (!isValid()) {
        if (config.logLevel >= NoiseSensor::LOG_ERROR) {
            if (config.i2cAddress < MIN_I2C_ADDRESS || config.i2cAddress > MAX_I2C_ADDRESS) {
                Serial.printf("ERROR: Dirección I2C inválida (0x%02X). Debe estar entre 0x%02X y 0x%02X\n", 
                              config.i2cAddress, MIN_I2C_ADDRESS, MAX_I2C_ADDRESS);
            }
            if (!isValidGpioPin(config.sdaPin)) {
                Serial.printf("ERROR: Pin SDA inválido (%d).\n", config.sdaPin);
            }
            if (!isValidGpioPin(config.sclPin)) {
                Serial.printf("ERROR: Pin SCL inválido (%d).\n", config.sclPin);
            }
            if (!isValidAdcPin(config.adcPin)) {
                Serial.printf("ERROR: Pin ADC inválido (%d) para esta plataforma.\n", config.adcPin);
            }
            if (config.sdaPin == config.sclPin) {
                Serial.println("ERROR: SDA y SCL no pueden usar el mismo pin.");
            }
            if (config.updateInterval < MIN_UPDATE_INTERVAL) {
                Serial.printf("ERROR: Intervalo de actualización inválido (%lu ms). Debe ser >= %lu ms\n", 
                              config.updateInterval, MIN_UPDATE_INTERVAL);
            }
        }
        return;
    }
    
    if (config.logLevel >= NoiseSensor::LOG_INFO) {
        Serial.println("=== Inicializando NoiseSensor I2C Slave ===");
        Serial.printf("Dirección I2C: 0x%02X\n", config.i2cAddress);
        Serial.printf("SDA Pin: %d, SCL Pin: %d\n", config.sdaPin, config.sclPin);
        Serial.printf("ADC Pin: %d\n", config.adcPin);
    }
    
    // Configurar tamaño de buffer I2C (debe hacerse antes de begin() para afectar I2C_BUFFER_LENGTH)
    Wire.setBufferSize(64);

    // Configurar I2C como esclavo
    // Firma Arduino-ESP32: begin(uint8_t slaveAddr, int sda, int scl, uint32_t frequency)
    Wire.begin(config.i2cAddress, config.sdaPin, config.sclPin, 100000);
    Wire.onRequest(onRequestStatic);  // Callback cuando el maestro solicita datos
    Wire.onReceive(onReceiveStatic);  // Callback cuando el maestro envía datos
    
    if (config.logLevel >= NoiseSensor::LOG_INFO) {
        Serial.println("I2C esclavo configurado");
    }
    
    // Inicializar sensor de ruido
    noiseSensor.begin();
    
    // Verificar que el ADC está recibiendo señal del micrófono
    adcActive = checkADCSignal();
    
    if (!adcActive) {
        if (config.logLevel >= NoiseSensor::LOG_ERROR) {
            Serial.println("ERROR: No se detecta señal en el ADC. Verifica la conexión del micrófono.");
        }
        // No marcar como inicializado si no hay señal ADC
        return;
    }
    
    // Marcar como inicializado solo si todo fue exitoso
    initialized = true;
    
    if (config.logLevel >= NoiseSensor::LOG_INFO) {
        Serial.println("Sensor de ruido inicializado");
        Serial.println("ADC activo - Micrófono detectado");
        Serial.println("Esperando solicitudes I2C...");
        Serial.println();
    }
}

void NoiseSensorI2CSlave::update() {
    // No hacer nada si no está inicializado
    if (!initialized) {
        return;
    }

    // Procesar acciones pedidas por I2C fuera del callback (contexto no crítico)
    if (pendingReset) {
        pendingReset = false;
        noiseSensor.resetCycle();
    }
    
    // Actualizar sensor de ruido
    noiseSensor.update();
    
    // Verificar periódicamente que el ADC sigue activo (cada 10 segundos, menos frecuente)
    // Solo verificar si ya estaba activo, para evitar falsos positivos
    static unsigned long lastADCCheck = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastADCCheck >= 10000) {  // Cada 10 segundos en lugar de 5
        lastADCCheck = currentMillis;
        bool previousState = adcActive;
        adcActive = checkADCSignal();
        
        // Solo mostrar advertencia si cambió de activo a inactivo
        if (!adcActive && previousState && config.logLevel >= NoiseSensor::LOG_INFO) {
            Serial.println("WARNING: Se perdió la señal del ADC");
        } else if (adcActive && !previousState && config.logLevel >= NoiseSensor::LOG_INFO) {
            Serial.println("INFO: Señal del ADC recuperada");
        }
    }
    
    // Actualizar datos cada intervalo configurado
    if (currentMillis - lastUpdate >= config.updateInterval) {
        lastUpdate = currentMillis;
        
        const auto& measurements = noiseSensor.getMeasurements();
        
        // Actualizar estructura de datos
        sensorData.noise = measurements.noise;
        sensorData.noiseAvg = measurements.noiseAvg;
        sensorData.noisePeak = measurements.noisePeak;
        sensorData.noiseMin = measurements.noiseMin;
        sensorData.noiseAvgLegal = measurements.noiseAvgLegal;
        sensorData.noiseAvgLegalMax = measurements.noiseAvgLegalMax;
        sensorData.lowNoiseLevel = measurements.lowNoiseLevel;
        sensorData.cycles = measurements.cycles;
        
        dataReady = true;
        
        // Mostrar datos por Serial (opcional, para debug)
        if (config.logLevel >= NoiseSensor::LOG_INFO) {
            Serial.println("=== Datos del Sensor ===");
            Serial.printf("Actual: %.2f mV\n", sensorData.noise);
            Serial.printf("Promedio: %.2f mV\n", sensorData.noiseAvg);
            Serial.printf("Pico: %.2f mV\n", sensorData.noisePeak);
            Serial.printf("Mínimo: %.2f mV\n", sensorData.noiseMin);
            Serial.printf("Promedio Legal: %.2f mV\n", sensorData.noiseAvgLegal);
            Serial.printf("Máximo Legal: %.2f mV\n", sensorData.noiseAvgLegalMax);
            Serial.printf("Nivel Base: %d mV\n", sensorData.lowNoiseLevel);
            Serial.printf("Ciclos: %u\n", sensorData.cycles);
            Serial.println();
        }
        
        // Si el ciclo está completo, resetear
        if (noiseSensor.isCycleComplete()) {
            if (config.logLevel >= NoiseSensor::LOG_INFO) {
                Serial.println("Ciclo completado - datos listos para enviar");
            }
            noiseSensor.resetCycle();
        }
    }
}

// Callbacks estáticos que redirigen a la instancia (deben ser mínimos)
void IRAM_ATTR NoiseSensorI2CSlave::onRequestStatic() {
    if (instance != nullptr) {
        instance->onRequest();
    }
}

void IRAM_ATTR NoiseSensorI2CSlave::onReceiveStatic(int numBytes) {
    if (instance != nullptr) {
        instance->onReceive(numBytes);
    }
}

// Implementación de los callbacks
void NoiseSensorI2CSlave::onRequest() {
    // IMPORTANTE (ESP32-C3): onRequest() debe escribir SIEMPRE al menos 1 byte
    // y debe ser lo más corto posible (sin Serial/delay/cálculos).

    // Si no está listo, devolver 1 byte 0x00 para evitar colgar el bus.
    if (!initialized) {
        uint8_t zero = 0x00;
        Wire.write(&zero, 1);
        return;
    }

    uint8_t cmd = lastCommand;

    switch (cmd) {
        case CMD_GET_DATA:
            if (!dataReady) {
                uint8_t zero = 0x00;
                Wire.write(&zero, 1);
                return;
            }
            Wire.write((uint8_t*)&sensorData, sizeof(sensorData));
            return;

        case CMD_GET_AVG:
            Wire.write((uint8_t*)&sensorData.noiseAvg, sizeof(float));
            return;

        case CMD_GET_PEAK:
            Wire.write((uint8_t*)&sensorData.noisePeak, sizeof(float));
            return;

        case CMD_GET_MIN:
            Wire.write((uint8_t*)&sensorData.noiseMin, sizeof(float));
            return;

        case CMD_GET_LEGAL:
            Wire.write((uint8_t*)&sensorData.noiseAvgLegal, sizeof(float));
            return;

        case CMD_GET_LEGAL_MAX:
            Wire.write((uint8_t*)&sensorData.noiseAvgLegalMax, sizeof(float));
            return;

        case CMD_GET_STATUS: {
            uint8_t status = dataReady ? 0x01 : 0x00;
            Wire.write(&status, 1);
            return;
        }

        case CMD_GET_READY: {
            uint8_t ready = isReady() ? 0x01 : 0x00;
            Wire.write(&ready, 1);
            return;
        }

        case CMD_PING: // CMD_PING y CMD_IDENTIFY comparten valor (0x09)
        /* case CMD_IDENTIFY: */ {
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
            return;
        }

        default: {
            // Respuesta mínima por defecto
            uint8_t zero = 0x00;
            Wire.write(&zero, 1);
            return;
        }
    }
}

void NoiseSensorI2CSlave::onReceive(int numBytes) {
    // Callback mínimo (ESP32-C3): NO Serial, NO delay, NO cálculos pesados.
    // Solo leer el comando y guardar estado para que onRequest() responda luego.

    if (numBytes <= 0) {
        return;
    }

    int cmd = Wire.read();
    if (cmd < 0) {
        return;
    }

    lastCommand = static_cast<uint8_t>(cmd);

    // Consumir cualquier byte extra para evitar dejar basura en el buffer
    while (Wire.available()) {
        (void)Wire.read();
    }

    if (lastCommand == CMD_RESET) {
        pendingReset = true;
    }
}

bool NoiseSensorI2CSlave::setConfig(const Config& newConfig) {
    if (initialized) {
        if (config.logLevel >= NoiseSensor::LOG_ERROR) {
            Serial.println("ERROR: No se puede cambiar la configuración después de begin().");
        }
        return false;
    }

    if (!validateConfig(newConfig)) {
        if (config.logLevel >= NoiseSensor::LOG_ERROR) {
            Serial.println("ERROR: Configuración inválida, no se aplicó.");
        }
        return false;
    }

    config = newConfig;

    NoiseSensor::Config noiseConfig;
    noiseConfig.adcPin = config.adcPin;
    noiseConfig.logLevel = config.logLevel;
    noiseSensor = NoiseSensor(noiseConfig);

    return true;
}

// Implementación de métodos públicos
bool NoiseSensorI2CSlave::isReady() const {
    // Está listo si está inicializado Y el ADC está activo
    return initialized && adcActive;
}

// Implementación de método privado para verificar señal ADC
bool NoiseSensorI2CSlave::checkADCSignal() {
    // Mejor estrategia: usar los datos del NoiseSensor en lugar de leer directamente el ADC
    // Si el NoiseSensor está funcionando y generando datos, significa que el ADC está activo
    
    // Verificar que el NoiseSensor tiene datos válidos
    const auto& measurements = noiseSensor.getMeasurements();
    
    // El ADC está activo si:
    // 1. El sensor tiene datos (noise > 0 o hay variación en los valores)
    // 2. Los valores no están todos en cero (lo que indicaría problema)
    // 3. Hay variación en las lecturas (señal dinámica)
    
    // Leer algunas muestras rápidas para verificar variación
    const int numSamples = 5;
    int lastValue = 0;
    bool hasVariation = false;
    bool hasValidValues = false;
    
    for (int i = 0; i < numSamples; i++) {
        int adcValue = analogRead(config.adcPin);
        
        // Verificar que no está en valores extremos que indicarían problema
        // 0 o 4095 constantemente indicarían conexión abierta o cortocircuito
        if (adcValue > 0 && adcValue < 4095) {
            hasValidValues = true;
        }
        
        // Verificar variación (señal dinámica)
        if (i > 0 && abs(adcValue - lastValue) > 5) {
            hasVariation = true;
        }
        
        lastValue = adcValue;
        delay(5);  // Delay más corto para no bloquear tanto
    }
    
    // También verificar que el NoiseSensor tiene datos válidos
    // Si el NoiseSensor está generando datos, el ADC definitivamente está activo
    bool noiseSensorActive = (measurements.noise > 0.0 || 
                              measurements.noiseAvg > 0.0 ||
                              measurements.cycles > 0);
    
    // El ADC está activo si:
    // 1. El NoiseSensor tiene datos (más confiable) O
    // 2. Hay valores válidos en el ADC (no en extremos) Y hay variación
    // Esto prioriza los datos del NoiseSensor sobre la lectura directa del ADC
    if (noiseSensorActive) {
        return true;  // Si el NoiseSensor tiene datos, el ADC está activo
    }
    
    // Si el NoiseSensor no tiene datos aún, verificar el ADC directamente
    // pero ser más permisivo: solo necesita valores válidos (no extremos)
    return hasValidValues;
}

bool NoiseSensorI2CSlave::validateConfig(const Config& cfg) {
    return (cfg.i2cAddress >= MIN_I2C_ADDRESS && cfg.i2cAddress <= MAX_I2C_ADDRESS) &&
           (cfg.updateInterval >= MIN_UPDATE_INTERVAL) &&
           isValidGpioPin(cfg.sdaPin) &&
           isValidGpioPin(cfg.sclPin) &&
           isValidAdcPin(cfg.adcPin) &&
           (cfg.sdaPin != cfg.sclPin);
}

bool NoiseSensorI2CSlave::isValidGpioPin(uint8_t pin) {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    return pin <= 21;
#elif defined(ARDUINO_ARCH_ESP32) && defined(SOC_GPIO_PIN_COUNT)
    return pin < SOC_GPIO_PIN_COUNT;
#else
    return true;
#endif
}

bool NoiseSensorI2CSlave::isValidAdcPin(uint8_t pin) {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    // ESP32-C3 ADC1: GPIO0-4
    return pin <= 4;
#else
    return true;
#endif
}