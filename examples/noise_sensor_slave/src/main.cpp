/**
 * ESP32 (MASTER) - Escaneo y lectura I2C de sensor de ruido
 *
 * Requisitos del usuario:
 * - SDA = GPIO 8
 * - SCL = GPIO 9
 * - Buscar la dirección I2C del sensor y decir si está conectado
 *
 * Nota importante (para el problema "muestra valores con el esclavo apagado"):
 * - Este código SOLO imprime datos si el esclavo respondió y llegaron TODOS los bytes esperados.
 * - Si no hay respuesta, no usa datos viejos ni memoria sin inicializar.
 */

#include <Arduino.h>
#include <Wire.h>

// Configuración I2C (master)
static constexpr uint8_t SDA_PIN = 8;
static constexpr uint8_t SCL_PIN = 9;
static constexpr uint32_t I2C_FREQ_HZ = 100000; // 100kHz

static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;

// Protocolo (debe coincidir con el esclavo)
static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;

enum I2CCommand : uint8_t {
    CMD_GET_DATA = 0x01,
    CMD_PING = 0x09,
};

struct SensorIdentity {
    uint8_t sensorType;
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t status;
    uint8_t i2cAddress;
} __attribute__((packed));

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

static uint8_t sensorAddress = 0;
static bool sensorDetected = false;

static bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}

static bool readIdentity(uint8_t address, SensorIdentity& out) {
    // CMD_PING
    Wire.beginTransmission(address);
    Wire.write(CMD_PING);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    const uint8_t got = Wire.requestFrom(address, (uint8_t)sizeof(SensorIdentity));
    if (got != sizeof(SensorIdentity)) {
        return false;
    }

    Wire.readBytes((uint8_t*)&out, sizeof(SensorIdentity));
    return (out.sensorType == SENSOR_TYPE_NOISE);
}

static bool readSensorData(uint8_t address, SensorData& out) {
    Wire.beginTransmission(address);
    Wire.write(CMD_GET_DATA);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    const uint8_t got = Wire.requestFrom(address, (uint8_t)sizeof(SensorData));
    if (got != sizeof(SensorData)) {
        return false;
    }

    Wire.readBytes((uint8_t*)&out, sizeof(SensorData));
    return true;
}

static void scanAndReport() {
    sensorDetected = false;
    sensorAddress = 0;

    Serial.println("=== Buscando sensor de ruido (I2C) ===");
    Serial.printf("SDA=%u SCL=%u Freq=%luHz\n", SDA_PIN, SCL_PIN, (unsigned long)I2C_FREQ_HZ);
    Serial.printf("Rango: 0x%02X - 0x%02X\n", MIN_I2C_ADDRESS, MAX_I2C_ADDRESS);

    for (uint8_t addr = MIN_I2C_ADDRESS; addr <= MAX_I2C_ADDRESS; addr++) {
        if (!i2cDevicePresent(addr)) {
            continue;
        }

        SensorIdentity id{};
        if (readIdentity(addr, id)) {
            sensorDetected = true;
            sensorAddress = addr;

            Serial.println("═══════════════════════════════════");
            Serial.println("✓ SENSOR CONECTADO");
            Serial.printf("  Dirección I2C: 0x%02X (%u)\n", sensorAddress, sensorAddress);
            Serial.printf("  Versión: %u.%u\n", id.versionMajor, id.versionMinor);
            Serial.printf("  Status: 0x%02X\n", id.status);
            Serial.println("═══════════════════════════════════");
            Serial.println();
            return;
        }
    }

    Serial.println("═══════════════════════════════════");
    Serial.println("✗ SENSOR NO CONECTADO");
    Serial.println("═══════════════════════════════════");
    Serial.println("Verifica:");
    Serial.println("  - Pull-ups (4.7k) en SDA y SCL a 3V3");
    Serial.println("  - GND común");
    Serial.println("  - Alimentación del sensor");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);

    Serial.println();
    Serial.println("=== ESP32 MASTER I2C - Sensor de Ruido ===");
    Serial.println();

    scanAndReport();
}

void loop() {
    static unsigned long lastActionMs = 0;
    const unsigned long now = millis();

    // Reintentar búsqueda cada 5s si no está detectado
    if (!sensorDetected) {
        if (now - lastActionMs >= 5000) {
            lastActionMs = now;
            scanAndReport();
        }
        delay(50);
        return;
    }

    // Si está detectado, verificar y leer datos cada 30s
    if (now - lastActionMs >= 30000) {
        lastActionMs = now;

        if (!i2cDevicePresent(sensorAddress)) {
            Serial.println("⚠ Sensor desconectado (no responde por I2C)");
            sensorDetected = false;
            sensorAddress = 0;
            Serial.println();
            return;
        }

        SensorData data{}; // evita basura si falla la lectura
        if (!readSensorData(sensorAddress, data)) {
            Serial.println("⚠ Lectura I2C falló (no se imprimen datos)");
            return;
        }

        Serial.printf("Ruido=%.2f  Prom=%.2f  Pico=%.2f  Min=%.2f  AvgLgl=%.2f AvgLglMax=%.2fCiclos=%lu\n",
                      data.noise, data.noiseAvg, data.noisePeak, data.noiseMin, data.noiseAvgLegal, data.noiseAvgLegalMax, (unsigned long)data.cycles);
    }

    delay(10);
}

