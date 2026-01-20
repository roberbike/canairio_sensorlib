#ifndef NOISE_SENSOR_SLAVE_H
#define NOISE_SENSOR_SLAVE_H

#include <Arduino.h>
#include <Wire.h>

class NoiseSensorSlave {
 public:
  static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;
  static constexpr uint8_t CMD_GET_DATA = 0x01;
  static constexpr uint8_t CMD_PING = 0x09;
  static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
  static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;

  struct Identity {
    uint8_t sensorType;
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t status;
    uint8_t i2cAddress;
  } __attribute__((packed));

  struct Data {
    float noise;
    float noiseAvg;
    float noisePeak;
    float noiseMin;
    float noiseAvgLegal;
    float noiseAvgLegalMax;
    uint16_t lowNoiseLevel;
    uint16_t reserved;
    uint32_t cycles;
  };

#if __cplusplus >= 201103L
  static_assert(sizeof(Data) == 32, "NoiseSensorSlave::Data size mismatch");
#endif

  void begin(TwoWire *wire);
  bool detect();
  bool isDetected() const;
  uint8_t address() const;
  bool readData(Data &out);
  void reset();

 private:
  bool devicePresent(uint8_t address);
  bool readIdentity(uint8_t address, Identity &out);
  bool readDataInternal(uint8_t address, Data &out);

  TwoWire *wire_ = nullptr;
  uint8_t address_ = 0;
};

#endif
