#include "Noise_Sensor_Slave.h"

void NoiseSensorSlave::begin(TwoWire *wire) {
  wire_ = wire;
}

bool NoiseSensorSlave::detect() {
  if (wire_ == nullptr) return false;

  for (uint8_t addr = MIN_I2C_ADDRESS; addr <= MAX_I2C_ADDRESS; addr++) {
    if (!devicePresent(addr)) continue;
    Identity identity{};
    if (!readIdentity(addr, identity)) continue;
    if (identity.sensorType != SENSOR_TYPE_NOISE) continue;
    address_ = addr;
    return true;
  }

  address_ = 0;
  return false;
}

bool NoiseSensorSlave::isDetected() const { return address_ != 0; }

uint8_t NoiseSensorSlave::address() const { return address_; }

bool NoiseSensorSlave::readData(Data &out) {
  if (wire_ == nullptr || address_ == 0) return false;
  if (!devicePresent(address_)) {
    address_ = 0;
    return false;
  }
  return readDataInternal(address_, out);
}

void NoiseSensorSlave::reset() { address_ = 0; }

bool NoiseSensorSlave::devicePresent(uint8_t address) {
  wire_->beginTransmission(address);
  return (wire_->endTransmission() == 0);
}

bool NoiseSensorSlave::readIdentity(uint8_t address, Identity &out) {
  wire_->beginTransmission(address);
  wire_->write(CMD_PING);
  if (wire_->endTransmission() != 0) return false;

  uint8_t got = wire_->requestFrom(address, (uint8_t)sizeof(Identity));
  if (got != sizeof(Identity)) return false;

  wire_->readBytes(reinterpret_cast<uint8_t *>(&out), sizeof(Identity));
  return true;
}

bool NoiseSensorSlave::readDataInternal(uint8_t address, Data &out) {
  wire_->beginTransmission(address);
  wire_->write(CMD_GET_DATA);
  if (wire_->endTransmission() != 0) return false;

  uint8_t got = wire_->requestFrom(address, (uint8_t)sizeof(Data));
  if (got != sizeof(Data)) return false;

  uint8_t buffer[sizeof(Data)] = {0};
  wire_->readBytes(buffer, sizeof(Data));
  memcpy(&out, buffer, sizeof(Data));
  return true;
}
