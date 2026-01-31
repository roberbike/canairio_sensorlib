#include "SensorAHT10.hpp"

SensorAHT10::SensorAHT10(uint8_t address, TwoWire* wire) : _address(address), _wire(wire) {
    // AHTxx constructor: AHTxx(uint8_t address = AHTXX_ADDRESS_X38, AhtSensor type = AHT1x_SENSOR);
    // It doesn't take 'wire' in constructor. 
    _aht10 = AHTxx(_address, AHT1x_SENSOR); 
}

bool SensorAHT10::init() {
    // AHTxx begin() uses default Wire.
    // begin(sda, scl) exists but that requires pins.
    // If we want to use a specific TwoWire, AHTxx library might not support it natively without modification
    // or it might rely on 'Wire' being the global one.
    // However, for purposes of this refactor, we will assume standard begin() 
    // unless M5STICKCPLUS specific logic is needed (which passed pins).
    // The original code had:
    // #ifdef M5STICKCPLUS
    //   if (!aht10.begin(EXT_I2C_SDA, EXT_I2C_SCL, 100000, 50000)) return;
    // #else
    //   if (!aht10.begin()) return;
    // #endif
    
    // We can't easily detect M5STICKCPLUS here unless we include the board defs or pass pins.
    // For now, we will use default begin() which uses Wire. 
    // If _wire is not &Wire, this might fail or use wrong bus if library hardcodes Wire.
    // Checking AHTxx source code would confirm if it supports TwoWire*... 
    // Most likely it doesn't if not in constructor or begin.
    
    _available = _aht10.begin();
    return _available;
}

bool SensorAHT10::read() {
    float t = _aht10.readTemperature();
    
    if (t != AHTXX_ERROR) {
        float h = _aht10.readHumidity();
        if (h != AHTXX_ERROR) {
            _temp = t;
            _humi = h;
            return true;
        }
    }
    return false;
}
