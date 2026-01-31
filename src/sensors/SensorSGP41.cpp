#include "SensorSGP41.hpp"
#include <Arduino.h>

SensorSGP41::SensorSGP41(TwoWire* wire) : _wire(wire) {
}

bool SensorSGP41::init() {
    _sgp41.begin(*_wire);
    
    uint16_t testResult;
    uint16_t error = _sgp41.executeSelfTest(testResult);
    
    if (error) return false;
    if (testResult != 0xD400) return false;

    _available = true;
    return true;
}

bool SensorSGP41::read() {
    uint16_t error;
    uint16_t defaultRh = 0x8000;
    uint16_t defaultT = 0x6666;
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;

    if (_conditioning_s > 0) {
        // During NOx conditioning (10s) SRAW NOx will remain 0
        // We pass local srawVoc by reference, but srawVoc is output? 
        // executeConditioning(uint16_t rh, uint16_t t, uint16_t& srawVoc)
        // Check library signature. 
        // existing: error = sgp41.executeConditioning(defaultRh, defaultT, voc);
        error = _sgp41.executeConditioning(defaultRh, defaultT, srawVoc);
        _conditioning_s--;
        _vocRaw = srawVoc;
        _noxRaw = 0; 
        if (error) return false;
        return true; 
    } else {
        // Read Measurement
        // existing: error = sgp41.measureRawSignals(defaultRh, defaultT, voc, nox);
        error = _sgp41.measureRawSignals(defaultRh, defaultT, srawVoc, srawNox);
        if (error) return false;
        
        _vocRaw = srawVoc;
        _noxRaw = srawNox;
        return true;
    }
}
