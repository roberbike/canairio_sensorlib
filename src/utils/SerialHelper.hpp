#ifndef SERIAL_HELPER_HPP
#define SERIAL_HELPER_HPP

#include <Arduino.h>
#include <vector>

class SerialHelper {
public:
    /**
     * @brief Reads from a stream into a buffer safely without using String class
     * @param stream The source stream (Serial, SoftwareSerial, etc.)
     * @param buffer The destination buffer
     * @param maxLen Maximum bytes to read
     * @param timeoutMs Timeout in milliseconds
     * @return Number of bytes read
     */
    static size_t readBytes(Stream& stream, uint8_t* buffer, size_t maxLen, uint32_t timeoutMs = 100) {
        size_t count = 0;
        uint32_t start = millis();
        
        while (millis() - start < timeoutMs && count < maxLen) {
            if (stream.available()) {
                buffer[count++] = stream.read();
            } else {
                delay(1); // Short yield
            }
        }
        return count;
    }
    
    /**
     * @brief Check for a specific header sequence
     */
    static bool checkHeader(const uint8_t* buffer, size_t len, const uint8_t* header, size_t headerLen) {
        if (len < headerLen) return false;
        for (size_t i = 0; i < headerLen; i++) {
            if (buffer[i] != header[i]) return false;
        }
        return true;
    }
};

#endif // SERIAL_HELPER_HPP
