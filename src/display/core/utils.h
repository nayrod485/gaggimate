#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <memory>
#include <sstream>

template <typename T, typename... Args> std::unique_ptr<T> make_unique(Args &&...args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

inline uint8_t randomByte() { return static_cast<uint8_t>(esp_random() & 0xFF); }

String generateUUIDv4() {
    std::array<uint8_t, 16> bytes;
    for (auto &b : bytes)
        b = randomByte();
    bytes[6] = (bytes[6] & 0x0F) | 0x40;
    bytes[8] = (bytes[8] & 0x3F) | 0x80;
    char uuidStr[37];
    snprintf(uuidStr, sizeof(uuidStr), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", bytes[0], bytes[1],
             bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10], bytes[11], bytes[12],
             bytes[13], bytes[14], bytes[15]);
    return String(uuidStr);
}

#endif // UTILS_H
