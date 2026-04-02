#pragma once
#include <Arduino.h>

// Minimal SBUS reader for ESP32.
// SBUS protocol: 100000 baud, 8E2, inverted signal, 25-byte frames.
// Channel values: 172 (min) – 992 (center) – 1811 (max).

class SbusReader {
public:
    struct Data {
        int16_t ch[16];  // 16 channel values
        bool lost_frame;
        bool failsafe;
    };

    SbusReader(HardwareSerial* serial, int8_t rx_pin, int8_t tx_pin);

    // Call once in setup(). Configures the UART for SBUS.
    void begin();

    // Call every loop. Returns true when a complete, valid frame is received.
    bool read();

    const Data& data() const { return data_; }

private:
    HardwareSerial* serial_;
    int8_t rx_pin_;
    int8_t tx_pin_;
    uint8_t buf_[25];
    uint8_t buf_pos_ = 0;
    Data data_ = {};
};
