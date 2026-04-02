#include "SbusReader.h"

SbusReader::SbusReader(HardwareSerial* serial, int8_t rx_pin, int8_t tx_pin)
    : serial_(serial), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

void SbusReader::begin() {
    // SBUS: 100,000 baud, 8-bit data, even parity, 2 stop bits, inverted signal
    serial_->begin(100000, SERIAL_8E2, rx_pin_, tx_pin_, /*invert=*/true);
}

bool SbusReader::read() {
    while (serial_->available()) {
        uint8_t b = (uint8_t)serial_->read();

        // Re-sync on start byte
        if (buf_pos_ == 0 && b != 0x0F) continue;

        buf_[buf_pos_++] = b;
        if (buf_pos_ < 25) continue;
        buf_pos_ = 0;

        // Validate end byte
        if (buf_[24] != 0x00) continue;

        // Unpack 16 channels × 11 bits from bytes 1–22.
        // Each cast to uint16_t before shifting prevents sign-extension bugs.
        data_.ch[0]  = (int16_t)(((uint16_t)buf_[1]        | ((uint16_t)buf_[2]  << 8))                                       & 0x07FF);
        data_.ch[1]  = (int16_t)(((uint16_t)buf_[2]  >> 3  | ((uint16_t)buf_[3]  << 5))                                       & 0x07FF);
        data_.ch[2]  = (int16_t)(((uint16_t)buf_[3]  >> 6  | ((uint16_t)buf_[4]  << 2)  | ((uint16_t)buf_[5]  << 10))        & 0x07FF);
        data_.ch[3]  = (int16_t)(((uint16_t)buf_[5]  >> 1  | ((uint16_t)buf_[6]  << 7))                                       & 0x07FF);
        data_.ch[4]  = (int16_t)(((uint16_t)buf_[6]  >> 4  | ((uint16_t)buf_[7]  << 4))                                       & 0x07FF);
        data_.ch[5]  = (int16_t)(((uint16_t)buf_[7]  >> 7  | ((uint16_t)buf_[8]  << 1)  | ((uint16_t)buf_[9]  << 9))         & 0x07FF);
        data_.ch[6]  = (int16_t)(((uint16_t)buf_[9]  >> 2  | ((uint16_t)buf_[10] << 6))                                       & 0x07FF);
        data_.ch[7]  = (int16_t)(((uint16_t)buf_[10] >> 5  | ((uint16_t)buf_[11] << 3))                                       & 0x07FF);
        data_.ch[8]  = (int16_t)(((uint16_t)buf_[12]        | ((uint16_t)buf_[13] << 8))                                       & 0x07FF);
        data_.ch[9]  = (int16_t)(((uint16_t)buf_[13] >> 3  | ((uint16_t)buf_[14] << 5))                                       & 0x07FF);
        data_.ch[10] = (int16_t)(((uint16_t)buf_[14] >> 6  | ((uint16_t)buf_[15] << 2)  | ((uint16_t)buf_[16] << 10))        & 0x07FF);
        data_.ch[11] = (int16_t)(((uint16_t)buf_[16] >> 1  | ((uint16_t)buf_[17] << 7))                                       & 0x07FF);
        data_.ch[12] = (int16_t)(((uint16_t)buf_[17] >> 4  | ((uint16_t)buf_[18] << 4))                                       & 0x07FF);
        data_.ch[13] = (int16_t)(((uint16_t)buf_[18] >> 7  | ((uint16_t)buf_[19] << 1)  | ((uint16_t)buf_[20] << 9))         & 0x07FF);
        data_.ch[14] = (int16_t)(((uint16_t)buf_[20] >> 2  | ((uint16_t)buf_[21] << 6))                                       & 0x07FF);
        data_.ch[15] = (int16_t)(((uint16_t)buf_[21] >> 5  | ((uint16_t)buf_[22] << 3))                                       & 0x07FF);

        // Flags byte (byte 23)
        data_.lost_frame = (buf_[23] & (1 << 2)) != 0;
        data_.failsafe   = (buf_[23] & (1 << 3)) != 0;

        return true;
    }
    return false;
}
