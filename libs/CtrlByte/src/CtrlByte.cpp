#include "CtrlByte.h"

CtrlByte::CtrlByte() {
    _byte = 0;
}

void CtrlByte::setBit(uint8_t bitPosition) {
    if (bitPosition < 8) {
        _byte |= (1 << bitPosition);
    }
}

void CtrlByte::clearBit(uint8_t bitPosition) {
    if (bitPosition < 8) {
        _byte &= ~(1 << bitPosition);
    }
}

void CtrlByte::toggleBit(uint8_t bitPosition) {
    if (bitPosition < 8) {
        _byte ^= (1 << bitPosition);
    }
}

bool CtrlByte::readBit(uint8_t bitPosition) const {
    if (bitPosition < 8) {
        return (_byte & (1 << bitPosition)) != 0;
    }
    return false;
}

void CtrlByte::setByte(uint8_t value) {
    _byte = value;
}

uint8_t CtrlByte::getByte() const {
    return _byte;
}

void CtrlByte::linkBits(uint8_t bitPos1, uint8_t bitPos2, bool inputBit1, bool inputBit2) {
    if (bitPos1 < 8 and bitPos2 < 8) { // safety
        if (inputBit1) { // signal on Button 1
            if (CtrlByte::readBit(bitPos2)) { // other signal is true 
                CtrlByte::clearBit(bitPos2); // reset Bit -> stop Movement
                return;
            } else { // other signal is false
                CtrlByte::toggleBit(bitPos1); //set signal -> start Movement
                return;
            }
        }

        if (inputBit2) { // signal on Button 2
            if (CtrlByte::readBit(bitPos1)) { // other signal is true 
                CtrlByte::clearBit(bitPos1); // reset Bit -> stop Movement
                return;
            } else { // other signal is false
                CtrlByte::toggleBit(bitPos2); //set signal -> start Movement
                return;
            }
        }
    } 
}