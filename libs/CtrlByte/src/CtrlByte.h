#ifndef _CTRL_BYTE
#define _CTRL_BYTE

#include <Arduino.h>

class CtrlByte {
public:
    // Konstruktor: initialisiert alle Bits auf 0
    CtrlByte();

    // Bit setzen (auf 1)
    void setBit(uint8_t bitPosition);

    // Bit löschen (auf 0)
    void clearBit(uint8_t bitPosition);

    // Bit umschalten (flip)
    void toggleBit(uint8_t bitPosition);

    // Bit prüfen (true = gesetzt)
    bool readBit(uint8_t bitPosition) const;

    // Ganzes Byte setzen/lesen
    void setByte(uint8_t value);
    uint8_t getByte() const;

    // zwei Bits linken damit und abhängig von einander setzen
    void linkBits(uint8_t bitPos1, uint8_t bitPos2, bool inputBit1, bool inputBit2);

private:
    uint8_t _byte;
};

#endif