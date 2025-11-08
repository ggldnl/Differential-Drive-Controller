#ifndef SHIFTREGISTER_HPP
#define SHIFTREGISTER_HPP

#include <Arduino.h>


class ShiftRegister {
public:

    ShiftRegister(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin);

    void send(uint8_t value);

private:

    uint8_t _dataPin;
    uint8_t _clockPin;
    uint8_t _latchPin;

    void pulseClock();
    void pulseLatch();
};

#endif // SHIFTREGISTER_HPP
