#include "shiftregister.hpp"

ShiftRegister::ShiftRegister(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin)
    : _dataPin(dataPin), _clockPin(clockPin), _latchPin(latchPin)
{
    pinMode(_dataPin, OUTPUT);
    pinMode(_clockPin, OUTPUT);
    pinMode(_latchPin, OUTPUT);

    // Set all pins low initially
    digitalWrite(_dataPin, LOW);
    digitalWrite(_clockPin, LOW);
    digitalWrite(_latchPin, LOW);
}

void ShiftRegister::send(uint8_t value) {
    /*
     * Send a byte to the shift register. The byte is sent MSB first. 
     * It will be latched to the outputs after all bits are sent (8 clock cycles).
     */
    
    // Start by pulling latch low
    digitalWrite(_latchPin, LOW);

    // Send each bit (MSB first)
    for (int8_t i = 7; i >= 0; --i) {
        bool bit = (value >> i) & 0x01;
        digitalWrite(_dataPin, bit ? HIGH : LOW);
        pulseClock();
    }

    // Latch the data to outputs
    pulseLatch();
}

void ShiftRegister::pulseClock() {
    /*
     * Internal helper to pulse the clock.
     */
    digitalWrite(_clockPin, HIGH);
    delayMicroseconds(1); // Short delay for stability
    digitalWrite(_clockPin, LOW);
}

void ShiftRegister::pulseLatch() {
    /*
     * Internal helper to pulse the latch.
     * This will transfer the data from the shift register to the outputs.
     */
    digitalWrite(_latchPin, HIGH);
    delayMicroseconds(1); // Short delay
    digitalWrite(_latchPin, LOW);
}
