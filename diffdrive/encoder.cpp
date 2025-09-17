#include "encoder.hpp"

// Create space for all the possible instasnces
Encoder* Encoder::_instances[2] = {nullptr, nullptr};
int Encoder::_count = 0;

Encoder::Encoder(uint8_t pin) : _pin(pin), _ticks(0) {
    pinMode(_pin, INPUT_PULLUP);

    int id = _count;
    _instances[id] = this;
    _count++;

    // Attach the ISR automatically
    if (id == 0) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::isr0, RISING);
    } else if (id == 1) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::isr1, RISING);
    }
}

void Encoder::update() {
    _ticks++;
}

long Encoder::getTicks() {
    return _ticks;
}

void Encoder::reset() {
    _ticks = 0;
}

void Encoder::isr0() {
    if (_instances[0]) _instances[0]->update();
}

void Encoder::isr1() {
    if (_instances[1]) _instances[1]->update();
}
