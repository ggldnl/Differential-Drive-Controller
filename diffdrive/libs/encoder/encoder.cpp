#include "encoder.hpp"


// Create space for all the possible instances (by default only 2)
Encoder* Encoder::_instances[2] = {nullptr, nullptr};
int Encoder::_count = 0;

Encoder::Encoder(uint8_t pin, Mode mode): 
  _pin(pin), 
  _ticks(0),
  _lastTickMicros(0),
  _tickIntervalMicros(0),
  _mode(mode) 
{
  pinMode(_pin, INPUT_PULLUP);

  // Assign the instance
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

EncoderData Encoder::getData() {

  EncoderData data;

  // Disable interrupts
  noInterrupts();

  data.ticks = _ticks;
  data.dtMicros = _tickIntervalMicros;

  // Re-enable interrupts
  interrupts();

  return data;
}

void Encoder::update() {

    unsigned long now = micros();

    // Always increment tick counter
    _ticks++;

    // In PERIOD_MODE, measure time between pulses
    if (_mode == PERIOD_MODE) {
        if (_lastTickMicros > 0) {
            _tickIntervalMicros = now - _lastTickMicros;
        }
        _lastTickMicros = now;
    }
    // In COUNT_MODE, _lastTickMicros = 0
}

void Encoder::reset() {

  // Disable interrupts
  noInterrupts();

  _ticks = 0;
  _tickIntervalMicros = 0;
  _lastTickMicros = 0;

  // Re-enable interrupts
  interrupts();
}

void Encoder::setMode(Mode mode) {
  _mode = mode;
  reset();
}

Encoder::Mode Encoder::getMode() {
  return _mode;
}

void Encoder::isr0() {
  if (_instances[0]) {
    _instances[0]->update();
  }
}

void Encoder::isr1() {
  if (_instances[1]) {
    _instances[1]->update();
  }
}
