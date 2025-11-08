#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>


struct EncoderData {
    long ticks;               // Total ticks since reset
    unsigned long dtMicros;   // Time between last two ticks (0 if not updated)
};

class Encoder {

  public:

    /* 
     * We can use the encoder in two modes:
     * - COUNT_MODE (default): counts pulses over a time window.
     * - PERIOD_MODE: measures time between pulses. This is handy when the ticks per revolution are low.
     */
    enum Mode {
        COUNT_MODE,   // count pulses in time window
        PERIOD_MODE   // measure time between pulses
    };

    Encoder(uint8_t pin, Mode mode = COUNT_MODE);

    void reset();
    void update();

    EncoderData getData();

    void setMode(Mode mode);
    Mode getMode();

  private:

    // Encoder pin
    uint8_t _pin;

    // Mode
    Mode _mode;

    // Count mode
    long _lastTickMicros;
    long _tickIntervalMicros;

    // Period mode
    long _ticks;

    /*
     * We need to update the encoder inside the ISR. To do so, we need to 
     * create the Encoder first, like this:
     *
     * Encoder encoder(pin);
     *
     * void encoderISR() {
     *    encoder.update();
     * }
     *
     * attachInterrupt(digitalPinToInterrupt(pin), encoderISR, RISING);
     *
     * This means we should do these things separately. I want to be able 
     * to create an Encoder and forget about it so the ISR stuff should be 
     * handled automatically. To do so, we need to use this pattern and 
     * define a priori a certain number of encoders that we will be using 
     * and a static ISR for each of them, such that each ISR can reference 
     * the respective Encoder. Unfortunately, we need to hardcode some stuff.
     */
    static Encoder* _instances[2]; // support 2 encoders
    static int _count;

    // 2 static ISR handlers
    static void isr0();
    static void isr1();
};

#endif
