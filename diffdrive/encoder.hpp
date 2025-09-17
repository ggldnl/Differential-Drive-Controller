#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>


class Encoder {

  public:

    Encoder(uint8_t pin);

    long getTicks();
    void reset();
    void update();

  private:

    // Encoder stuff
    volatile long _ticks;
    uint8_t _pin;

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
