#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>


class Motor {
  
  public:
  
    Motor(uint8_t in1Pin, uint8_t in2Pin, uint8_t enablePin);

    void enable();              // enable the whole board
    void disable();             // disable the whole board
    void flip();

    void drive(float speed);    // speed in RPM (range: [-1, 1])
    void brake();               // short brake
    void coast();               // spin freely

  private:
  
    uint8_t _in1;
    uint8_t _in2;
    uint8_t _en;

    int8_t _flip = 1;

    static constexpr int _maxPWM = 255;
};

#endif
