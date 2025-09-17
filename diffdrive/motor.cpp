#include "motor.hpp"


Motor::Motor(uint8_t in1Pin, uint8_t in2Pin, uint8_t enablePin)
  : _in1(in1Pin), _in2(in2Pin), _en(enablePin) {
  /* 
   * Creates a motor. The input pins (in1 and in2) should be PWM capable.
   * This class abstracts the H-bridge for a single motor. We assume that
   * to assume that the enable pin is shared between all motors
   * on the board, so enabling/disabling it willl affect all of them.
   */
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_en, OUTPUT);
  disable(); // default to off

  // Prevent floating pins to make the motor spin if we enable it without issuing a drive velocity
  drive(0.0f);

  // Match 0–255 PWM scale
  // analogWriteRange(_maxPWM);
}

void Motor::enable() {
  digitalWrite(_en, HIGH);
}

void Motor::disable() {
  digitalWrite(_en, LOW);
}

void Motor::flip() {
  /*
   * Flip the motors default direction.
   */
  _flip *= -1.0f;
}

void Motor::drive(float speed) {
  /*
   * Drive the motor at a given speed (range [-1, 1]):
   * -  1 -> full speed forward
   * - -1 -> full speed backward
   * -  0 -> stopped
   */

  // Clamp input to [-1, 1]
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;

  // Scale to PWM range
  int pwm = int(fabs(speed) * _maxPWM);
  
  if (speed * _flip > 0) {
    analogWrite(_in1, pwm);
    digitalWrite(_in2, LOW);
  } else if (speed * _flip < 0.0f) {
    analogWrite(_in2, pwm);
    digitalWrite(_in1, LOW);
  } else {
    brake(); // stop actively
  }
}

void Motor::brake() {
  /*
  Both motor terminals are connected to HIGH (low impedance staste). 
  This creates a low-resistance path through the H-bridge, 
  rapidly stopping the motor due to back EMF dissipation. 
  Motor "locks" in place.
  */
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
}

void Motor::coast() {
  /*
  Both terminals are set LOW (high impedance state). 
  The motor spins freely and slows down gradually. 
  No braking force is applied.
  */
  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
}
