#include "libs/motor/motor.hpp"


// N20 Motor
const uint8_t motor1In1 = 5;
const uint8_t motor1In2 = 6;

// TT Motor
const uint8_t motor2In1 = 10;
const uint8_t motor2In2 = 11;

const uint8_t enablePin = 4;

// Instantiate motors
Motor motor1(motor1In1, motor1In2, enablePin);
Motor motor2(motor2In1, motor2In2, enablePin);

void setup() {

  Serial.begin(115200);
  Serial.println("Motor speed sweep test starting...");

  // Flip the right motor so that a command in range [0, 1] makes it spin forward
  motor2.flip();

  // Enable the motors
  motor1.enable();
  motor2.enable();

}

void loop() {

  // Sweep forward
  Serial.println("Sweeping forward...");
  for (float speed = 0.0; speed <= 1.0; speed += 0.1) {
    motor1.drive(speed);
    motor2.drive(speed);
    Serial.print("Speed: ");
    Serial.println(speed, 2);
    delay(500);
  }

  // Hold for a moment
  delay(1000);

  // Sweep down to zero
  Serial.println("Slowing to stop...");
  for (float speed = 1.0; speed >= 0.0; speed -= 0.1) {
    motor1.drive(speed);
    motor2.drive(speed);
    Serial.print("Speed: ");
    Serial.println(speed, 2);
    delay(500);
  }

  // Short pause
  motor1.brake();
  motor2.brake();
  delay(1000);

  // Sweep reverse
  Serial.println("Sweeping reverse...");
  for (float speed = 0.0; speed >= -1.0; speed -= 0.1) {
    motor1.drive(speed);
    motor2.drive(speed);
    Serial.print("Speed: ");
    Serial.println(speed, 2);
    delay(500);
  }

  // Back to zero
  Serial.println("Returning to stop...");
  for (float speed = -1.0; speed <= 0.0; speed += 0.1) {
    motor1.drive(speed);
    motor2.drive(speed);
    Serial.print("Speed: ");
    Serial.println(speed, 2);
    delay(500);
  }

  // Coast at the end of the cycle
  Serial.println("Coasting...");
  motor1.coast();
  motor2.coast();
  delay(2000);

  // Repeat
}
