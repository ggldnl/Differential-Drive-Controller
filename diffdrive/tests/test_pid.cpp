#include "libs/encoder/encoder.hpp"
#include "libs/motor/motor.hpp"
#include "libs/kalman_filter/kalman_filter.hpp"
#include "libs/pid/pid.hpp"

// N20 Motor + High resolution, motor shaft, magnetic encoder -> Count mode
const uint8_t encoder1A = 2;
const uint8_t encoder1B = 7;
const uint8_t motor1In1 = 5;
const uint8_t motor1In2 = 6;
const int encoder1TicksPerRev = 3 * 298;

// TT Motor + low resolution, ouptut shaft, infrared encoder -> Period mode
const uint8_t encoder2A = 3;
const uint8_t encoder2B = 8;
const uint8_t motor2In1 = 10;
const uint8_t motor2In2 = 11;
const int encoder2TicksPerRev = 20;

const uint8_t enable = 4;

// PID stuff
const float kp1 = 0.01;
const float ki1 = 0.0025;
const float kd1 = 0.0;  // We're better off without derivative term eheh

const float kp2 = 0.1;
const float ki2 = 0.1;
const float kd2 = 0.1;

// Define the target RPM each motor should maintain
float targetRPM1 = 30.0f;
float targetRPM2 = 100.0f;

// Define which motor we want to study
bool applyControl1 = true;
bool applyControl2 = false;

// Instantiate components
Motor motor1(motor1In1, motor1In2, enable);
Motor motor2(motor2In1, motor2In2, enable);

Encoder encoder1(encoder1A, Encoder::COUNT_MODE);
Encoder encoder2(encoder2A, Encoder::PERIOD_MODE);

KalmanFilter filter1(0.01f, 1.0f, 1.0f); // q, r, p — process noise, measurement noise, estimate error
KalmanFilter filter2(0.1f, 1.0f, 25.0f);

PID pid1(kp1, ki1, kd1);
PID pid2(kp2, ki2, kd2);

// Support variables
unsigned long lastTime = 0;
const long dtMillis = 100;

void setup() {

  Serial.begin(115200);

  // Enable the motors
  motor1.enable();
  motor2.enable();

  // Flip the right motor so that a command in range [0, 1] makes it spin forward
  motor2.flip();

  // Motors start from 0 RPM (PID will drive them)
  
  delay(3000);
}

// Used by motor+encoder combo 1
float computeCountMode(long deltaTicks, long deltaTime, int ticksPerRev) {
  return (60.0f * 1000000.0f * deltaTicks) / (ticksPerRev * deltaTime);
}

// Used by motor+encoder combo 2
float computePeriodMode(float tickIntervalMicros, int ticksPerRev) {
  return (60.0f * 1000000.0f) / (tickIntervalMicros * ticksPerRev);
}

void loop() {

  unsigned long now = millis();
  if (now - lastTime < dtMillis) return;
  float dt = (now - lastTime) / 1000.0f; // seconds
  lastTime = now;

  // Encoder 1 RPM computation
  static long lastEncoder1Ticks = 0;
  EncoderData encoder1Data = encoder1.getData();
  long deltaTicks = encoder1Data.ticks - lastEncoder1Ticks;
  lastEncoder1Ticks = encoder1Data.ticks;
  
  float encoder1RPM = computeCountMode(deltaTicks, dtMillis * 1000, encoder1TicksPerRev);  
  float filteredEncoder1RPM = filter1.update(encoder1RPM);
 
  // Encoder 2 RPM computation
  EncoderData encoder2Data = encoder2.getData();
  float encoder2RPM = computePeriodMode(encoder2Data.dtMicros, encoder2TicksPerRev);
  float filteredEncoder2RPM = filter2.update(encoder2RPM);

  // Compute PID effects
  double control1 = pid1.update(targetRPM1, filteredEncoder1RPM, dt);
  double control2 = pid2.update(targetRPM2, filteredEncoder2RPM, dt);

  // Apply the controls
  if (applyControl1) motor1.drive(control1);
  if (applyControl2) motor2.drive(control2);

  Serial.print("RPM_1:");
  Serial.print(filteredEncoder1RPM);
  Serial.print(",");
  
  Serial.print("Target_1:");
  Serial.print(targetRPM1);
  Serial.print(",");

  /*
  Serial.print("RPM_2:");
  Serial.print(filteredEncoder2RPM);
  Serial.print(",");
  
  Serial.print("Target_2:");
  Serial.print(targetRPM1);
  Serial.print(",");
  */

  Serial.print("Min:");
  Serial.print(0);
  Serial.print(",");
  
  Serial.print("Max:");
  Serial.print(60);
  Serial.println();

  delay(dtMillis);
}
