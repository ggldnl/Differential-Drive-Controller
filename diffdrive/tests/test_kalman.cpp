#include "libs/encoder/encoder.hpp"
#include "libs/motor/motor.hpp"
#include "libs/kalman_filter/kalman_filter.hpp"


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


// Instantiate components
Motor motor1(motor1In1, motor1In2, enable);
Motor motor2(motor2In1, motor2In2, enable);

Encoder encoder1(encoder1A, Encoder::COUNT_MODE);
Encoder encoder2(encoder2A, Encoder::PERIOD_MODE);

KalmanFilter filter1(0.01f, 1.0f, 1.0f); // q, r, p — process noise, measurement noise, estimate error
KalmanFilter filter2(0.1f, 1.0f, 25.0f);
//                   ^      ^     ^
//                   |      |     Initial error covariance (allow more initial uncertainty)
//                   |      Measurement noise (fast motor = more measurement variance)
//                   Process noise (prevent covariance collapse)

void setup() {

  Serial.begin(115200);

  // Enable the motors
  motor1.enable();
  motor2.enable();

  // Flip the right motor so that a command in range [0, 1] makes it spin forward
  motor2.flip();

  // Make them spin at 50% max speed
  motor1.drive(0.5);
  motor2.drive(0.5);

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

  long dtMillis = 100;

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

  Serial.print("Encoder_1_(N20):");
  Serial.print(encoder1RPM);
  Serial.print(",");

  Serial.print("Encoder_1_filtered_(N20):");
  Serial.print(filteredEncoder1RPM);
  Serial.print(",");

  Serial.print("Encoder_2_(TT):");
  Serial.print(encoder2RPM);
  Serial.print(",");

  Serial.print("Encoder_2_filtered_(TT):");
  Serial.print(filteredEncoder2RPM);
  Serial.println();

  delay(dtMillis);
}