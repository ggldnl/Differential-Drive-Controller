#ifndef CONFIG_HPP
#define CONFIG_HPP

// Pinout
const uint8_t leftEncoderA = 2;
const uint8_t leftEncoderB = 7;
const uint8_t leftMotorIn1 = 5;
const uint8_t leftMotorIn2 = 6;

const uint8_t rightEncoderA = 3;
const uint8_t rightEncoderB = 8;
const uint8_t rightMotorIn1 = 10;
const uint8_t rightMotorIn2 = 11;

const uint8_t enable = 4;

// Motor parameters

// Gear ratio
const float leftMaxSpeedRPM = 60.0f;
const float leftEncoderPPR = 3.0f;
const float leftGearRatio = 298.0f;
const float leftTicksPerRev = leftEncoderPPR * leftGearRatio;

const float rightMaxSpeedRPM = 60.0f;
const float rightEncoderPPR = 3.0f;
const float rightGearRatio = 298.0f;
const float rightTicksPerRev = rightEncoderPPR * rightGearRatio;

// Robot geometry
const float wheelBase = 0.08;       // 80 mm
const float wheelRadius = 0.0125;   // 25 / 2 mm

// PID
const bool PIDEnabled = true;

const float leftKp = 0.01;
const float leftKi = 0.0025;
const float leftKd = 0.0;
const float leftKf = 0.0;   // Feedforward disabled

const float rightKp = 0.01;
const float rightKi = 0.0025;
const float rightKd = 0.0;
const float rightKf = 0.0;  // Feedforward disabled

// Kalman
const bool KalmanEnabled = true;

const float leftQ = 0.01;
const float leftR = 1.0;
const float leftP = 1.0;

const float rightQ = 0.01;
const float rightR = 1.0;
const float rightP = 1.0;

// Update frequencies
const unsigned long controlLoopPeriodHz = 50;

#endif