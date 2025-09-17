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

/*
// I used this setup to estimate the ticks per revolution, not knowing which type of motor I had
const float leftMaxSpeedRPM = 67.0;               // Used opencv to determine the actual RPM for both motors
const float leftATicksPerRev = 995;               // Used an Arduino sketch to count the average number of ticks in 1 sec
const float leftRevPerSec = leftMaxSpeedRPM / 60;
const float leftTicksPerRev = leftATicksPerRev / leftRevPerSec;

const float rightMaxSpeedRPM = 65.5;
const float rightATicksPerRev = 942;
const float rightRevPerSec = rightMaxSpeedRPM / 60;
const float rightTicksPerRev = rightATicksPerRev / rightRevPerSec;
*/

// I used this setup with new motors for which I knew exactly the gear ratio
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
const bool PIDenabled = true;

const float leftKp = 0.2;
const float leftKi = 0.8;
const float leftKd = 0.01;
const float leftKf = 0.0;

const float rightKp = 0.2;
const float rightKi = 0.8;
const float rightKd = 0.01;
const float rightKf = 0.0;

// Filtering
const bool filterEnabled = false;
const bool deadbandEnabled = true;

#endif