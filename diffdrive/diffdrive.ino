#include "driveunit.hpp"
#include "encoder.hpp"
#include "config.hpp"
#include "motor.hpp"
#include "robot.hpp"


// Motors, Encoders and DriveUnits
Encoder leftEncoder(leftEncoderA);
Motor leftMotor(leftMotorIn1, leftMotorIn2, enable);
DriveUnit leftDriveUnit(leftMotor, leftEncoder, leftTicksPerRev, leftMaxSpeedRPM);

Encoder rightEncoder(rightEncoderA);
Motor rightMotor(rightMotorIn1, rightMotorIn2, enable);
DriveUnit rightDriveUnit(rightMotor, rightEncoder, rightTicksPerRev, rightMaxSpeedRPM);

// Now that we have everything, we can create the Robot
Robot robot(leftDriveUnit, rightDriveUnit, wheelRadius, wheelBase);


void setup() {

  // Serial setup
  Serial.begin(115200);

  // Robot setup
  robot.enable();

  if (PIDenabled){
    leftDriveUnit.setGains(leftKp, leftKi, leftKd, leftKf);
    rightDriveUnit.setGains(rightKp, rightKi, rightKd, rightKf);    
  } else {
    leftDriveUnit.disablePID();
    rightDriveUnit.disablePID();
  }

  if (filterEnabled)
      robot.enableFiltering();
  else
      robot.disableFiltering();

  // Small delay before starting
  delay(3000);

  robot.setRPM(40.0, 40.0);
}

void loop() {

  /*
  // Read linear and angular velocities from UART (format: "linear,angular\n")
  if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      int commaIndex = input.indexOf(',');
      if (commaIndex > 0) {
          float linearVel  = input.substring(0, commaIndex).toFloat();
          float angularVel = input.substring(commaIndex + 1).toFloat();  
          robot.setVelocity(linearVel, angularVel);
      }
  }
  */

  // Read left and right RPM from UART (format: "left_RPM,right_RPM\n")
  if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      int commaIndex = input.indexOf(',');
      if (commaIndex > 0) {
          float leftRPM  = input.substring(0, commaIndex).toFloat();
          float rightRPM = input.substring(commaIndex + 1).toFloat();  
          robot.setRPM(leftRPM, rightRPM);
      }
  }

  robot.update();

}