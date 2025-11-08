#include "libs/drive_unit/drive_unit.hpp"
#include "libs/encoder/encoder.hpp"
#include "libs/motor/motor.hpp"
#include "libs/robot/robot.hpp"
#include "config.hpp"


// Motors, Encoders and DriveUnits
Encoder leftEncoder(leftEncoderA);
Motor leftMotor(leftMotorIn1, leftMotorIn2, enable);
DriveUnit leftDriveUnit(leftMotor, leftEncoder, leftTicksPerRev, leftMaxSpeedRPM, leftKf);

Encoder rightEncoder(rightEncoderA);
Motor rightMotor(rightMotorIn1, rightMotorIn2, enable);
DriveUnit rightDriveUnit(rightMotor, rightEncoder, rightTicksPerRev, rightMaxSpeedRPM, rightKf);

// Now that we have everything, we can create the Robot
Robot robot(leftDriveUnit, rightDriveUnit, wheelBase, wheelRadius);


void setup() {

  // Serial setup
  Serial.begin(115200);

  // Robot setup
  robot.enable();

  // PID setup
  if (PIDEnabled){
    
    // Eanble feedback control loops
    robot.enablePID();

    // Set gains
    robot.leftDriveUnit.setFeedforward(leftKf);
    robot.rightDriveUnit.setFeedforward(rightKf);
    robot.leftDriveUnit.setPIDGains(leftKp, leftKi, leftKd);
    robot.rightDriveUnit.setPIDGains(rightKp, rightKi, rightKd);

    // Set update rates
    robot.setControlLoopPeriodHz(controlLoopPeriodHz);

  } else {

    // Disable control loops
    robot.disablePID();
  }

  // Kalman setup
  if (KalmanEnabled){
    
    // Eanble 1D Kalman filter
    robot.enableKalman();

    // Set gains
    robot.leftDriveUnit.setKalmanGains(leftQ, leftR, leftP);
    robot.rightDriveUnit.setKalmanGains(rightQ, rightR, rightP);
  
  } else {

    // Disable Kalman filter
    robot.disableKalman();
  }

  // Small delay before starting
  delay(3000);

  // TODO remove
  robot.setRPMs(40.0, 40.0);
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
          robot.setRPMs(leftRPM, rightRPM);
      }
  }

  robot.update();

}