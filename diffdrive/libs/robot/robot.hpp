#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "libs/drive_unit/drive_unit.hpp"

#include <Arduino.h>


class Robot {

  public:

    Robot(
        DriveUnit& leftDriveUnit, 
        DriveUnit& rightDriveUnit, 
        float wheelBase,
        float leftWheelRadius, 
        float rightWheelRadius=0
    );

    // Enable/disable the whole robot
    void enable();
    void disable();

    // Enable/disable PID
    void enablePID();
    void disablePID();

    // Enable/disable 1D Kalman filter
    void enableKalman();
    void disableKalman();

    // Control inputs
    void setVelocity(float linVel, float angVel);
    void setLeftRightVelocity(float leftAngVel, float rightAngVel);
    void setRPMs(float leftRPM, float rightRPM);

    // Control loop update
    void setControlLoopPeriodHz(int hz);
    void update();  // To be called regularly in the loop

    // DriveUnits
    DriveUnit& leftDriveUnit;
    DriveUnit& rightDriveUnit;

  private:

    // Robot geometry
    float leftWheelRadius;
    float rightWheelRadius;
    float wheelBase;

    // Convert linear and angular velocities to left and right angular velocities
    void linAngToLeftRight(float v, float w, float leftWheelRadius, float rightWheelRadius, float wheelBase, float& omegaLeft, float& omegaRight) const;

    // Convert left and right RPMs to left and right angular velocities
    void RPMToLeftRight(float leftRPM, float rightRPM, float leftWheelRadius, float rightWheelRadius, float& vLeft, float& vRight) const;

};

#endif
