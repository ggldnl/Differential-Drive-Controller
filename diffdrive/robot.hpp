#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "driveunit.hpp"


class Robot {
public:

    Robot(DriveUnit& left, DriveUnit& right, float wheelRadius, float wheelBase);

    void update();
    void enable();
    void disable();

    // Set the target linear (m/s) and angular (rad/s) velocities
    void setVelocity(float linearVel, float angularVel);

    // Set the target left and right velocities in RPM
    void setRPM(float leftRPM, float rightRPM);

    // Set PID parameters
    void setLeftGains(float Kp, float Ki, float Kd);
    void setRightGains(float Kp, float Ki, float Kd);

    // Get the current wheel velocities in RPM
    float getLeftRPM() const;
    float getRightRPM() const;

    void setUpdateRate(int hz);

    void enableFiltering();
    void disableFiltering();

    void enableDeadband();
    void disableDeadband();

private:

    // DriveUnits
    DriveUnit& leftDriveUnit;
    DriveUnit& rightDriveUnit;

    float targetLeftRPM;
    float targetRightRPM;

    // Robot geometry
    float wheelRadius; // wheel radius in meters
    float wheelBase;   // distance between wheels in meters

    // Smoothing techniques
    bool _filterEnabled = false;
    const float _alpha = 0.2f;  // smoothing factor between 0 (slow) and 1 (no smoothing)
    bool _deadbandEnabled = true;
    const float _delta = 0.5f;  // RPM threshold for ignoring small changes

    // Transform linear & angular velocities to left/right wheel RPM
    void computeWheelRPM(float linearVel, float angularVel);
};

#endif
