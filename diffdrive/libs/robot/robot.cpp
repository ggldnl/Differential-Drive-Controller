#include "robot.hpp"


Robot::Robot(
    DriveUnit& leftDriveUnit, 
    DriveUnit& rightDriveUnit,  
    float wheelBase,
    float leftWheelRadius,
    float rightWheelRadius
): 
    leftDriveUnit(leftDriveUnit),
    rightDriveUnit(rightDriveUnit),
    wheelBase(wheelBase),
    leftWheelRadius(leftWheelRadius),
    rightWheelRadius(rightWheelRadius) {

    if (wheelBase <= 0)
        wheelBase = 0.1;    // Default to 10 cm between wheels

    if (leftWheelRadius <= 0)
        leftWheelRadius = 0.025;    // Default to 5 cm diameter wheel

    if (rightWheelRadius <= 0)
        rightWheelRadius = leftWheelRadius;

    // Flip the right motor so that a command in range [0, 1] makes it spin forward
    rightDriveUnit.flip();
}

void Robot::enable() {
    leftDriveUnit.enable();
    rightDriveUnit.enable();
}

void Robot::disable() {
    leftDriveUnit.disable();
    rightDriveUnit.disable();
}

void Robot::enablePID() {
    leftDriveUnit.enablePID();
    rightDriveUnit.enablePID();
}

void Robot::disablePID() {
    leftDriveUnit.disablePID();
    rightDriveUnit.disablePID();
}

void Robot::enableKalman() {
    leftDriveUnit.enableKalman();
    rightDriveUnit.enableKalman();
}

void Robot::disableKalman() {
    leftDriveUnit.disableKalman();
    rightDriveUnit.disableKalman();
}

void Robot::setVelocity(float linVel, float angVel) {

    // Convert linear and angular velocities to left and right wheel velocities
    float vL, vR;
    linAngToLeftRight(linVel, angVel, leftWheelRadius, rightWheelRadius, wheelBase, vL, vR);

    // Track the setpoint velocities
    setLeftRightVelocity(vL, vR);
}


void Robot::setRPMs(float leftRPM, float rightRPM) {

    // Convert left and right RPMs to left and right wheel velocities
    float vL, vR;
    RPMToLeftRight(leftRPM, rightRPM, leftWheelRadius, rightWheelRadius, vL, vR);

    // Track the setpoint velocities
    setLeftRightVelocity(vL, vR);
}

void Robot::setLeftRightVelocity(float leftWheelVelocity, float rightWheelVelocity) {
    leftDriveUnit.setTargetAngularVelocity(leftWheelVelocity);
    rightDriveUnit.setTargetAngularVelocity(rightWheelVelocity);
}

void Robot::linAngToLeftRight(
    float v, float w, 
    float radiusLeft, float radiusRight,
    float wheelBase, 
    float& omegaLeft, float& omegaRight
) const {

    // Convert linear and angular velocity of the robot to left and right wheel velocities
    omegaLeft  = (v - (w * wheelBase / 2.0f)) / radiusLeft;
    omegaRight = (v + (w * wheelBase / 2.0f)) / radiusRight;

}

void Robot::RPMToLeftRight(
    float rpmLeft, float rpmRight, 
    float radiusLeft, float radiusRight, 
    float& vLeft, float& vRight
) const {
    
    // Convert RPM to linear velocity (m/s)
    vLeft  = (2.0f * M_PI * radiusLeft  * rpmLeft)  / 60.0f;
    vRight = (2.0f * M_PI * radiusRight * rpmRight) / 60.0f;
}

void Robot::setControlLoopPeriodHz(int hz) {
    leftDriveUnit.setControlLoopPeriodHz(hz);
    rightDriveUnit.setControlLoopPeriodHz(hz);
}

void Robot::update() {
    leftDriveUnit.update();
    rightDriveUnit.update();
}
