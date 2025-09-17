#include "robot.hpp"


Robot::Robot(DriveUnit& left, DriveUnit& right, float wheelRadius, float wheelBase)
    : leftDriveUnit(left),
      rightDriveUnit(right),
      wheelRadius(wheelRadius),
      wheelBase(wheelBase) {

    // Flip the left motor such that both motors spin on the same direction going forward and backward
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

void Robot::enableFiltering() {
    _filterEnabled = true;
}

void Robot::disableFiltering() {
    _filterEnabled = false;
}

void Robot::enableDeadband() {
    _deadbandEnabled = true;
}

void Robot::disableDeadband() {
    _deadbandEnabled = false;
}

void Robot::setLeftGains(float kp, float ki, float kd) {
    leftDriveUnit.setGains(kp, ki, kd);
}

void Robot::setRightGains(float kp, float ki, float kd) {
    rightDriveUnit.setGains(kp, ki, kd);
}

void Robot::setVelocity(float linearVel, float angularVel) {
    /**
     * Set the target linear (m/s) and angular (rad/s) velocities
     */

    computeWheelRPM(linearVel, angularVel);
    leftDriveUnit.setTargetRPM(targetLeftRPM);
    rightDriveUnit.setTargetRPM(targetRightRPM);
}

void Robot::setRPM(float leftRPM, float rightRPM) {

    /**
     * Set the target linear (m/s) and angular (rad/s) velocities
     */
    
    targetLeftRPM = leftRPM;
    targetRightRPM = rightRPM;

    leftDriveUnit.setTargetRPM(targetLeftRPM);
    rightDriveUnit.setTargetRPM(targetRightRPM);
}

void Robot::computeWheelRPM(float linearVel, float angularVel) {
    /**
    * Transform linear & angular velocities to left/right wheel RPM
    */
    
    // v = linearVel, w = angularVel
    float vLeft  = linearVel - (wheelBase / 2.0) * angularVel;
    float vRight = linearVel + (wheelBase / 2.0) * angularVel;

    // Convert m/s to RPM
    targetLeftRPM  = (vLeft  / (2.0 * PI * wheelRadius)) * 60.0;
    targetRightRPM = (vRight / (2.0 * PI * wheelRadius)) * 60.0;
}

void Robot::update() {
    
    /*
     * A single DriveUnit will stabilyze itself to the target. 
     * If we have multiple DriveUnit, each will try to do the same
     * independently, but this will result in jitter (in a differential 
     * drive robot, a small zigzag pattern when going forward).
     * To address the problem, instead of letting the DriveUnit apply
     * the control signal to its motor, we handle it here.
     */
    leftDriveUnit.update(false);
    rightDriveUnit.update(false);

    /*
     * The DriveUnits have an update loop of 10Hz, so the value they produce
     * stays the same for 1/10=0.1 seconds and then abruptly change. To 
     * address this we can simply smooth the control and feed it at a faster rate
     */
    float leftControl = leftDriveUnit.getControl();
    float rightControl = rightDriveUnit.getControl();

    if (_deadbandEnabled) {

        static float lastLeftControl  = 0.0f;  // static variables hold their value
        static float lastRightControl = 0.0f;

        if (fabs(leftControl - lastLeftControl) < _delta) {
            leftControl = lastLeftControl;  // discard tiny change
        } else {
            lastLeftControl = leftControl;  // accept new value
        }

    }

    if (_filterEnabled) {

        static float leftFiltered = 0.0f;
        static float rightFiltered = 0.0f;

        leftFiltered = _alpha * leftControl + (1 - _alpha) * leftFiltered;
        rightFiltered = _alpha * rightControl + (1 - _alpha) * rightFiltered;

        // Apply smoothed control signals to motors
        leftDriveUnit.applyControl(leftFiltered);
        rightDriveUnit.applyControl(rightFiltered);
      
    } else {
      
        leftDriveUnit.applyControl(leftControl);
        rightDriveUnit.applyControl(rightControl);

    }

    Serial.print("max_RPM:");
    Serial.print(leftDriveUnit.getMaxRPM());
    Serial.print(",");
    Serial.print("left_RPM:");
    Serial.print(leftDriveUnit.getCurrentRPM());
    Serial.print(",");
    Serial.print("right_RPM:");
    Serial.print(rightDriveUnit.getCurrentRPM());
    Serial.print(",");
    Serial.print("min_RPM:");
    Serial.print(-leftDriveUnit.getMaxRPM());
    Serial.println();
}

float Robot::getLeftRPM() const {
    /**
     * Get the current left wheel velocity in RPM
     */
    return leftDriveUnit.getCurrentRPM();
}

float Robot::getRightRPM() const {
    /**
     * Get the current right wheel velocity in RPM
     */
    return rightDriveUnit.getCurrentRPM();
}