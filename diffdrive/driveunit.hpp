#ifndef DRIVEUNIT_HPP
#define DRIVEUNIT_HPP

#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"


class DriveUnit {

  public:

    DriveUnit(Motor& motor, Encoder& encoder, float ticksPerRevolution, float maxRPM);

    void enable();
    void disable();

    void enablePID();
    void disablePID();

    void flip();

    void setTargetRPM(float rpm);
    void setTargetAngularVelocity(float w);

    float getTargetRPM() const;
    float getTargetAngularVelocity() const;

    float getCurrentRPM() const;
    float getCurrentAngularVelocity() const;

    float getMaxRPM() const;
    float getMaxAngularVelocity() const;

    float getControl() const;
    float applyControl(float control) const;

    void setGains(float kp, float ki, float kd, float Kf = 0.0) const;

    void setUpdateRate(int hz);
    void update(bool apply = true);  // To be called regularly in the loop

  private:

    Motor& _motor;
    Encoder& _encoder;

    // PID controller
    bool _pidenabled = true;
    PID _pid;

    // Encoder RPM estimation
    float _control = 0;
    float _ticksPerRev;
    long _lastTicks = 0;
    unsigned long _lastTime = 0;
    unsigned long _updatePeriodMs = 100;

    // PID input and output variables
    float _targetRPM = 0.0;
    float _currentRPM = 0.0;
    float _maxRPM;

    // Moving average for RPM smoothing
    static const int RPM_SMOOTHING = 5;
    float _rpmHistory[RPM_SMOOTHING] = {0};
    int _rpmIndex = 0;

    // Convert RPM to rad/s
    float rpmToAngular(float rpm) const;

    // Convert rad/s to RPM
    float angularToRPM(float angular) const;
};

#endif
