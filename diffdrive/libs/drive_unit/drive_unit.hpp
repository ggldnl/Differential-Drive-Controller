#ifndef DRIVEUNIT_HPP
#define DRIVEUNIT_HPP

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <Arduino.h>

#include "libs/motor/motor.hpp"
#include "libs/encoder/encoder.hpp"
#include "libs/kalman_filter/kalman_filter.hpp"
#include "libs/pid/pid.hpp"


class DriveUnit {

  public:

    DriveUnit(Motor& motor, Encoder& encoder, float ticksPerRevolution, float maxRPM, float feedforward=0);

    // Enable/disable the whole Drive Unit
    void enable();
    void disable();

    // Enable/disable PID
    void enablePID();
    void disablePID();

    // Enable/disable 1D Kalman filter
    void enableKalman();
    void disableKalman();

    // Flip the forward direction of the motor
    void flip();

    // Set target (RPM [r/60s] or angular velocity [rad/s^2])
    void setTargetRPM(float rpm);
    void setTargetAngularVelocity(float w);

    // Getters
    float getTargetRPM() const;
    float getTargetAngularVelocity() const;

    float getCurrentRPM() const;
    float getCurrentAngularVelocity() const;

    float getMaxRPM() const;
    float getMaxAngularVelocity() const;

    // Set PID/Kalman gains
    void setPIDGains(float kp, float ki, float kd);
    void setKalmanGains(float q, float r, float p);
    void setFeedforward(float kf);
    
    void setControlLoopPeriodHz(int hz);
    void update();  // To be called regularly in the loop

  private:

    Motor& _motor;
    Encoder& _encoder;

    // PID controller
    bool _pidEnabled = true;
    float _feedforward;
    long _updateIntervalMicros = 100000;  // Default = 10 Hz
    PID _pid;

    // Kalman filter
    bool _kalmanEnabled = true;
    KalmanFilter _kalman;

    // Support variables for RPM computation
    float _ticksPerRev;
    unsigned long _lastUpdateMicros = 0;
    long _lastTicks = 0;

    // PID input and output variables
    float _targetRPM = 0.0f;
    float _currentRPM = 0.0f;
    float _maxRPM = 50.0f;   // Sample value for N20 motors 
    float _control = 0.0f;

    // Convert RPM to rad/s
    float rpmToAngular(float rpm) const;

    // Convert rad/s to RPM
    float angularToRPM(float angular) const;

    // Compute RPM if encoder in Count Mode
    float computeCountMode(long deltaTicks, long deltaTime, int ticksPerRev) const;

    // Compute RPM if encoder in Period Mode
    float computePeriodMode(float tickIntervalMicros, int ticksPerRev) const;

};

#endif
