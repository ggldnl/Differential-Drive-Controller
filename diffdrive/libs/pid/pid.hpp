#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>


class PID {
public:
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);

    // Set PID gains
    void setProportional(double kp);
    void setIntegral(double ki);
    void setDerivative(double kd);
    void setGains(double kp, double ki, double kd);

    // Getters
    double getProportional() const;
    double getIntegral() const;
    double getDerivative() const;

    // Reset internal state (integral, last error)
    void reset();

    // Compute the control output
    //   setpoint: desired value
    //   measurement: actual feedback (ignored if feedback disabled)
    //   dt_ms: time since last update in milliseconds
    double update(double setpoint, double measurement, double dt_ms);

private:

    double Kp, Ki, Kd;

    double integral;
    double prevError;
    bool firstUpdate;
};

#endif // PID_HPP
