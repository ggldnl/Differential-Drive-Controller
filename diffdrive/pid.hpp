#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0, double kf = 0.0);

    // Set PID gains
    void setGains(double kp, double ki, double kd, double kf = 0.0);

    // Enable or disable feedforward
    void setFeedforwardEnabled(bool enabled);

    // Reset internal state (integral, last error)
    void reset();

    // Compute the control output
    //   setpoint: desired value
    //   measurement: actual feedback (ignored if feedback disabled)
    //   dt_ms: time since last update in milliseconds
    double update(double setpoint, double measurement, double dt_ms);

private:
    double Kp, Ki, Kd, Kf;
    bool feedforwardEnabled;

    double integral;
    double prevError;
    bool firstUpdate;
};

#endif // PID_HPP
