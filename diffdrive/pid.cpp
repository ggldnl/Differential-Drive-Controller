#include "pid.hpp"

PID::PID(double kp, double ki, double kd, double kf = 0.0)
    : Kp(kp), Ki(ki), Kd(kd), Kf(kf),
      feedforwardEnabled(true),
      integral(0.0), prevError(0.0), firstUpdate(true) {}

void PID::setGains(double kp, double ki, double kd, double kf = 0.0) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Kf = kf;
}

void PID::setFeedforwardEnabled(bool enabled) {
    feedforwardEnabled = enabled;
}

void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
    firstUpdate = true;
}

double PID::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    double error = setpoint - measurement;

    // Integral with clamping
    integral += error * dt;
    double maxIntegral = 0.5 * (1.0 / Ki);  // heuristic limit
    // if (integral > maxIntegral) integral = maxIntegral;
    // if (integral < -maxIntegral) integral = -maxIntegral;

    // Derivative
    double derivative = firstUpdate ? 0.0 : (error - prevError) / dt;
    firstUpdate = false;
    prevError = error;

    // PID + feedforward
    double pidOutput = Kp * error + Ki * integral + Kd * derivative;
    double ffOutput = feedforwardEnabled ? Kf * setpoint : 0.0;

    return pidOutput + ffOutput;
}
