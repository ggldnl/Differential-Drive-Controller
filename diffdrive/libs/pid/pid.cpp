#include "pid.hpp"


PID::PID(double kp, double ki, double kd)
    : Kp(kp), Ki(ki), Kd(kd),
      integral(0.0), prevError(0.0), firstUpdate(true) {}

void PID::setProportional(double kp) {
    kp = kp;
}

void PID::setIntegral(double ki) {
    ki = ki;
}

void PID::setDerivative(double kd) {
    kd = kd;
}

void PID::setGains(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

double PID::getProportional() const {
    return Kp;
}

double PID::getIntegral() const {
    return Ki;
}

double PID::getDerivative() const {
    return Kd;
}

void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
    firstUpdate = true;
}

double PID::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    double error = setpoint - measurement;

    // Integral
    integral += error * dt;

    // Clamp the integral with an heuristic limit
    // double maxIntegral = 0.5 * (1.0 / Ki);
    // if (integral > maxIntegral) integral = maxIntegral;
    // if (integral < -maxIntegral) integral = -maxIntegral;

    // Derivative
    double derivative = firstUpdate ? 0.0 : (error - prevError) / dt;
    firstUpdate = false;
    prevError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}
