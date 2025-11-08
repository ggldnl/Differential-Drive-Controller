#include "drive_unit.hpp"


DriveUnit::DriveUnit(
  Motor& motor, 
  Encoder& encoder, 
  float ticksPerRevolution, 
  float maxRPM, 
  float feedforward
): 
    _motor(motor), 
    _encoder(encoder), 
    _ticksPerRev(ticksPerRevolution), 
    _maxRPM(maxRPM), 
    _pid(0.0, 0.0, 0.0),
    _kalman(0.0, 0.0, 0.0),
    _feedforward(feedforward) {

      _lastUpdateMicros = micros();
}

void DriveUnit::enable() {
  _motor.enable();
}

void DriveUnit::disable() {
  _motor.disable();
}

void DriveUnit::enablePID() {
  _pidEnabled = true;
}

void DriveUnit::disablePID() {
  _pidEnabled = false;
}

void DriveUnit::enableKalman() {
  _kalmanEnabled = true;
}

void DriveUnit::disableKalman() {
  _kalmanEnabled = false;
}

void DriveUnit::flip() {
  _motor.flip();
}

void DriveUnit::setTargetRPM(float rpm) {
  if (rpm > _maxRPM) rpm = _maxRPM;
  if (rpm < -_maxRPM) rpm = -_maxRPM;
  _targetRPM = rpm;
}

void DriveUnit::setTargetAngularVelocity(float w) {
  float rpm = angularToRPM(w);
  setTargetRPM(rpm);
}

float DriveUnit::getCurrentRPM() const {
  return _currentRPM;
}

float DriveUnit::getCurrentAngularVelocity() const {
  return rpmToAngular(_currentRPM);
}

float DriveUnit::getTargetRPM() const {
  return _targetRPM;
}

float DriveUnit::getTargetAngularVelocity() const {
  return rpmToAngular(_targetRPM);
}

float DriveUnit::getMaxRPM() const {
  return _maxRPM;
}

float DriveUnit::getMaxAngularVelocity() const {
  return rpmToAngular(_maxRPM);
}

void DriveUnit::setPIDGains(float Kp, float Ki, float Kd) {
  _pid.setGains(Kp, Ki, Kd);
}

void DriveUnit::setKalmanGains(float q, float r, float p) {
  _kalman.setGains(q, r, p);
}

void DriveUnit::setFeedforward(float Kf) {
  _feedforward = Kf;
}

void DriveUnit::setControlLoopPeriodHz(int hz) {
    if (hz > 0) {
        _updateIntervalMicros = 1000000 / hz;
      }
}

float DriveUnit::rpmToAngular(float rpm) const {
    // 1 revolution = 2pi radians
    // 1 minute = 60 seconds
    return rpm * (2.0f * M_PI / 60.0f);
}

float DriveUnit::angularToRPM(float angularVelocity) const {
    // Inverse of the above
    return angularVelocity * (60.0f / (2.0f * M_PI));
}


float DriveUnit::computeCountMode(long deltaTicks, long deltaTime, int ticksPerRev) const {
    return (60.0f * 1000000.0f * deltaTicks) / (ticksPerRev * deltaTime);
}

float DriveUnit::computePeriodMode(float tickIntervalMicros, int ticksPerRev) const {
    return (60.0f * 1000000.0f) / (tickIntervalMicros * ticksPerRev);
}

void DriveUnit::update() {

  // Compute delta from previous update
  unsigned long now = micros();
  unsigned long dt = now - _lastUpdateMicros;
  if (isnan(dt) || isinf(dt)) return;       // Skip loop iteration if bad update
  if (dt < _updateIntervalMicros) return;   // Skip loop iteration if too soon
  _lastUpdateMicros = now;

  EncoderData data = _encoder.getData();

  if (_encoder.getMode() == Encoder::COUNT_MODE) {

    long deltaTicks = data.ticks - _lastTicks;
    _currentRPM = computeCountMode(deltaTicks, dt, _ticksPerRev);
    _lastTicks = data.ticks;

  } else if (_encoder.getMode() == Encoder::PERIOD_MODE) {
    _currentRPM = computePeriodMode(data.dtMicros, _ticksPerRev);
  }

  /*
  * We don't have quadrature encoders (not enough pins) so we 
  * need to estimate the direction based on the control signal
  * we gave to the motor the previous iteration. 
  */
  float direction = (_control >= 0.0f) ? 1.0f : -1.0f;
  _currentRPM *= direction;

  // If Kalman filter enabled, smooth out RPM reading
  if (_kalmanEnabled)
    _currentRPM = _kalman.update(_currentRPM);

  // If PID enabled, compute a suitable control signal, else use target directly
  if (_pidEnabled) {
    _control = _pid.update(_targetRPM, _currentRPM, dt) + _targetRPM * _feedforward;
  } else {
    _control = _targetRPM;
  }

  _motor.drive(_control);
}
