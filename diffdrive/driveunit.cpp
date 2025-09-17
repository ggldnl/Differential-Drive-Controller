#include "driveunit.hpp"

DriveUnit::DriveUnit(Motor& motor, Encoder& encoder, float ticksPerRevolution, float maxRPM)
  : _motor(motor), _encoder(encoder), _ticksPerRev(ticksPerRevolution), _maxRPM(maxRPM), _pid(0.0, 0.0, 0.0) {
}

void DriveUnit::enable() {
  _motor.enable();
}

void DriveUnit::disable() {
  _motor.disable();
}

void DriveUnit::enablePID() {
  _pidenabled = true;
}

void DriveUnit::disablePID() {
  _pidenabled = false;
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

float DriveUnit::getControl() const {
  /**
   * Returns the control signal (in RPM) produced by the PID controller
   */
  return _control;
}

float DriveUnit::applyControl(float controlRPM) const {
  /**
   * Take the control signal (in RPM), scale it in the interval [-1, 1]
   * and apply it.
   */
  _motor.drive(controlRPM / _maxRPM);
}

void DriveUnit::setGains(float Kp, float Ki, float Kd, float Kf = 0.0) const {
  _pid.setGains(Kp, Ki, Kd, Kf);
}

void DriveUnit::setUpdateRate(int hz) {
    if (hz > 0) {
        _updatePeriodMs = 1000 / hz;
    }
}

void DriveUnit::update(bool apply) {

  unsigned long now = millis();
  unsigned long elapsed = now - _lastTime;
  if (elapsed < _updatePeriodMs) return;

  float dt = elapsed / 1000.0f;  // dt in seconds
  _lastTime = now;

  // Skip loop iteration if too soon or bad update
  if (isnan(dt) || isinf(dt)) return;

  // Measure encoder ticks from the last time we checked
  long currentTicks = _encoder.getTicks();
  long tickDelta = currentTicks - _lastTicks;
  _lastTicks = currentTicks;

  /*
   * We don't have quadrature encoders (not enough pins) so we 
   * need to estimate the direction based on the control signal
   * we gave to the motor the previous iteration. 
   */
  float direction = (_control >= 0) ? 1.0f : -1.0f;
  float ticksPerSec = tickDelta / dt;
  float revPerSec = ticksPerSec / _ticksPerRev;
  float instantRPM = direction * revPerSec * 60.0f;

  // Skip bad RPM measurements
  if (isnan(instantRPM) || isinf(instantRPM)) return;

  // Smooth RPM measurement and update current RPM
  _rpmHistory[_rpmIndex] = instantRPM;
  _rpmIndex = (_rpmIndex + 1) % RPM_SMOOTHING;
  
  float rpmSum = 0.0f;
  for (int i = 0; i < RPM_SMOOTHING; i++) rpmSum += _rpmHistory[i];
  _currentRPM = rpmSum / RPM_SMOOTHING;

  // PID control
  if (_pidenabled) {
    _control = _pid.update(_targetRPM, _currentRPM, dt);
  } else {
    _control = _targetRPM;
  }

  /*
   * Apply the control (e.g. you are using a single DriveUnit) or
   * simply return the control to an object that encloses more than
   * one DriveUnits to synchronize/smooth the control signals
   * (otherwise we have jitter due to different DriveUnits 
   * stabilizying their RPM independently)
   */
  if (apply)
    applyControl(_control);

  /*
  Serial.print("setpoint:");
  Serial.print(_targetRPM);
  Serial.print(",");
  Serial.print("measurement:");
  Serial.print(_currentRPM);
  Serial.print(",");
  // Serial.print("control:");
  // Serial.print(_control);
  // Serial.print(",");
  Serial.print("error:");
  Serial.print(_targetRPM - _currentRPM);
  Serial.print(",");
  Serial.print("max:");
  Serial.print(_maxRPM);
  Serial.print(",");
  Serial.print("min:");
  Serial.print(-_maxRPM);
  Serial.println();
  */
}