#include "SpeedMotor.h"

SpeedMotor::SpeedMotor(SpeedMotorPins pins, float kp, float ki, int encDir)
  : _pins(pins), _dir(encDir), _pi(kp, ki) {}

void SpeedMotor::begin(int pwmFreq, int pwmRes) {
  _maxPwm = (1 << pwmRes) - 1;
  _pi.setLimits(-_maxPwm, _maxPwm);

  ledcAttach(_pins.in1, pwmFreq, pwmRes);
  ledcAttach(_pins.in2, pwmFreq, pwmRes);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  _enc.attachFullQuad(_pins.encA, _pins.encB);
  _enc.clearCount();
  _last = 0;

  drive(0);
}

void SpeedMotor::setTarget(int ticksPerPeriod) { _target = ticksPerPeriod; }

long SpeedMotor::update() {
  long now = _dir * (long)_enc.getCount();
  _measured = now - _last;
  _last = now;

  if (_target == 0) {
    _pi.reset();
    drive(0);
    return _measured;
  }

  // The integral is the accumulated tick error, so motors given the same
  // target also end up having travelled the same distance.
  drive((int)_pi.update(_target, _measured));
  return _measured;
}

void SpeedMotor::drive(int pwm) {
  _pwm = constrain(pwm, -_maxPwm, _maxPwm);
  if (_pwm >= 0) { ledcWrite(_pins.in1, _pwm);  ledcWrite(_pins.in2, 0); }
  else           { ledcWrite(_pins.in1, 0);     ledcWrite(_pins.in2, -_pwm); }
}
