#include "PIController.h"

PIController::PIController(float kp, float ki, float outMin, float outMax)
  : _kp(kp), _ki(ki), _min(outMin), _max(outMax) {}

float PIController::update(float target, float measured) {
  float error = target - measured;
  _integral += error;
  float out = _kp * error + _ki * _integral;

  // clamp + simple anti-windup (undo the integration when saturated)
  if (out > _max) { out = _max; _integral -= error; }
  if (out < _min) { out = _min; _integral -= error; }
  return out;
}

void PIController::reset() { _integral = 0; }

void PIController::setGains(float kp, float ki) { _kp = kp; _ki = ki; }

void PIController::setLimits(float outMin, float outMax) { _min = outMin; _max = outMax; }
