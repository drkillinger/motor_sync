// Signed PI controller with output clamp and simple anti-windup.
// Pure math, no hardware - usable for any control loop.
#pragma once

class PIController {
public:
  PIController(float kp, float ki, float outMin = -255, float outMax = 255);

  // Call once per control period. Returns the clamped output.
  float update(float target, float measured);

  void reset();                                // clear the integral
  void setGains(float kp, float ki);
  void setLimits(float outMin, float outMax);

  float integral() const { return _integral; }

private:
  float _kp, _ki;
  float _min, _max;
  float _integral = 0;
};
