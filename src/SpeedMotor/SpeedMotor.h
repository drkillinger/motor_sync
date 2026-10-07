// DC motor with quadrature encoder, driven by an Adafruit DRV8871,
// with closed-loop speed control via PIController.
// ESP32 (Arduino core 3.x). Requires library: ESP32Encoder (by Kevin Harrington / madhephaestus)
//
// Encoder wiring:
//   Blue  (Encoder+) -> 3.3V   (NOT 5V - keeps the signals safe for the ESP32)
//   Black (Encoder-) -> GND
//   Yellow (A)       -> encA
//   Green  (B)       -> encB
//
// Speed is in encoder ticks per control period (full quad = 4 ticks per pulse).
// If the motor shows NEGATIVE speed when driving forward, pass encDir = -1.
#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "../PIController/PIController.h"

struct SpeedMotorPins {
  int in1, in2;   // DRV8871 IN1 / IN2
  int encA, encB; // encoder yellow / green
};

class SpeedMotor {
public:
  SpeedMotor(SpeedMotorPins pins, float kp, float ki, int encDir = 1);

  // Attach PWM and encoder. Call from setup().
  void begin(int pwmFreq = 20000, int pwmRes = 8);

  // Signed ticks per control period (+ forward, - reverse, 0 = stop).
  void setTarget(int ticksPerPeriod);

  // Call exactly once per control period. Returns measured ticks this period.
  long update();

  // Open-loop signed PWM (+ forward, - reverse). The next update()
  // overrides it, so stop calling update() while driving open-loop.
  void drive(int pwm);

  int  target()   const { return _target; }
  long measured() const { return _measured; }
  int  pwm()      const { return _pwm; }

  PIController& pi() { return _pi; }

private:
  SpeedMotorPins _pins;
  int  _dir;
  int  _maxPwm = 255;
  ESP32Encoder _enc;
  PIController _pi;

  int  _target = 0;
  long _last = 0;
  long _measured = 0;
  int  _pwm = 0;
};
