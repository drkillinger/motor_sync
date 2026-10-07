// Two DC motors with quadrature encoders driving a robot in a circle.
// ESP32 (Arduino core 3.x) + 2x Adafruit DRV8871
// Requires library: ESP32Encoder (Library Manager, by Kevin Harrington / madhephaestus)
// Wiring notes: see src/SpeedMotor/SpeedMotor.h

#include "src/SpeedMotor/SpeedMotor.h"

// ---- Control loop ----
const int   LOOP_MS = 50;    // control period
const int   TARGET  = 60;    // outer wheel speed in encoder ticks per LOOP_MS (tune)
                             // full quad = 44 ticks per motor rev (11 pulses x 4)
const float KP = 1.5;        // proportional gain
const float KI = 0.4;        // integral gain

// ---- Circle ----
// Outer wheel runs at TARGET, inner wheel at TARGET * CIRCLE_RATIO.
// Smaller ratio = tighter circle (0 = pivot on inner wheel, 1 = straight).
// For a circle of radius R (to the robot's centre) and wheel spacing W:
//   CIRCLE_RATIO = (R - W/2) / (R + W/2)
const float CIRCLE_RATIO = 0.5;
const bool  TURN_LEFT    = true;   // true: counter-clockwise, false: clockwise

//                IN1 IN2 ENC_A ENC_B          encoder direction (+1 / -1)
SpeedMotor left ({25, 26, 32,   14},  KP, KI,  1);
SpeedMotor right({33, 27, 18,   19},  KP, KI,  1);

unsigned long lastLoop = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Motor circle");

  left.begin();
  right.begin();

  int inner = (int)(TARGET * CIRCLE_RATIO);
  left.setTarget (TURN_LEFT ? inner  : TARGET);
  right.setTarget(TURN_LEFT ? TARGET : inner);

  lastLoop = millis();
}

void loop() {
  if (millis() - lastLoop < LOOP_MS) return;
  lastLoop += LOOP_MS;

  left.update();
  right.update();

  // Serial Plotter friendly (all signed)
  Serial.printf("t1:%d t2:%d m1:%ld m2:%ld pwm1:%d pwm2:%d\n",
                left.target(), right.target(), left.measured(), right.measured(),
                left.pwm(), right.pwm());
}
