# motor_sync

Arduino sketch for a two-wheel robot: two DC gearmotors with quadrature encoders,
closed-loop speed control per wheel, currently driving in a circle.

## Hardware
- Adafruit HUZZAH32 ESP32 Feather (original V1 pinout), Arduino ESP32 core 3.x
- 2x Adafruit DRV8871 motor drivers (just two PWM inputs per motor, no protocol)
- Encoders: 11 pulses/rev, read full-quad = 44 ticks per motor rev
- Encoder power from **3.3V, never 5V** (ESP32 pins are not 5V tolerant)

## Pin map
| | IN1 | IN2 | Enc A (yellow) | Enc B (green) |
|---|---|---|---|---|
| Motor 1 (left)  | 25 | 26 | 32 | 14 |
| Motor 2 (right) | 33 | 27 | 18 | 19 |

Avoid GPIO 12 (boot strapping — pull-up can stop the board booting) and 15.
GPIO 34/36/39 are input-only with no internal pull-ups (encoders need pull-ups).
22/23 are I2C SCL/SDA and currently free.

## Layout
- `motor_sync.ino` — config (pins, gains, speeds) and behaviour only; owns the
  fixed-period control loop (`LOOP_MS`), since speed is "ticks per period" and both
  motors must update in the same tick.
- `src/PIController/` — pure PI math (clamp + undo-integration anti-windup). No hardware.
- `src/SpeedMotor/` — one wheel: DRV8871 PWM + ESP32Encoder + PIController.

Libraries live in the sketch's `src/` so the folder is self-contained; include as
`#include "src/SpeedMotor/SpeedMotor.h"`.

## Conventions / decisions
- Speeds and PWM are **signed** (+ forward, − reverse). Target 0 = stop and reset integral.
- The PI integral is the sum of tick error (not scaled by dt) — it doubles as distance
  sync between wheels. Keep that property if the controller changes.
- Decided to keep our own `SpeedMotor`/`PIController` rather than a library. If more
  PID features are needed, swap QuickPID in *inside* `PIController` only.
- Wrong encoder sign → set that motor's `encDir` to -1. Robot spins instead of driving
  forward → motors are mirrored; swap one motor's wires (or IN1/IN2).

## Build
- Dependency: **ESP32Encoder** (madhephaestus) via Library Manager.
- Board FQBN: `esp32:esp32:featheresp32` (ESP32 core 3.3.x installed).
- Compile: `arduino-cli compile --fqbn esp32:esp32:featheresp32 .`
- Upload: `arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:featheresp32 .`
  (find the port with `arduino-cli board list`)
- `arduino-cli` is a symlink in `/opt/homebrew/bin` to the copy bundled inside
  Arduino IDE.app (Homebrew has no bottle for this macOS). It shares the IDE's cores
  and libraries, and updates when the IDE updates.
- Test with wheels off the ground; Serial Plotter @115200 shows `t1 t2 m1 m2 pwm1 pwm2`.
- Not a git repo.
