# Motion safety

*Applies to the Teensy 4.0 `linorobot2_overlay` firmware.*

Firmware can drive the motors directly, so the safety-relevant behaviours are documented here.
This is the current commissioning state; it is **not** a substitute for a hardware E-stop (none is
wired yet — see the [overlay module mapping](../../boards/teensy_4_0/linorobot2_overlay/README.md)).

## Command watchdog (`/cmd_vel` timeout)

If no fresh `/cmd_vel` arrives within **200 ms**, `moveBase()` performs a **deterministic full
stop**: it brakes all motors and resets the PIDs (not merely "set velocity to 0 with the PID still
running"). Odometry keeps updating. This is the primary software safety net — it works even if the
publisher dies or the network drops.

## Open-loop diagnostic bounds

The raw-PWM `/debug/openloop` path bypasses the PID and can move the motors, so it is fenced:

- **Input validation** — NaN/Inf are rejected (treated as PWM 0).
- **Magnitude clamp** — the value is clamped to `OPENLOOP_PWM_LIMIT = 0.7 · PWM_MAX ≈ ±716`, so a
  stray or oversized command cannot slam the motors at full PWM.
- **Staleness timeout** — the command is only honoured for **300 ms**; it must be republished to
  keep driving, so a dropped publisher stops the wheels.
- **Production gate** — powered raw debug requires the `ENABLE_POWERED_DEBUG` build flag. Remove it
  for a production image and `/debug/openloop` can no longer move the motors (the subscriber stays,
  so the executor layout is unchanged).

See [debug telemetry](../architecture/debug-telemetry.md) for the full interface.

## Startup / agent-loss behaviour

- The control loop only runs while the micro-ROS agent is connected. On **agent disconnect** the
  firmware calls `fullStop()` and tears down its ROS entities, then waits to reconnect.
- IMU init failure is treated as **fatal**: the firmware blinks the LED and does not run the
  control loop (it never silently drives with a dead IMU). See
  [micro-ROS bringup](../bringup/micro-ros-bringup.md) for the LED blink codes.
  - Note: the board is an **MPU6500 (no magnetometer)** (Gate A unit; the 2.0 IMU is the ICM-42688-P) driven via the MPU9250 driver. The
    magnetometer path is not a real sensor here — treat a "magnetometer init" blink code as
    informational, not a safety stop, and do not use `/imu/mag` as a heading source.

## Measured velocity floors (operating limits)

Closed-loop sweep on the ground (under load, PID + dither, like docking), 2026-07-02:

| Axis | Stalls | Judders | Reliable floor | Clean floor |
|---|---|---|---|---|
| Linear  | ≤ 0.02 m/s | ~0.03 m/s | **0.04 m/s** | 0.05 m/s |
| Angular | ≤ 0.08 rad/s | ~0.10–0.12 rad/s | **0.15 rad/s** | 0.20 rad/s |

- Keep commanded speeds **above** these floors (docking tapers are floored at 0.05 m/s, rotation at
  0.17 rad/s). Below the floor the wheels judder or stall from stick-slip.
- **The motor is well-sized, not under-torqued.** Above the floors the real/commanded velocity
  ratio is ~1.0. The floors are **static friction (stick-slip) + coarse Hall commutation at low
  RPM** (an operating-point limit), not a torque shortfall. The anti-stiction dither (see
  [control loop](../architecture/control-loop.md)) is what pushes the clean floor down toward
  ~0.06 m/s for docking.

## Hardware safety note — encoder over-voltage

The Teensy 4.0 is **not 5 V tolerant** (3.3 V max). The AS5040 encoders accept 3.3–5 V, so they
**must be powered from the 3.3 V rail**: a 5 V supply drove ~4–5 V on the A/B lines, forcing the
Teensy's internal clamp diodes to conduct (out of spec — slow pin degradation, possible dropped
counts). This was corrected on 2026-06-19 by moving the encoder supply to 3.3 V (A/B now cap at
~3.3 V, clean counting). If the 3.3 V rail ever browns out under load, the fallback is a series
resistor (~1–2.2 kΩ) or divider per A/B line, or a level-shifter. Wiring details live in the
hardware repo (`openamr-platform-hw`).

## Assurance fork hardening (2026-10-20, draft, not yet verified on hardware)

The assurance fork adds the following to the overlay. Change record and impact analysis:
`openamrobot-assurance/assurance/07-supporting/impact/change-2026-10-20-fw-hardening.md`.

- **Hardware watchdog.** The i.MX RT1062 WDOG1 is enabled at boot with a 1.0 s timeout and serviced from the
  control-loop timer (agent connected) or from `loop()` (otherwise). A hung core or a wedged executor resets the
  MCU; PWM is 0 after reset.
- **`/firmware/status`** (`std_msgs/String`, 1 Hz): firmware version, reset cause (`SRC_SRSR` at boot), watchdog
  timeout, whether powered debug is compiled in, motion-fault bits, whether the encoder table is loaded, counts of
  rejected and clamped commands, uptime. A silent reboot shows as a new reset cause and `enc_cal=0`.
- **`/cmd_vel` validation.** A command with a non-finite field becomes zero and does not feed the 200 ms watchdog,
  so the base stops. `|linear.x|` is clamped to `CMD_VEL_MAX_LINEAR_MPS` (0.25 m/s) and `|angular.z|` to
  `CMD_VEL_MAX_ANGULAR_RPS` (0.6 rad/s); `linear.y` is forced to 0. These are the navigation limits plus a margin
  (assurance decision DQ-006, 2026-10-20); a bench image that needs faster motion overrides the two defines.
- **Powered debug is OFF** in the reference build (`ENABLE_POWERED_DEBUG` commented out): `/debug/openloop`
  cannot drive the motors and `/debug/tune` is rejected. A bench image defines it; there, `/debug/tune` accepts
  only finite gains in [0, 50], a right-wheel gain in [0.5, 2.0], feedforward ≤ 50 and dither ≤ 200.
- **`/debug/enc_cal`** loads a table only if all 72 ratios are finite and in [0.5, 2.0]; otherwise nothing changes.
  The per-boot alignment ritual still needs the open-loop spin and therefore a bench image.
- **Stall / encoder-loss stop.** A wheel commanded above 60 % PWM whose measured speed stays below 2 rpm for 1 s
  latches a motion fault: both wheels stop and the PIDs reset. The latch clears after `/cmd_vel` has commanded
  zero for 1 s. Thresholds are drafts for the bench test.
- `PWM_MIN` is now −1023 (was −1025 through a missing parenthesis).

None of this is a substitute for the hardware E-stop, which is still not wired.
