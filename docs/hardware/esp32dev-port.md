# Experimental port: ESP32-WROOM-32 DevKit (LEDC servos, no PCA9685)

Working notes for the `esp32dev` / `TE_BOARD_ESP32DEV` experiment, so the next
session (human or agent) can pick this up without re-deriving it. Pin table
and avoided-pins list are the single source of truth in
[pinout.md](pinout.md#experimental-esp32-wroom-32-devkit-ledc-no-pca9685) —
this file is the *why* and *what's left*, not a duplicate of the *what*.
A printable one-page version for the bench is
[esp32dev-wiring-reference.pdf](esp32dev-wiring-reference.pdf).

## Goal

Run the same firmware application (Wi-Fi, HTTP API, animations, OLED, I2S
audio) on a plain ESP32-WROOM-32 DevKit instead of the Waveshare
ESP32-C3-Zero + PCA9685 main control board, with the 5 servo PWM signals
generated directly by the ESP32's own LEDC peripheral. Driven by the user
already having a WROOM-32 wired to a MAX98357A I2S DAC and wanting to reuse
it, not by a plan to replace the main control board.

The C3/PCA9685 build stays the default and fully unchanged; `esp32dev` is a
second, parallel PlatformIO environment.

## Architecture decision

Rather than sprinkling `#ifdef` through the existing motion/pose/slew/NVS
logic in `servo_wrapper.cpp`, the hardware boundary was extracted into
`src/hardware/servo_backend.{h,cpp}`. `servo_wrapper.cpp` now only calls
`angleToPulse()` / `writeChannel()` / `writeChannelVerified()` /
`initServoOutputPin()` / `disableServoOutputs()` / `enableServoOutputs()` /
`servoBackendBringUp()` / `servoPwmDriverState()` / `setUpServoPwmDriver()` —
all declared once in `servo_backend.h`, defined twice:

- `#ifdef TE_BOARD_ESP32DEV`: `ledcAttach`/`ledcWrite` per channel, 50 Hz,
  16-bit. `kServoOffValue = 0` (0% duty = no pulses = de-energized, the same
  safety behavior the PCA9685 OE pin gave). No chip-level output-enable
  exists on this backend, so `disableServoOutputs()`/`enableServoOutputs()`
  are no-ops — the real "outputs off" effect is per-channel 0 duty, driven by
  the existing (unchanged) `writeServoChannelsOff()` in `servo_wrapper.cpp`.
- `#else`: the original PCA9685 logic, moved verbatim (not rewritten) —
  `Adafruit_PWMServoDriver`, mode/prescale setup, 12-bit count math,
  `kServoOffValue = 4096`.

This follows the existing `TE_OTA` flag precedent (`src/network/ota.h`,
`platformio.ini` `extends` + `build_flags` append) — same pattern, new flag
name `TE_BOARD_ESP32DEV`.

`src/hardware/i2c_health.cpp` is guarded the same way: all PCA9685-driver
health tracking (`g_servoDriver`, `recoverServoDriver()`,
`i2cNoteServoDriverFault()`, etc.) is `#ifndef TE_BOARD_ESP32DEV`-only; on
`esp32dev` `i2cServoDriverUp()` always returns `true` and `i2cHealth()`
reports a synthetic always-healthy PCA9685 field so `/health` keeps the same
JSON shape on both boards.

## Decisions made along the way

- **Servo output disable stays internal-only.** User explicitly asked for a
  software equivalent of the PCA9685 OE pin, then chose "internal only" over
  adding a new HTTP route when asked — used for boot sleep / idle
  power-release / future fault-recovery, never exposed via the API. Avoids
  the CLAUDE.md HTTP-API-sync checklist (kRoutes, docs/api.md,
  ApiReference.tsx, README) entirely.
- **Pin assignment kept as originally chosen: GPIO25/27/32/33/18 for the 5
  servos.** User later asked whether GPIO 25/33/32/35/34 would be easier to
  wire; GPIO34/35 are hardware input-only (ADC1, no output driver) on the
  classic ESP32 — cannot drive PWM under any firmware, so that's a hard
  constraint, not a software choice. User chose to leave the existing
  assignment unchanged rather than swap in GPIO26 (currently I2S DIN) or
  GPIO12 (flash-voltage strapping pin) for a same-silkscreen-row 5th pin.
- **I2S DIN moved from the user's original GPIO22 to GPIO26** before any
  firmware work, to avoid the Arduino-default I2C pins (GPIO21/22 are the
  default SDA/SCL, used here for the OLED).

## Verification done

- `pio run -e esp32-c3-devkitm-1` — unchanged C3 target still builds clean
  (regression check for the `servo_wrapper.cpp` extraction).
- `pio run -e esp32dev` — new target builds clean.
- `pio test -e native` — 157/157 passing (servo norm/slew logic is
  chip-independent, untouched).
- **Real hardware**, flashed and tested over HTTP (`tiny-engineer.local`,
  since the serial link had an unrelated tooling problem — see below): 14+
  minutes stable uptime, Wi-Fi connected, OLED healthy, `cpu_temp_c: null` as
  expected (classic ESP32 has no on-chip temp sensor), all 5 LEDC servo
  channels responded correctly to `POST /test/servo`, and a combined
  `POST /anim` (audio + servo + eyes together) ran without destabilizing the
  board.

## Known limitations / open items (not yet acted on)

Flagged to the user, explicitly deferred — pick up only if asked:

1. **`/health` JSON still calls the field `"pca9685"` on the LEDC board.**
   Hardcoded in `src/http/health_handlers.cpp`, untouched by this port.
   Cosmetic only — the synthetic always-healthy value is correct, just
   misleadingly named for a board that has no PCA9685.
2. **No I2C bus-recovery path left for OLED-only faults on `esp32dev`.** On
   the C3/PCA9685 board, `restartBus()` is reachable via the servo-driver
   recovery path as a side effect; since that whole path is `#ifndef
   TE_BOARD_ESP32DEV`-only, an OLED-only I2C fault on the LEDC board has no
   recovery trigger. Accepted as a reasonable scope cut for an experimental
   build, not fixed.
3. **Serial-link ("sync") protocol times out via `tiny-engineer-cli`
   tooling on this dev machine** (`port --probe` / `monitor` / `doctor
   --transport serial`), on the CP2102 USB-UART bridge. Not diagnosed — HTTP
   testing fully substituted for it and confirmed the firmware itself is
   fine. Working theory, untested: the CP2102's DTR/RTS-driven auto-reset
   circuit resets the board on serial port open/close, racing the sync
   handshake. If this needs fixing later, start there rather than
   re-deriving from scratch — but it is a test-tooling/environment issue,
   not a sign of a firmware defect.

## Files touched by this port

New: `src/hardware/servo_backend.{h,cpp}`.

Modified: `src/hardware/servo_wrapper.{h,cpp}`, `src/hardware/pca9685_servos.cpp`,
`src/hardware/i2c_health.cpp`, `src/hardware/chip_temp.cpp` (classic ESP32 has
no on-chip temp sensor — stubbed under the same flag), `src/main.cpp` (guarded
`Serial.setTxTimeoutMs()` behind `SOC_USB_SERIAL_JTAG_SUPPORTED`, since that
method only exists on native-USB-CDC boards), `include/pins.h`,
`platformio.ini` (new `[env:esp32dev]`), `docs/hardware/pinout.md`,
`docs/hardware/README.md`.

None of this is committed yet (as of 2026-11-26) — it's sitting as working
tree changes. Commit scope would be `firmware` (and/or `servos`) per
CLAUDE.md conventions whenever that happens.
