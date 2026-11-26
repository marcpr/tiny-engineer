#include <Arduino.h>
#include <cstring>

#include "pins.h"
#include "servos.h"
#include "hardware/servo_backend.h"

#ifdef TE_BOARD_ESP32DEV

// ESP32-WROOM-32 experiment: 5 LEDC channels drive the servos directly, no
// PCA9685 on the bus. See docs/hardware/pinout.md.

namespace {

constexpr uint16_t kServoPwmHz = 50;
constexpr uint8_t kLedcResolutionBits = 16;
constexpr uint32_t kLedcMaxDuty = (1UL << kLedcResolutionBits) - 1;  // 65535
constexpr uint32_t kServoPeriodUs = 1000000UL / kServoPwmHz;        // 20000

}  // namespace

const uint16_t kServoOffValue = 0;  // 0% duty: no pulses on that pin

uint16_t angleToPulse(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);

  const float pulseUs =
    SERVO_MIN_US +
    (angle / 180.0f) *
    (SERVO_MAX_US - SERVO_MIN_US);

  const float duty =
    pulseUs * (float)kLedcMaxDuty / (float)kServoPeriodUs + 0.5f;

  return (uint16_t)constrain(duty, 0.0f, (float)kLedcMaxDuty);
}

void writeChannel(uint8_t channel, uint16_t value) {
  ledcWrite(SERVO_GPIO_PINS[channel], value);
}

bool writeChannelVerified(uint8_t channel, uint16_t value) {
  ledcWrite(SERVO_GPIO_PINS[channel], value);
  return true;
}

void initServoOutputPin() {}
void disableServoOutputs() {}
void enableServoOutputs() {}

bool servoBackendBringUp() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    ledcAttach(SERVO_GPIO_PINS[i], kServoPwmHz, kLedcResolutionBits);
  }

  return true;
}

bool servoPwmDriverState(ServoPwmDriverState& state) {
  state = {true, 0, 0, 0};
  return true;
}

bool setUpServoPwmDriver() {
  return servoBackendBringUp();
}

#else

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "hardware/i2c_health.h"

namespace {

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

constexpr uint16_t kServoPwmHz = 50;

constexpr uint8_t kPcaMode1Reg = 0x00;
constexpr uint8_t kPcaMode2Reg = 0x01;
// Awake, on the internal clock, with register auto-increment on: setPWM
// writes four registers in one transfer and needs it. EXTCLK is sticky (only
// a power cycle clears it), so a corrupted one keeps the chip down, visibly.
constexpr uint8_t kPcaMode1Mask = MODE1_SLEEP | MODE1_AI | MODE1_EXTCLK;
constexpr uint8_t kPcaMode1Servo = MODE1_AI;
// Totem-pole outputs, not inverted: what a servo signal line needs. Any
// other value leaves the servos without a usable pulse while every write
// still succeeds.
constexpr uint8_t kPcaMode2Servo = MODE2_OUTDRV;
// What setPWMFreq(kServoPwmHz) programs, by the Adafruit driver's own
// formula. Computed, not read back: a readback after a lost write would
// bless the wrong value. A chip that lost power comes back with a
// different default.
constexpr uint8_t kPcaPrescaleServo =
  static_cast<uint8_t>(FREQUENCY_OSCILLATOR / (kServoPwmHz * 4096.0) + 0.5 - 1);

// Reads `count` registers from `reg` in one transfer; relies on auto-increment.
bool readPcaRegisters(uint8_t reg, uint8_t* values, uint8_t count) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  // Without a STOP nothing is sent yet; a failure shows up in requestFrom.
  Wire.endTransmission(false);

  if (Wire.requestFrom(PCA9685_ADDRESS, count) != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    values[i] = static_cast<uint8_t>(Wire.read());
  }

  return true;
}

// Output mode and servo frequency, shared by boot and recovery. The chip
// must be awake first: setPWMFreq() restores the sleep bit it finds. False
// when the MODE2 write failed: the frequency is then skipped, since on a
// dead bus each of its transfers would wait out a timeout too.
bool writeServoSetup() {
  // The Adafruit driver never writes MODE2, so a value corrupted by a bus
  // glitch would outlive every restart. The chip keeps its registers across
  // an ESP32 reset: only its own power cycle clears them.
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(kPcaMode2Reg);
  Wire.write(kPcaMode2Servo);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  pwm.setPWMFreq(kServoPwmHz);
  return true;
}

}  // namespace

// PCA9685 LEDn_OFF full-off bit. Do not use (4096, 0) — that is full-on HIGH.
const uint16_t kServoOffValue = 4096;

uint16_t angleToPulse(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);

  const float pulseUs =
    SERVO_MIN_US +
    (angle / 180.0f) *
    (SERVO_MAX_US - SERVO_MIN_US);

  const float counts =
    pulseUs * 4096.0f / 20000.0f + 0.5f;

  return (uint16_t)constrain(counts, 0.0f, 4095.0f);
}

// Skipped while the PCA9685 is unusable, so a dead bus does not stall the
// loop. The first failed write takes the chip down at once, with its outputs
// off, even inside a blocking move that never reaches the health check; the
// recovery rewrites every channel once the chip is back.
void writeChannel(uint8_t channel, uint16_t value) {
  if (!i2cServoDriverUp()) {
    return;
  }

  if (pwm.setPWM(channel, 0, value) != 0) {
    i2cNoteServoDriverFault("write failed");
  }
}

// Writes one channel and reads it back: an acknowledged write can still land
// corrupted on a bus that just glitched.
bool writeChannelVerified(uint8_t channel, uint16_t value) {
  if (pwm.setPWM(channel, 0, value) != 0) {
    return false;
  }

  uint8_t got[4] = {};
  const uint8_t want[4] = {
    0, 0, static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)
  };

  return readPcaRegisters(PCA9685_LED0_ON_L + 4 * channel, got, 4) &&
         memcmp(got, want, sizeof(want)) == 0;
}

void initServoOutputPin() {
  if (!PCA9685_OE_WIRED) {
    return;
  }

  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, HIGH);
}

void disableServoOutputs() {
  if (!PCA9685_OE_WIRED) {
    return;
  }

  digitalWrite(PCA9685_OE_PIN, HIGH);
}

void enableServoOutputs() {
  if (!PCA9685_OE_WIRED) {
    return;
  }

  digitalWrite(PCA9685_OE_PIN, LOW);
}

bool servoBackendBringUp() {
  // Not pwm.begin() here as well: callers that need the 1 kHz-then-50 Hz
  // window avoided use setUpServoPwmDriver() instead.
  pwm.begin();
  return writeServoSetup();
}

bool servoPwmDriverState(ServoPwmDriverState& state) {
  state = {};

  // MODE1 and MODE2 are adjacent, so one read returns both. With
  // auto-increment off the second byte repeats MODE1 and fails the MODE2
  // check.
  uint8_t modes[2] = {};
  state.answered = readPcaRegisters(kPcaMode1Reg, modes, 2);

  if (!state.answered) {
    return false;
  }

  state.mode1 = modes[0];
  state.mode2 = modes[1];
  state.prescale = pwm.readPrescale();

  return (state.mode1 & kPcaMode1Mask) == kPcaMode1Servo &&
         state.mode2 == kPcaMode2Servo &&
         state.prescale == kPcaPrescaleServo;
}

bool setUpServoPwmDriver() {
  disableServoOutputs();

  // Not pwm.begin(): it programs 1 kHz before writeServoSetup() sets 50 Hz,
  // and on a board with OE not wired the live servos would get those pulses.
  // reset() wakes the chip, which is all begin() adds here.
  pwm.reset();

  if (!writeServoSetup()) {
    return false;
  }

  ServoPwmDriverState state;
  return servoPwmDriverState(state);
}

#endif  // TE_BOARD_ESP32DEV
