#pragma once

#include <stdint.h>

// The hardware boundary servo_wrapper.cpp writes through: either the
// PCA9685 I2C PWM driver (stock C3 board) or, with TE_BOARD_ESP32DEV, the
// ESP32's own LEDC peripheral. Everything above this line (pose, slew,
// NVS persistence) is backend-agnostic.

// Backend-native pulse value for one channel: a 12-bit on-time count on the
// PCA9685 (0-4095), a 16-bit LEDC duty on ESP32DEV (0-65535).
// kServoOffValue writes no pulse on that channel.
extern const uint16_t kServoOffValue;

uint16_t angleToPulse(float angle);
void writeChannel(uint8_t channel, uint16_t value);
// Writes one channel and reads it back where the backend can (PCA9685);
// always true on ESP32DEV, where the ESP32 drives the pin itself.
bool writeChannelVerified(uint8_t channel, uint16_t value);

void initServoOutputPin();
void disableServoOutputs();
void enableServoOutputs();

// Wakes the driver and programs 50 Hz, outputs off. True when it is usable.
bool servoBackendBringUp();

struct ServoPwmDriverState {
  bool answered;
  uint8_t mode1;
  uint8_t mode2;
  uint8_t prescale;
};
// Reads back the driver's setup. True when it is awake with the servo
// output mode and frequency (always true on ESP32DEV: nothing to read back).
bool servoPwmDriverState(ServoPwmDriverState& state);
// Redoes servoBackendBringUp() after a fault.
bool setUpServoPwmDriver();
