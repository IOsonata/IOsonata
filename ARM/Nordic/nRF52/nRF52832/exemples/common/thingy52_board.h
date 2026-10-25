/** Thingy:52 production PCA20020 (v1.x) board helpers. SPDX-License-Identifier: MIT */
#ifndef __THINGY52_BOARD_H__
#define __THINGY52_BOARD_H__

#include "miscdev/ioexp_sx1509.h"

#define THINGY52_I2C_DEVNO 0
#define THINGY52_I2C_PORT 0
#define THINGY52_I2C_SDA_PIN 7
#define THINGY52_I2C_SCL_PIN 8
// LIS2DH12 uses the separate external I2C bus and the always-on supply.
#define THINGY52_ACCEL_I2C_DEVNO 1
#define THINGY52_ACCEL_I2C_PORT 0
#define THINGY52_ACCEL_I2C_SDA_PIN 14
#define THINGY52_ACCEL_I2C_SCL_PIN 15
#define THINGY52_VDD_PORT 0
#define THINGY52_VDD_PIN 30
#define THINGY52_SX_RESET_PORT 0
#define THINGY52_SX_RESET_PIN 16
#define THINGY52_BUTTON_PORT 0
#define THINGY52_BUTTON_PIN 11
#define THINGY52_COLOR_IRQ_PORT 0
#define THINGY52_COLOR_IRQ_PIN 31
#define THINGY52_GAS_IRQ_PORT 0
#define THINGY52_GAS_IRQ_PIN 22
#define THINGY52_SPEAKER_POWER_PORT 0
#define THINGY52_SPEAKER_POWER_PIN 29

#define THINGY52_SX_BATTERY_MONITOR 4
#define THINGY52_SX_LIGHTWELL_GREEN 5
#define THINGY52_SX_LIGHTWELL_BLUE 6
#define THINGY52_SX_LIGHTWELL_RED 7
#define THINGY52_SX_MOTION_POWER 8
#define THINGY52_SX_MIC_POWER 9
#define THINGY52_SX_GAS_POWER 10
#define THINGY52_SX_GAS_RESET 11
#define THINGY52_SX_GAS_WAKE 12
#define THINGY52_SX_COLOR_RED 13
#define THINGY52_SX_COLOR_GREEN 14
#define THINGY52_SX_COLOR_BLUE 15

// Startup ownership: call before bus/driver initialization, not while sampling.
void Thingy52PowerInit();
// Explicitly resets the expander. LEDs and switched sensor supplies start off;
// expansion pins 0..3 and battery monitor pin 4 remain inputs.
bool Thingy52ExpanderInit(IoExpSx1509 &Expander, DeviceIntrf &Bus);
// Enable before MPU9250 initialization; stop bus access before disabling.
bool Thingy52MotionPower(IoExpSx1509 &Expander, bool Enable);
bool Thingy52GasPower(IoExpSx1509 &Expander, bool Enable);
// Install with GasCcs811::WakeControl(Thingy52GasWake, &Expander).
bool Thingy52GasWake(bool Active, void *pContext);
bool Thingy52Lightwell(IoExpSx1509 &Expander, bool Red, bool Green, bool Blue);
bool Thingy52ColorLight(IoExpSx1509 &Expander, bool Red, bool Green, bool Blue);
bool Thingy52ButtonPressed();
bool Thingy52ColorIrqActive();
bool Thingy52GasIrqActive();

#endif
