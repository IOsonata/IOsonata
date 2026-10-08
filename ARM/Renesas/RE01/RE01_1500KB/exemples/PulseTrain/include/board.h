// SPDX-License-Identifier: MIT
// Sample MCU pin map; adjust to the application wiring.
#ifndef __BOARD_H__
#define __BOARD_H__

#define PULSE0_PORT		0
#define PULSE0_PIN		9
#define PULSE0_PINOP		IOPINOP_GPIO
#define PULSE1_PORT		0
#define PULSE1_PIN		8
#define PULSE1_PINOP		IOPINOP_GPIO
#define PULSE2_PORT		0
#define PULSE2_PIN		7
#define PULSE2_PINOP		IOPINOP_GPIO
#define PULSE_TRAIN_PINS_MAP { \
	{PULSE0_PORT, PULSE0_PIN, PULSE0_PINOP}, \
	{PULSE1_PORT, PULSE1_PIN, PULSE1_PINOP}, \
	{PULSE2_PORT, PULSE2_PIN, PULSE2_PINOP}, \
}

#endif
