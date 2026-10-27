/**-------------------------------------------------------------------------
@file thingy91x_board.h

@brief Thingy:91 X application-level power and GPIO integration helpers.

@license
MIT License

Copyright (c) 2026 I-SYST inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------*/
#ifndef __THINGY91X_BOARD_H__
#define __THINGY91X_BOARD_H__

#include "pwrmgnt/pm_npm1300.h"

#define THINGY91X_PMIC_HOST_IRQ_PORT			0
#define THINGY91X_PMIC_HOST_IRQ_PIN			2
#define THINGY91X_PMIC_IRQ_GPIO				3
#define THINGY91X_POWER_SWITCH_GPIO			2
#define THINGY91X_NPM6001_READY_GPIO			4
#define THINGY91X_NPM6001_ENABLE_LDSW			0
#define THINGY91X_SENSOR_ENABLE_LDSW			1
#define THINGY91X_BUCK2_INDEX				1
#define THINGY91X_EXP_BOARD_ENABLE_PORT			0
#define THINGY91X_EXP_BOARD_ENABLE_PIN			3
#define THINGY91X_BUTTON1_PORT				0
#define THINGY91X_BUTTON1_PIN				26
#define THINGY91X_LED_RED_PORT				0
#define THINGY91X_LED_RED_PIN				29
#define THINGY91X_LED_GREEN_PORT				0
#define THINGY91X_LED_GREEN_PIN				31
#define THINGY91X_LED_BLUE_PORT				0
#define THINGY91X_LED_BLUE_PIN				30

bool Thingy91XPmicConfigure(PmnPM1300 &Pmic);
int Thingy91XPmicInterruptEnable(PmnPM1300 &Pmic, int Priority);
void Thingy91XPmicInterruptDisable(int IntNo);
bool Thingy91XNpm6001Power(PmnPM1300 &Pmic, bool Enable);
bool Thingy91XNpm6001Ready(PmnPM1300 &Pmic, bool &Ready);
bool Thingy91XSensorPower(PmnPM1300 &Pmic, bool Enable);
void Thingy91XExpansionPowerInit(bool Enable);
void Thingy91XExpansionPower(bool Enable);
void Thingy91XUserIoInit();
void Thingy91XLedSet(bool Red, bool Green, bool Blue);
bool Thingy91XButtonPressed();

#endif
