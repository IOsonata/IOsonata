/**-------------------------------------------------------------------------
@file thingy91x_board.cpp

@brief Thingy:91 X application-level power and GPIO integration helpers.

@license
MIT License

Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "iopinctrl.h"

#include "thingy91x_board.h"

static void Thingy91XPmicIrqHandler(int IntNo, void *pContext)
{
	PmnPM1300 *pmic;

	(void)IntNo;
	pmic = (PmnPM1300 *)pContext;
	if (pmic != NULL)
		pmic->IrqHandler();
}

bool Thingy91XPmicConfigure(PmnPM1300 &Pmic)
{
	if (!Pmic.SetGpioPull(THINGY91X_POWER_SWITCH_GPIO, false, false) ||
		!Pmic.SetGpioMode(THINGY91X_POWER_SWITCH_GPIO,
			NPM1300_GPIOS_GPIOMODE_GPIINPUT) ||
		!Pmic.SetBuckEnableGpio(THINGY91X_BUCK2_INDEX,
			THINGY91X_POWER_SWITCH_GPIO, false) ||
		!Pmic.SetLoadSwitchMode(THINGY91X_NPM6001_ENABLE_LDSW) ||
		!Pmic.SetLoadSwitchMode(THINGY91X_SENSOR_ENABLE_LDSW) ||
		!Pmic.SetGpioPull(THINGY91X_NPM6001_READY_GPIO, false, false) ||
		!Pmic.SetGpioMode(THINGY91X_NPM6001_READY_GPIO,
			NPM1300_GPIOS_GPIOMODE_GPIINPUT) ||
		!Pmic.SetGpioMode(THINGY91X_PMIC_IRQ_GPIO,
			NPM1300_GPIOS_GPIOMODE_GPOIRQ))
	{
		return false;
	}

	IOPinConfig(THINGY91X_PMIC_HOST_IRQ_PORT, THINGY91X_PMIC_HOST_IRQ_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	return true;
}

int Thingy91XPmicInterruptEnable(PmnPM1300 &Pmic, int Priority)
{
	int intNo;

	if (!Pmic.SetGpioMode(THINGY91X_PMIC_IRQ_GPIO,
		NPM1300_GPIOS_GPIOMODE_GPOIRQ))
	{
		return -1;
	}

	IOPinConfig(THINGY91X_PMIC_HOST_IRQ_PORT, THINGY91X_PMIC_HOST_IRQ_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	intNo = IOPinAllocateInterrupt(Priority, THINGY91X_PMIC_HOST_IRQ_PORT,
		THINGY91X_PMIC_HOST_IRQ_PIN, IOPINSENSE_HIGH_TRANSITION,
		Thingy91XPmicIrqHandler, &Pmic);
	return intNo;
}

void Thingy91XPmicInterruptDisable(int IntNo)
{
	if (IntNo >= 0)
		IOPinDisableInterrupt(IntNo);
}

bool Thingy91XNpm6001Power(PmnPM1300 &Pmic, bool Enable)
{
	if (!Pmic.SetLoadSwitchMode(THINGY91X_NPM6001_ENABLE_LDSW))
		return false;
	return Pmic.SetLoadSwitchEnabled(THINGY91X_NPM6001_ENABLE_LDSW, Enable);
}

bool Thingy91XNpm6001Ready(PmnPM1300 &Pmic, bool &Ready)
{
	return Pmic.GpioState(THINGY91X_NPM6001_READY_GPIO, Ready);
}

bool Thingy91XSensorPower(PmnPM1300 &Pmic, bool Enable)
{
	if (!Pmic.SetLoadSwitchMode(THINGY91X_SENSOR_ENABLE_LDSW))
		return false;
	return Pmic.SetLoadSwitchEnabled(THINGY91X_SENSOR_ENABLE_LDSW, Enable);
}

void Thingy91XExpansionPowerInit(bool Enable)
{
	if (Enable)
		IOPinSet(THINGY91X_EXP_BOARD_ENABLE_PORT, THINGY91X_EXP_BOARD_ENABLE_PIN);
	else
		IOPinClear(THINGY91X_EXP_BOARD_ENABLE_PORT, THINGY91X_EXP_BOARD_ENABLE_PIN);

	IOPinConfig(THINGY91X_EXP_BOARD_ENABLE_PORT, THINGY91X_EXP_BOARD_ENABLE_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

void Thingy91XExpansionPower(bool Enable)
{
	if (Enable)
		IOPinSet(THINGY91X_EXP_BOARD_ENABLE_PORT, THINGY91X_EXP_BOARD_ENABLE_PIN);
	else
		IOPinClear(THINGY91X_EXP_BOARD_ENABLE_PORT, THINGY91X_EXP_BOARD_ENABLE_PIN);
}

void Thingy91XUserIoInit()
{
	IOPinClear(THINGY91X_LED_RED_PORT, THINGY91X_LED_RED_PIN);
	IOPinClear(THINGY91X_LED_GREEN_PORT, THINGY91X_LED_GREEN_PIN);
	IOPinClear(THINGY91X_LED_BLUE_PORT, THINGY91X_LED_BLUE_PIN);

	IOPinConfig(THINGY91X_LED_RED_PORT, THINGY91X_LED_RED_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91X_LED_GREEN_PORT, THINGY91X_LED_GREEN_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91X_LED_BLUE_PORT, THINGY91X_LED_BLUE_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(THINGY91X_BUTTON1_PORT, THINGY91X_BUTTON1_PIN,
		IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
}

void Thingy91XLedSet(bool Red, bool Green, bool Blue)
{
	if (Red)
		IOPinSet(THINGY91X_LED_RED_PORT, THINGY91X_LED_RED_PIN);
	else
		IOPinClear(THINGY91X_LED_RED_PORT, THINGY91X_LED_RED_PIN);

	if (Green)
		IOPinSet(THINGY91X_LED_GREEN_PORT, THINGY91X_LED_GREEN_PIN);
	else
		IOPinClear(THINGY91X_LED_GREEN_PORT, THINGY91X_LED_GREEN_PIN);

	if (Blue)
		IOPinSet(THINGY91X_LED_BLUE_PORT, THINGY91X_LED_BLUE_PIN);
	else
		IOPinClear(THINGY91X_LED_BLUE_PORT, THINGY91X_LED_BLUE_PIN);
}

bool Thingy91XButtonPressed()
{
	return IOPinRead(THINGY91X_BUTTON1_PORT, THINGY91X_BUTTON1_PIN) == 0;
}
