/**-------------------------------------------------------------------------
@file	adc_fw.cpp

@brief	STM32 ADC1 driver test firmware, STM32L4, L4+ and G4

Runs the production ADC1 driver, ARM/ST/src/adc_stm32.cpp, in the emulator
of run_adc.py, against its model of the ADC registers. usDelay tells the
model the time that passed. Results go to the model through the
test registers at 0x500FF000: 0 fault to inject, 1 check result, 2 end with
the number of checks, 3 time passed in usec, 4 core clock in Hz.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "stm32.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "adc_stm32.h"

#if defined(ADC123_COMMON)
#define TEST_COMMON		ADC123_COMMON
#elif defined(ADC12_COMMON)
#define TEST_COMMON		ADC12_COMMON
#else
#define TEST_COMMON		ADC1_COMMON
#endif

#ifdef ADC_CCR_VSENSESEL
#define TEST_CCR_TEMP	ADC_CCR_VSENSESEL
#define TEST_CCR_VBAT	ADC_CCR_VBATSEL
#define TEST_RCC_EN		RCC_AHB2ENR_ADC12EN
#define TEST_CKMODE_140	3		// 70 MHz is above the 60 MHz of the G4

// 16 inputs for one scan, the inputs of the family without pin are refused
static const int s_ScanInput[ADC_STM32_MAX_CHAN] = { 18, 17, 16, 15, 14, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2 };
static const int s_NoInput[] = { 0, 13 };

// Port and pin of the inputs 0 to 16 from the ST pin data, -1 without pin
static const int s_Pin[17][2] = {
	{ -1, 0 }, { IOPORTA, 0 }, { IOPORTA, 1 }, { IOPORTA, 2 }, { IOPORTA, 3 }, { IOPORTB, 14 },
	{ IOPORTC, 0 }, { IOPORTC, 1 }, { IOPORTC, 2 }, { IOPORTC, 3 }, { IOPORTF, 0 }, { IOPORTB, 12 },
	{ IOPORTB, 1 }, { -1, 0 }, { IOPORTB, 11 }, { IOPORTB, 0 }, { -1, 0 },
};
#else
#define TEST_CCR_TEMP	ADC_CCR_TSEN
#define TEST_CCR_VBAT	ADC_CCR_VBATEN
#define TEST_RCC_EN		RCC_AHB2ENR_ADCEN
#define TEST_CKMODE_140	2

static const int s_ScanInput[ADC_STM32_MAX_CHAN] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };
static const int s_NoInput[] = { ADC_STM32_MAX_INPUT + 1 };

static const int s_Pin[17][2] = {
	{ -1, 0 }, { IOPORTC, 0 }, { IOPORTC, 1 }, { IOPORTC, 2 }, { IOPORTC, 3 }, { IOPORTA, 0 },
	{ IOPORTA, 1 }, { IOPORTA, 2 }, { IOPORTA, 3 }, { IOPORTA, 4 }, { IOPORTA, 5 }, { IOPORTA, 6 },
	{ IOPORTA, 7 }, { IOPORTC, 4 }, { IOPORTC, 5 }, { IOPORTB, 0 }, { IOPORTB, 1 },
};
#endif

#define TEST_FAULT_NO_EOC		1	// Conversions do not end
#define TEST_FAULT_OVR			2	// Overrun at the end of conversion
#define TEST_FAULT_NO_STOP		3	// Conversions and stop do not end
#define TEST_FAULT_NO_CAL		4	// Calibration does not end
#define TEST_FAULT_NO_RDY		5	// ADRDY never set

static volatile uint32_t *const s_Io = (volatile uint32_t *)0x500FF000;
static unsigned s_Checks = 0;
static unsigned s_Configured = 0;
static unsigned s_Analog = 0;
static int s_CfgPort = -1;
static int s_CfgPin = -1;
static int s_AnaPort = -1;
static int s_AnaPin = -1;
static uint32_t s_ClockHz = 80000000;

static const uint8_t s_AhbShift[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };

extern "C" uint32_t SystemHFClockGet(void)
{
	return s_ClockHz >> s_AhbShift[(RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];
}

extern "C" void usDelay(uint32_t Us)
{
	s_Io[3] = Us;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if (PinOp == IOPINOP_GPIO && Dir == IOPINDIR_INPUT && Resistor == IOPINRES_NONE && Type == IOPINTYPE_NORMAL)
	{
		s_Configured++;
		s_CfgPort = PortNo;
		s_CfgPin = PinNo;
	}
}

extern "C" void IOPinDisable(int PortNo, int PinNo)
{
	s_Analog++;
	s_AnaPort = PortNo;
	s_AnaPin = PinNo;
}

extern "C" void *memset(void *p, int c, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;

	while (n--)
	{
		*d++ = c;
	}

	return p;
}

extern "C" void *memcpy(void *p, const void *q, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;
	const volatile unsigned char *s = (const volatile unsigned char *)q;

	while (n--)
	{
		*d++ = *s++;
	}

	return p;
}

void operator delete(void *) noexcept
{
}

void operator delete(void *, size_t) noexcept
{
}

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static void SetClock(uint32_t Hz)
{
	s_ClockHz = Hz;
	s_Io[4] = Hz;
}

static bool Near(float V, float Expect, float Tol)
{
	return V > Expect - Tol && V < Expect + Tol;
}

// Voltages of the model: internal inputs, 1, 5 and 14, 0.1 V per input
// number for the others
static float Volt(int Input)
{
	return Input == ADC_STM32_INPUT_VREFINT ? 1.212f : Input == ADC_STM32_INPUT_TEMP ? 0.8f :
		   Input == ADC_STM32_INPUT_VBAT ? 3.0f : Input == 1 ? 0.5f : Input == 5 ? 1.5f :
		   Input == 14 ? 2.0f : 0.1f * Input;
}

static uint32_t Smp(int Input)
{
	return Input < 10 ? (ADC1->SMPR1 >> (Input * 3)) & 7 : (ADC1->SMPR2 >> ((Input - 10) * 3)) & 7;
}

// GPIOx_ASCR of the port is Mask, on the parts that have the register
static bool AscrIs(int Port, uint32_t Mask)
{
#ifdef GPIO_ASCR_ASC0
	return ((GPIO_TypeDef *)(GPIOA_BASE + Port * (GPIOB_BASE - GPIOA_BASE)))->ASCR == Mask;
#else
	(void)Port;
	(void)Mask;
	return true;
#endif
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *s = &_sidata;

	for (uint32_t *d = &_sdata; d < &_edata;)
	{
		*d++ = *s++;
	}
	for (uint32_t *d = &_sbss; d < &_ebss;)
	{
		*d++ = 0;
	}

	SetClock(80000000);

	AdcStm32 adc, other;
	AdcRefVolt_t ref = { ADC_REFVOLT_TYPE_SUPPLY, 3.3f, -1 };
	AdcCfg_t cfg = { ADC_CONV_MODE_SINGLE, &ref, 1, 0, 12, 0, 0, false, 6, nullptr };
	const AdcChanCfg_t vref = { 0, 0, ADC_CHAN_TYPE_SINGLE_ENDED, 0x100, 0, false, { ADC_STM32_INPUT_VREFINT, ADC_PIN_CONN_NONE },
							   { -1, ADC_PIN_CONN_NONE }, 0, nullptr };
	AdcChanCfg_t set[4] = { vref, vref, vref, vref };
	AdcData_t d;
	AdcData_t r[4];

	set[1].Chan = 1;
	set[1].PinP.PinNo = ADC_STM32_INPUT_TEMP;
	set[2].Chan = 2;
	set[2].PinP.PinNo = ADC_STM32_INPUT_VBAT;
	set[3].Chan = 3;
	set[3].PinP.PinNo = 14;		// PC5

	// Nothing works before Init
	Check(!adc.Read(0, &d) && !adc.StartConversion() && !adc.OpenChannel(set, 1) && !adc.Valid());

	// Init refuses the modes it does not implement
	cfg.bInterrupt = true;
	Check(!adc.Init(cfg));
	cfg.bInterrupt = false;
	cfg.Mode = ADC_CONV_MODE_CONTINUOUS;
	Check(!adc.Init(cfg));
	cfg.Mode = ADC_CONV_MODE_SINGLE;
	cfg.OvrSample = 3;
	Check(!adc.Init(cfg));
	cfg.OvrSample = 512;
	Check(!adc.Init(cfg));
	cfg.OvrSample = -1;
	Check(!adc.Init(cfg));
	cfg.OvrSample = 0;
	ref.Type = ADC_REFVOLT_TYPE_INTERNAL;
	Check(!adc.Init(cfg));
	ref.Type = ADC_REFVOLT_TYPE_SUPPLY;
	ref.Voltage = 0;
	Check(!adc.Init(cfg));
	ref.Voltage = 3.3f;
	cfg.NbRefVolt = 2;
	Check(!adc.Init(cfg));
	cfg.NbRefVolt = 1;
	Check(!adc.Init(cfg, nullptr, (DeviceIntrf *)&ref));
	SetClock(200000);
	Check(!adc.Init(cfg));		// 100 kHz ADC clock
	SetClock(80000000);

	Check(adc.Init(cfg) && adc.Valid());
	Check(((TEST_COMMON->CCR & ADC_CCR_CKMODE) >> ADC_CCR_CKMODE_Pos) == 2 && ADC1->CFGR2 == 0);
	Check(!other.Init(cfg) && adc.Valid());
	Check(!adc.StartConversion() && !adc.Read(0, &d));

	// Four inputs: VREFINT, temperature, VBAT, PC5
	Check(adc.OpenChannel(set, 4) && s_Configured == 1 && s_Analog == 1 && AscrIs(IOPORTC, 1U << 5));
	Check((TEST_COMMON->CCR & (ADC_CCR_VREFEN | TEST_CCR_TEMP | TEST_CCR_VBAT)) ==
		  (ADC_CCR_VREFEN | TEST_CCR_TEMP | TEST_CCR_VBAT));
	// 40 MHz ADC clock: 4 us, 5 us, 12 us minimum, AcqTime 0
	Check(Smp(ADC_STM32_INPUT_VREFINT) == 6 && Smp(ADC_STM32_INPUT_TEMP) == 6 && Smp(ADC_STM32_INPUT_VBAT) == 7 &&
		  Smp(14) == 0);
	Check(adc.Read(0, &d) && d.Chan == 0 && d.Timestamp == 0 && Near(d.Data, 1.212f, 0.002f));
	Check(!adc.UpdateData());

	usDelay(2000);
	Check(adc.StartConversion() && adc.UpdateData());
	Check(adc.Read(r, 4) == 4 && !adc.UpdateData());
	Check(r[0].Chan == 0 && Near(r[0].Data, 1.212f, 0.002f) && r[1].Chan == 1 && Near(r[1].Data, 0.8f, 0.002f));
	Check(r[2].Chan == 2 && Near(r[2].Data, 3.0f, 0.005f) && r[3].Chan == 3 && Near(r[3].Data, 2.0f, 0.002f));
	usDelay(5000);
	Check(adc.Read(3, &d) && d.Chan == 3 && Near(d.Data, 2.0f, 0.002f));

	// Resolution, a change drops the results not read
	Check(adc.StartConversion() && adc.UpdateData() && adc.Resolution(10) == 10 && !adc.UpdateData() &&
		  adc.Read(r, 4) == 0);
	Check(adc.Resolution(9) == 10 && adc.Resolution() == 10 &&
		  ((ADC1->CFGR & ADC_CFGR_RES) >> ADC_CFGR_RES_Pos) == 1);
	Check(adc.Read(3, &d) && Near(d.Data, 2.0f, 0.005f));
	// 2.0 V is code 38 of 63 at 6 bits
	Check(adc.Resolution(1) == 6 && adc.Read(3, &d) && Near(d.Data, 38 * 3.3f / 63, 0.0005f));
	Check(adc.Resolution(0) == 12 && adc.Resolution(99) == 12 && (ADC1->CFGR & ADC_CFGR_RES) == 0);

	// OpenChannel refusals keep the channels open
	AdcChanCfg_t bad = set[3];
	bad.Chan = 4;
	bad.PinP.PinNo = 5;		// PA0
	bad.AcqTime = 17;		// More than 640.5 clocks at 40 MHz
	Check(!adc.OpenChannel(&bad, 1));
	bad.AcqTime = 10;
	bad.Chan = ADC_STM32_MAX_CHAN;
	Check(!adc.OpenChannel(&bad, 1));
	bad.Chan = 4;
	bad.PinP.PinNo = ADC_STM32_INPUT_VBAT + 1;
	Check(!adc.OpenChannel(&bad, 1));
	bad.PinP.PinNo = -1;
	Check(!adc.OpenChannel(&bad, 1));
	for (int in : s_NoInput)
	{
		bad.PinP.PinNo = in;
		Check(!adc.OpenChannel(&bad, 1));
	}
	bad.PinP.PinNo = 5;
	bad.Type = ADC_CHAN_TYPE_DIFFERENTIAL;
	Check(!adc.OpenChannel(&bad, 1));
	bad.Type = ADC_CHAN_TYPE_SINGLE_ENDED;
	bad.Gain = 2;
	Check(!adc.OpenChannel(&bad, 1));
	bad.Gain = 0x100;
	bad.BurstMode = true;
	Check(!adc.OpenChannel(&bad, 1));
	bad.BurstMode = false;
	bad.RefVoltIdx = 1;
	Check(!adc.OpenChannel(&bad, 1));
	bad.RefVoltIdx = 0;
	bad.AcqTime = -1;
	Check(!adc.OpenChannel(&bad, 1));
	bad.AcqTime = 10;
	bad.PinP.Conn = ADC_PIN_CONN_PULLUP;
	Check(!adc.OpenChannel(&bad, 1));
	bad.PinP.Conn = ADC_PIN_CONN_NONE;
	bad.FifoMemSize = 16;
	Check(!adc.OpenChannel(&bad, 1));
	bad.FifoMemSize = 0;
	bad.PinP.PinNo = 14;	// Open on logical channel 3
	Check(!adc.OpenChannel(&bad, 1));
	bad.PinP.PinNo = 5;

	AdcChanCfg_t two[2] = { bad, bad };

	Check(!adc.OpenChannel(two, 2));		// Same logical channel twice
	two[1].Chan = 5;
	Check(!adc.OpenChannel(two, 2));		// Same input twice
	Check(s_Configured == 1 && adc.Read(3, &d) && Near(d.Data, 2.0f, 0.002f));

	// 10 us at 40 MHz is 400 clocks: 640.5
	Check(adc.OpenChannel(&bad, 1) && Smp(5) == 7 && s_Configured == 2 && AscrIs(IOPORTA, 1U));
	Check(adc.Read(4, &d) && d.Chan == 4 && Near(d.Data, 1.5f, 0.002f));

	// Logical channel 3 moves from PC5 to PC0, PC5 moves to logical 6
	AdcChanCfg_t mv[2] = { set[3], set[3] };
	mv[0].PinP.PinNo = 1;
	mv[1].Chan = 6;
	Check(adc.OpenChannel(mv, 2) && AscrIs(IOPORTC, (1U << 5) | 1U));
	Check(adc.Read(3, &d) && Near(d.Data, 0.5f, 0.002f) && adc.Read(6, &d) && Near(d.Data, 2.0f, 0.002f));
	adc.CloseChannel(6);
	Check(AscrIs(IOPORTC, 1U) && !adc.Read(6, &d));
	// Logical channel 3 back on PC5 releases PC0
	Check(adc.OpenChannel(&set[3], 1) && AscrIs(IOPORTC, 1U << 5) && adc.Read(3, &d) && Near(d.Data, 2.0f, 0.002f));

	// Closing the temperature channel turns the sensor off
	adc.CloseChannel(1);
	Check(!(TEST_COMMON->CCR & TEST_CCR_TEMP) && (TEST_COMMON->CCR & ADC_CCR_VREFEN) && !adc.Read(1, &d));
	adc.CloseChannel(4);
	Check(AscrIs(IOPORTA, 0));

	// A scan of the open channels in logical order
	Check(adc.StartConversion() && adc.Read(r, 4) == 3 && r[0].Chan == 0 && r[1].Chan == 2 && r[2].Chan == 3);

	// Faults
	s_Io[0] = TEST_FAULT_NO_EOC;
	Check(!adc.Read(0, &d) && adc.Valid() && !(ADC1->CR & ADC_CR_ADSTART));
	s_Io[0] = 0;
	Check(adc.Read(0, &d) && Near(d.Data, 1.212f, 0.002f));
	s_Io[0] = TEST_FAULT_OVR;
	Check(!adc.Read(0, &d) && !adc.StartConversion() && !adc.UpdateData());
	s_Io[0] = 0;
	Check(adc.Read(0, &d));
	s_Io[0] = TEST_FAULT_NO_STOP;
	Check(!adc.Read(0, &d) && !adc.Valid());
	adc.PowerOff();
	Check(!adc.Valid() && !(RCC->AHB2ENR & TEST_RCC_EN) && AscrIs(IOPORTC, 0));
	s_Io[0] = 0;

	// Oversampling 16 times, shift 4
	cfg.OvrSample = 16;
	Check(adc.Init(cfg) && ADC1->CFGR2 == (ADC_CFGR2_ROVSE | (3U << ADC_CFGR2_OVSR_Pos) | (4U << ADC_CFGR2_OVSS_Pos)));
	Check(adc.OpenChannel(&set[3], 1) && adc.Read(3, &d) && Near(d.Data, 2.0f, 0.002f));
	cfg.OvrSample = 1;

	adc.Disable();
	Check(!(ADC1->CR & ADC_CR_ADEN) && adc.Enable() && (ADC1->CR & ADC_CR_ADEN));
	Check(adc.Calibrate() && (ADC1->CR & ADC_CR_ADEN) && adc.Read(3, &d) && Near(d.Data, 2.0f, 0.002f));
	adc.Reset();
	Check(adc.Valid() && adc.Read(3, &d));
	adc.PowerOff();
	Check(!adc.Valid() && !adc.Read(3, &d));

	// Reference and sensor buffers start with the ADC
	Check(adc.Init(cfg) && adc.OpenChannel(set, 2));
	adc.Disable();
	Check(adc.Enable() && adc.Read(1, &d) && Near(d.Data, 0.8f, 0.002f));
	adc.Disable();
	Check(adc.Enable() && adc.Read(0, &d) && Near(d.Data, 1.212f, 0.002f));
	adc.PowerOff();

	// The reference alone
	Check(adc.Init(cfg) && adc.OpenChannel(set, 1) && adc.Read(0, &d) && Near(d.Data, 1.212f, 0.002f));
	adc.PowerOff();

	// 16 ranks
	AdcChanCfg_t all[ADC_STM32_MAX_CHAN];
	AdcData_t res[ADC_STM32_MAX_CHAN];

	for (int i = 0; i < ADC_STM32_MAX_CHAN; i++)
	{
		all[i] = set[3];
		all[i].Chan = i;
		all[i].PinP.PinNo = s_ScanInput[i];
	}
	Check(adc.Init(cfg) && adc.OpenChannel(all, ADC_STM32_MAX_CHAN) && adc.StartConversion() &&
		  adc.Read(res, ADC_STM32_MAX_CHAN) == ADC_STM32_MAX_CHAN);

	bool ok = true;

	for (int i = 0; i < ADC_STM32_MAX_CHAN; i++)
	{
		ok = ok && res[i].Chan == i && Near(res[i].Data, Volt(s_ScanInput[i]), 0.004f);
	}
	Check(ok);
	adc.PowerOff();

	// Pin of each input, configured then put in analog mode
	Check(adc.Init(cfg));
	ok = true;
	for (int in = 0; in < 17; in++)
	{
		AdcChanCfg_t one = set[3];
		bool internal = in == ADC_STM32_INPUT_VREFINT || in == ADC_STM32_INPUT_TEMP || in == ADC_STM32_INPUT_VBAT;

		s_CfgPort = s_AnaPort = -1;
		one.PinP.PinNo = in;
		if (s_Pin[in][0] < 0)
		{
			ok = ok && adc.OpenChannel(&one, 1) == internal && s_CfgPort < 0 && s_AnaPort < 0;
		}
		else
		{
			ok = ok && adc.OpenChannel(&one, 1) && s_CfgPort == s_Pin[in][0] && s_CfgPin == s_Pin[in][1] &&
				 s_AnaPort == s_Pin[in][0] && s_AnaPin == s_Pin[in][1] && adc.Read(3, &d) && Near(d.Data, Volt(in), 0.002f);
		}
	}
	Check(ok);
	adc.PowerOff();

	// 1 MHz HCLK: 500 kHz ADC clock, 4 clocks after the calibration are 8 us
	SetClock(1000000);
	Check(adc.Init(cfg) && adc.OpenChannel(&set[3], 1) && adc.Read(3, &d) && Near(d.Data, 2.0f, 0.002f));
	Check(adc.Calibrate() && adc.Read(3, &d));
	adc.PowerOff();
	SetClock(80000000);

	// AHB clock 80 MHz / 2: ADC clock 20 MHz, 640.5 clocks are 32 us
	RCC->CFGR = 8U << RCC_CFGR_HPRE_Pos;
	bad.AcqTime = 17;
	Check(adc.Init(cfg) && adc.OpenChannel(&bad, 1) && Smp(5) == 7 && adc.Read(4, &d) && Near(d.Data, 1.5f, 0.002f));
	bad.AcqTime = 33;
	Check(!adc.OpenChannel(&bad, 1));
	adc.PowerOff();
	RCC->CFGR = 0;

	// HCLK / 2 above the ADC clock limit of the family takes HCLK / 4
	SetClock(140000000);
	Check(adc.Init(cfg) && ((TEST_COMMON->CCR & ADC_CCR_CKMODE) >> ADC_CCR_CKMODE_Pos) == TEST_CKMODE_140);
	adc.PowerOff();
	SetClock(200000000);
	Check(adc.Init(cfg) && ((TEST_COMMON->CCR & ADC_CCR_CKMODE) >> ADC_CCR_CKMODE_Pos) == 3);
	adc.PowerOff();
	SetClock(80000000);

	// Init failures leave the ADC off
	s_Io[0] = TEST_FAULT_NO_CAL;
	Check(!adc.Init(cfg) && !adc.Valid() && !(RCC->AHB2ENR & TEST_RCC_EN));
	s_Io[0] = TEST_FAULT_NO_RDY;
	Check(!adc.Init(cfg) && !adc.Valid());
	s_Io[0] = 0;

	// The ADC is free again
	Check(other.Init(cfg) && other.OpenChannel(set, 4) && other.StartConversion() && other.Read(r, 4) == 4);
	Check(Near(r[1].Data, 0.8f, 0.002f) && Near(r[2].Data, 3.0f, 0.005f));
	other.PowerOff();

	s_Io[2] = s_Checks;

	while (1)
	{
	}
}
