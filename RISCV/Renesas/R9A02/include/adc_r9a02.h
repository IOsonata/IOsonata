/**-------------------------------------------------------------------------
@file	adc_r9a02.h

@brief	R9A02G021 12-bit A/D converter (ADC12)

AdcDevice implementation. Each AdcChanCfg_t is a logical channel, Chan 0 to
R9A02_ADC_MAX_CHAN - 1, converting the input given in PinP.PinNo:

	0 to 5		: ANI0 to ANI5 (P002, P003, P400, P401, P006, P007)
	16 to 19	: ANI16 to ANI19 (P000, P001, P105, P106)
	R9A02_ADC_INPUT_TEMP : temperature sensor
	R9A02_ADC_INPUT_VREF : internal reference voltage, 1.47 V typical

R9A02G021 User's Manual R01UH1036EJ0110, chapter 29.

Conversion of the open channels, a round:
	- scan mode when the open channels are exactly ANI0 to ANI3, ANI1 to
	  ANI4 or ANI2 to ANI5 (table 29.13): the four in one sequence, one
	  ADC_ENDI
	- select mode otherwise, one channel after the other

ADC_CONV_MODE_SINGLE: StartConversion converts one round. Without interrupt
it returns with the results, with interrupt DEV_EVT_DATA_RDY marks the end.

ADC_CONV_MODE_CONTINUOUS, interrupt needed: rounds follow each other until
StopConversion. AdcCfg_t.Rate in mHz is the round rate, it needs the Timer
given to Init: a trigger of that timer starts each round, a round still
running when it comes is not restarted. Rate 0 starts each round at the end
of the previous one. Each round gives DEV_EVT_DATA_RDY and one AdcData_t
per channel to the channel FIFO (AdcChanCfg_t.pFifoMem), Timestamp in us of
the Timer, the round count without one.

Hardware trigger (Trigger): the TAU channel 1 interval, the RTC alarm, the
TML32 interval or an ELC event starts the conversions, hardware trigger
no-wait mode (29.6.10, 29.6.12). StartConversion arms it, StopConversion
disarms it. Each trigger converts a scan round, or one channel in select
mode, the next trigger the next channel. The internal inputs cannot be
used. A compare window (Window) then keeps only the results in, or out of,
the window: the others give no ADC_ENDI and are not stored (29.3.4).

Snooze (Snooze): the conversion runs in Snooze, started by the Snooze
request through the ELC (SYSTEM_SNZREQ), hardware trigger wait mode, one
round of one channel or one scan group (29.8). StartConversion arms it. With
R9a02SnoozeSet the result in the window wakes the CPU (WakeEvt
R9A02_EVT_ADC_ENDI), out of it the MCU goes back to Software Standby (End
R9A02_SNZEND_ADC_NOMATCH).

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef __ADC_R9A02_H__
#define __ADC_R9A02_H__

#include <stdint.h>

#include "converters/adc_device.h"

#define R9A02_ADC_MAX_CHAN			8			//!< Logical channels
#define R9A02_ADC_INPUT_TEMP		0x80		//!< PinP.PinNo of the temperature sensor
#define R9A02_ADC_INPUT_VREF		0x81		//!< PinP.PinNo of the internal reference

/// Hardware trigger, ADM1.ADTRS (29.3.2)
typedef enum __R9a02_Adc_Trig {
	R9A02_ADC_TRIG_SOFT,			//!< Software, StartConversion
	R9A02_ADC_TRIG_TAU0_CH1,		//!< TAU channel 1 (TAU0_ENDI1)
	R9A02_ADC_TRIG_RTC,				//!< RTC alarm
	R9A02_ADC_TRIG_TML32,			//!< TML32 interval (TML32_OUTI)
	R9A02_ADC_TRIG_ELC,				//!< Event linked by the ELC (ELSR8)
} R9A02_ADC_TRIG;

/// ADC12 as an AdcDevice
class AdcR9a02 : public AdcDevice {
public:
	AdcR9a02();
	virtual ~AdcR9a02();
	AdcR9a02(AdcR9a02&);	// Copy ctor not allowed

	/// @brief	Initialization.
	///
	/// Supported: ADC_CONV_MODE_SINGLE, ADC_CONV_MODE_CONTINUOUS with
	/// interrupt, resolution 8, 10 or 12 bits, one reference,
	/// ADC_REFVOLT_TYPE_SUPPLY (VCC) or ADC_REFVOLT_TYPE_EXTERNAL (AVREFP on
	/// P002) with its voltage. Anything else fails. The trigger is software,
	/// no window, no Snooze.
	///
	/// @param	Cfg		: Configuration
	/// @param	pTimer	: Continuous mode, round rate and time stamps
	/// @param	pIntrf	: Not used, internal converter
	///
	/// @return	true on success
	virtual bool Init(const AdcCfg_t &Cfg, Timer * const pTimer = NULL, DeviceIntrf * const pIntrf = NULL);

	/// @brief	Set the resolution, 8, 10 or 12 bits, the closest is taken.
	///
	/// @return	Resolution set
	virtual uint16_t Resolution(uint16_t Val);
	virtual uint16_t Resolution() { return vResolution; }

	/// @brief	Configure logical channels.
	///
	/// Single ended only. AcqTime (us) selects the conversion clock and the
	/// operation voltage mode so that the sampling time is long enough.
	virtual bool OpenChannel(const AdcChanCfg_t * const pChanCfg, int NbChan);
	virtual void CloseChannel(int Chan);

	/// @brief	Convert the open channels.
	///
	/// Single mode: one round. Without interrupt the conversions are done
	/// before it returns. With interrupt they run in the background, the
	/// event handler gets DEV_EVT_DATA_RDY when the last one is done.
	/// Continuous mode: rounds until StopConversion. Hardware trigger and
	/// Snooze: the trigger is armed.
	virtual bool StartConversion();
	virtual void StopConversion();

	/// @return	true if a channel has a result not read yet. With a hardware
	/// 			trigger and without interrupt it takes the last result.
	virtual bool UpdateData();

	/// @brief	Results not read yet, one per channel: the oldest of its FIFO
	/// 		for a channel with one, its last result otherwise.
	virtual int Read(AdcData_t *pBuff, int Len);

	/// @brief	Result of one channel, as Read. Software trigger without
	/// 		interrupt: the channel is converted now when it has none waiting.
	virtual bool Read(int Chan, AdcData_t *pBuff);

	/// @brief	Zero and full scale measurement (ADTES), the results are
	/// corrected with them afterward.
	virtual bool Calibrate();

	/// @brief	Select the start of the conversions.
	///
	/// With the conversions stopped, Snooze off and the internal inputs
	/// closed for a hardware trigger. The source is set up by its own driver:
	/// a Timer on TAU channel 1 (DevNo 2), the RTC alarm, R9a02Tml32Init
	/// (MODE_DTC or MODE_INT, the interval flag cleared each time), the ELC
	/// event source.
	///
	/// @param	Trig	: Trigger source
	/// @param	ElcEvt	: R9A02_ADC_TRIG_ELC, ELC event number
	///
	/// @return	false if not possible now
	bool Trigger(R9A02_ADC_TRIG Trig, uint8_t ElcEvt = 0);
	R9A02_ADC_TRIG Trigger() { return vTrig; }

	/// @brief	Compare window for the hardware trigger and Snooze.
	///
	/// The upper 8 bits of the result are compared with Low and High in 8
	/// bits of the reference (ADLL, ADUL). A result outside of what is asked
	/// gives no ADC_ENDI and is not stored (29.3.4). Without a hardware
	/// trigger or Snooze the window is kept but not used.
	///
	/// @param	LowVolt		: Low limit in volts
	/// @param	HighVolt	: High limit in volts, above LowVolt in 8 bits
	/// @param	bOutside	: false to keep the results from Low to High,
	/// 					  true those below Low or above High
	///
	/// @return	false if the limits are not valid
	bool Window(float LowVolt, float HighVolt, bool bOutside = false);

	/// @brief	No compare window, every result kept.
	void WindowOff();

	/// @brief	Conversion in Snooze, or back to the trigger set.
	///
	/// On: one channel or one scan group open, external inputs, conversions
	/// stopped, ICLK:PCLKB 8:1 or less. StartConversion then arms the
	/// converter, a conversion starts at each Snooze entry. Results come as
	/// with a hardware trigger. The converter is on in Software Standby only,
	/// R9a02LowPowerEnter sets ADM2.AWC and ADCE before it and clears them
	/// after it (table 29.20).
	///
	/// @param	bEnable	: true for on
	///
	/// @return	false if not possible
	bool Snooze(bool bEnable);
	bool Snooze() { return vbSnooze; }

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual void PowerOff();

	/// Interrupt handler, internal use
	void IntHandler();

	/// Continuous mode timer trigger, internal use
	void TimerHandler();

	/// Software Standby hook of the Snooze mode, internal use
	void StandbyHook(bool bEnter);

	/// @brief	Round rate of the continuous mode in mHz, as AdcCfg_t.Rate.
	///
	/// Taken at the next StartConversion.
	virtual uint32_t Rate(uint32_t Val) { vRate = Val; return vRate; }
	virtual uint32_t Rate() { return vRate; }

private:
	bool Configure();
	bool ConvertOne(int Chan);
	bool ConvertScan();
	bool StartOne(int Chan);
	bool StartRound();
	void EndRound();
	void Store(int Chan, uint16_t Raw);
	void StoreScan();
	void HwEnd();
	bool Arm();
	void Disarm();
	void UpdateScan();
	uint8_t Adm0();
	bool HwTrig() { return vTrig != R9A02_ADC_TRIG_SOFT || vbSnooze; }
	float ToVolt(int Chan, uint32_t Raw);
	int NextChan(int From);

	hCFifo_t vhFifo[R9A02_ADC_MAX_CHAN];	// Channel FIFO, NULL when none
	Timer *vpTimer;							// Round rate and time stamps
	int vTimerTrig;							// Timer trigger of the round rate, -1 when none
	uint32_t vRound;						// Rounds converted
	int vScan;								// First ANI of the scan group, -1 for select mode
	R9A02_ADC_TRIG vTrig;
	uint8_t vElcEvt;
	uint8_t vAdul;							// Window, 8 bits
	uint8_t vAdll;
	bool vbWindow;
	bool vbOutside;
	bool vbSnooze;
	bool vbArmed;							// Hardware trigger armed
	volatile bool vbRunning;				// Continuous mode started

	uint8_t vAds[R9A02_ADC_MAX_CHAN];		// ADS value, 0xFF when closed
	uint32_t vAcqNs[R9A02_ADC_MAX_CHAN];	// Sampling time needed
	uint16_t vRaw[R9A02_ADC_MAX_CHAN];		// Last result, right aligned
	uint32_t vStamp[R9A02_ADC_MAX_CHAN];	// Time stamp of the last result
	bool vbNew[R9A02_ADC_MAX_CHAN];			// Result not read yet
	float vRefVolt;							// '+' reference voltage
	uint8_t vAdm0;							// FR and LV
	uint8_t vAdm1;							// ADLSP, one-shot
	uint8_t vAdm2;							// Resolution and reference
	uint32_t vFadHz;
	uint32_t vLoops;						// Poll loops for one conversion
	int vIntNo;								// IELSR of ADC_ENDI, -1 when none
	int vCurChan;							// Channel converting in background, -1 when none
	uint16_t vZero;							// Calibration, 12-bit scale
	uint16_t vFull;
	bool vbCal;
	bool vbInternal;						// ADS.ADISS currently set
	bool vbDiscard;							// Next result after an ADISS change is not valid
};

#endif
