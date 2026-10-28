/**-------------------------------------------------------------------------
@file	adc_lpc546xx.h

@brief	LPC546xx ADC0 polling implementation

ADC0 of the LPC546xx, 12 bits, 12 inputs and the temperature sensor:
software triggered scans of sequence A read by polling. Not supported, Init
or OpenChannel fail on them: interrupt, DMA, continuous mode, oversampling,
differential inputs, BurstMode, FIFO memory, pin pull resistors, a Gain other
than 0x100. One object owns ADC0 until PowerOff or destruction.

Logical channel Chan 0 to 11. PinP.PinNo is the ADC0 input:
	0	P0_10		4	P0_16		8	P2_1
	1	P0_11		5	P0_31		9	P3_21
	2	P0_12		6	P1_0		10	P3_22
	3	P0_15		7	P2_0		11	P0_23
	12	ADC_LPC546XX_INPUT_TEMP, temperature sensor
The temperature sensor is converted on ADC channel 0 (INSEL), input 0 and the
sensor cannot be open together. P0_11 and P0_12 are SWCLK and SWDIO, P0_10
is SWO: opening them ends debugging. Each input is used by one logical
channel only.

Pins: the pin of an input is analog (function 0, no pull, digital buffer
off) while its channel is open and the ADC is powered. OpenChannel saves the
IOCON register and the GPIO direction of the pin, CloseChannel and PowerOff
put them back with the digital buffer on. Disable turns the digital buffer of
the open pins on before it powers ADC0 down. This is the workaround of
erratum ADC.1 (ES_LPC546XX, high current in low power modes): Init also turns
on the digital buffer of the ADC0 pins that have it off, before it powers the
ADC. Call Disable before deep sleep or deep power down, the erratum also
applies there.

Clock: synchronous mode, the system clock divided by CLKDIV + 1. The clock of
the conversions is at most 60 MHz, the clock of the NXP SDK example (180 MHz
/ 3), the calibration runs at 30 MHz or less (NXP SDK). Sampling time: 2.5 to
9.5 ADC clocks (TSAMP), common to all inputs. OpenChannel takes the fastest
clock and the shortest TSAMP that cover the largest AcqTime in usec of the
open pin inputs; it fails when 9.5 clocks at the system clock / 256 are too
short.

Resolution: 12 bits only. RESOL also has 6, 8 and 10 bits, the alignment of
their results in the data registers is not documented in the NXP sources the
driver was checked against; Resolution returns 12 for any value.

Reference: VREFP, the caller gives its voltage. Data in volt is
Raw * Voltage / 4095.

Calibration: Init and Enable power the ADC, wait 10 us, set ADC_ENA, then run
a calibration when CALREQD is set or the initialization conversion
(ADC_INIT) when it is not, as the NXP SDK. Calibrate runs a calibration.

Temperature sensor: powered while its channel is open, 100 us settling after
power up (the datasheet asks for settling, the driver has no time from it).
The datasheet requires single channel burst mode: the last result of 9 or
more back to back conversions is the accurate one. The driver converts the
sensor in burst mode with TSAMP 7 and uses the 9th result. Datasheet values
quoted by NXP: about 584 mV at 0 C, -2.04 mV/C, +/-5 C.

A conversion that does not end in time stops the sequence and makes the
driver invalid; Reset resets ADC0 and powers it up again.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __ADC_LPC546XX_H__
#define __ADC_LPC546XX_H__

#include <stdint.h>

#include "converters/adc_device.h"

/** @addtogroup Converters
  * @{
  */

#define ADC_LPC546XX_MAX_CHAN		12		//!< Logical channels
#define ADC_LPC546XX_NB_PIN_INPUT	12		//!< Pin inputs 0 to 11
#define ADC_LPC546XX_INPUT_TEMP		12		//!< Temperature sensor input

/// ADC0 of the LPC546xx
class AdcLpc546xx : public AdcDevice {
public:
	AdcLpc546xx();
	~AdcLpc546xx() override;

	bool Init(const AdcCfg_t &Cfg, Timer *pTimer = nullptr, DeviceIntrf *pIntrf = nullptr) override;
	bool OpenChannel(const AdcChanCfg_t *pCfg, int Count) override;
	void CloseChannel(int Chan) override;
	bool StartConversion() override;
	void StopConversion() override;
	bool UpdateData() override;
	int Read(AdcData_t *pBuff, int Len) override;
	bool Read(int Chan, AdcData_t *pBuff) override;
	bool Calibrate() override;
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	void PowerOff() override;
	uint16_t Resolution(uint16_t Val) override;
	using AdcDevice::Resolution;

private:
	AdcLpc546xx(const AdcLpc546xx &) = delete;
	AdcLpc546xx &operator=(const AdcLpc546xx &) = delete;

	bool Owned();
	bool PowerUp();
	void PinAnalog(int Chan);
	void PinRestore(int Chan);
	void Timing();
	bool Scan(const int *pChan, int Count);
	bool ScanPins(uint32_t Mask, uint16_t *pRaw);
	bool ScanTemp(uint16_t *pRaw);
	void Abort();

	uint8_t vInput[ADC_LPC546XX_MAX_CHAN];		//!< ADC0 input + 1 of each logical channel, 0 when closed
	bool vbNew[ADC_LPC546XX_MAX_CHAN];			//!< Result not read yet
	uint16_t vRaw[ADC_LPC546XX_MAX_CHAN];		//!< Last result
	int vAcqUs[ADC_LPC546XX_MAX_CHAN];			//!< Sampling time asked for each logical channel, usec
	uint32_t vPinCfg[ADC_LPC546XX_MAX_CHAN];	//!< IOCON register of the pin before OpenChannel
	bool vbPinOut[ADC_LPC546XX_MAX_CHAN];		//!< GPIO direction of the pin before OpenChannel
	float vRef;									//!< Reference voltage
	uint32_t vSysClk;							//!< System clock at Init, Hz
	uint32_t vCtrl;								//!< CTRL of the pin conversions
	uint32_t vTempCtrl;							//!< CTRL of the temperature sensor conversions
	uint32_t vPwrOwned;							//!< PDRUNCFG0 supplies Init powered
	bool vbOn;									//!< ADC0 powered and initialized
};

/** @} End of group Converters */

#endif // __ADC_LPC546XX_H__
