/**-------------------------------------------------------------------------
@file	dac_r9a02.h

@brief	R9A02G021 8-bit D/A converter (DAC8)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 30. Two channels, 8 bits,
the reference is VCC: Vout = Code / 256 x VCC (30.3). Channel 0 drives the
DACOUT0 pin (P000) and can be the reference of comparator 1, channel 1
drives DACOUT1 (P001). The output has no buffer, 9 kOhm typical output
resistance, the load must be 4 MOhm or more and 35 pF or less (table
37.51). The output settles in 3 us (tDCONV) after a write. DAC8 needs VCC
2.7 V or more.

An ELC event can turn a channel on (ELSR20 for channel 0, ELSR19 for
channel 1): R9a02DacTrigger turns the channel off and links the event, the
output starts with the event (30.4). The event does not load new data.

The C functions own a channel per user (the DacR9a02 object, or the
comparator driver for channel 0 as the reference of comparator 1).

DacR9a02 is the DacDevice of IOsonata. DAC_CFG.Mode must be
DAC_MODE_SINGLE_ENDED. VCC is given with Vcc() for the conversions in volts,
3.3 V by default.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#ifndef __DAC_R9A02_H__
#define __DAC_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#define R9A02_DAC_CHAN_CNT			2
#define R9A02_DAC_SETTLE_US			3			//!< tDCONV, table 37.51

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	Open a channel: module on, code 0, conversion and output on.
///
/// @param	Chan	: 0 or 1
/// @param	bPinOut	: true to drive the DACOUTn pin
/// @param	pOwner	: User of the channel
///
/// @return	false if Chan is not valid or another user has the channel
bool R9a02DacOpen(int Chan, bool bPinOut, const void *pOwner);

/// @brief	Close a channel: output off, pin released. The module stops with
/// 		the last channel.
///
/// @param	Chan	: 0 or 1
/// @param	pOwner	: User of the channel
void R9a02DacClose(int Chan, const void *pOwner);

/// @brief	Set the output code. The output settles in R9A02_DAC_SETTLE_US.
///
/// @param	Chan	: 0 or 1, open
/// @param	Code	: Vout = Code / 256 x VCC
/// @param	pOwner	: User of the channel
///
/// @return	false if the channel is not open by pOwner
bool R9a02DacWrite(int Chan, uint8_t Code, const void *pOwner);

/// @brief	Turn the conversion and the output of a channel on or off.
///
/// The code is kept. Off, the pin is high impedance and the analog current
/// stops (30.5.2).
///
/// @param	Chan	: 0 or 1, open
/// @param	bOn		: true for on
/// @param	pOwner	: User of the channel
///
/// @return	false if the channel is not open by pOwner
bool R9a02DacOutput(int Chan, bool bOn, const void *pOwner);

/// @brief	Start a channel with an ELC event.
///
/// The channel is turned off, then the event turns it on (30.4).
///
/// @param	Chan	: 0 or 1, open
/// @param	EvtId	: ELC event number, 0 removes the link
/// @param	pOwner	: User of the channel
///
/// @return	false if the channel is not open by pOwner or the link failed
bool R9a02DacTrigger(int Chan, uint8_t EvtId, const void *pOwner);

#ifdef __cplusplus
}

#include "converters/dac_device.h"

/// DAC8 as a DacDevice
class DacR9a02 : public DacDevice {
public:
	DacR9a02();
	virtual ~DacR9a02() {}
	DacR9a02(DacR9a02&);	// Copy ctor not allowed

	/// @brief	Initialization, the channels stay closed.
	///
	/// @param	Cfg		: DAC_MODE_SINGLE_ENDED
	/// @param	pIntrf	: Not used, internal converter
	///
	/// @return	false for another mode
	virtual bool Init(const DAC_CFG &Cfg, DeviceIntrf * const pIntrf = NULL);

	/// @brief	Open a channel at code 0.
	///
	/// @param	Chan	: 0 or 1
	/// @param	bPinOut	: true to drive the DACOUTn pin
	///
	/// @return	false if Chan is not valid or the comparator has channel 0
	bool OpenChannel(int Chan, bool bPinOut = true);
	void CloseChannel(int Chan);

	/// @brief	Output code, Vout = Code / 256 x VCC.
	bool WriteCode(int Chan, uint8_t Code);

	/// @brief	Output voltage, the closest code below VCC.
	///
	/// @return	Voltage of the code set, negative on failure
	float WriteVolt(int Chan, float Volt);

	/// @brief	Turn the channel on with an ELC event, R9a02DacTrigger.
	bool Trigger(int Chan, uint8_t EvtId);

	/// @brief	Supply voltage VCC, the reference, for WriteVolt.
	void Vcc(float Volt) { if (Volt > 0.0f) vVcc = Volt; }
	float Vcc() { return vVcc; }

	/// @brief	Output on again for the open channels.
	virtual bool Enable();

	/// @brief	Output off for the open channels, codes kept.
	virtual void Disable();

	/// @brief	Codes back to 0.
	virtual void Reset();

	/// @brief	Channels closed, module stopped.
	virtual void PowerOff();

private:
	float vVcc;
	uint8_t vCode[R9A02_DAC_CHAN_CNT];
	bool vbOpen[R9A02_DAC_CHAN_CNT];
	bool vbPinOut[R9A02_DAC_CHAN_CNT];
};

#endif

#endif
