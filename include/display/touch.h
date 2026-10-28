/**-------------------------------------------------------------------------
@file	touch.h

@brief	Generic touch panel controller

A touch panel controller is a Device on a DeviceIntrf, I2C in general. Read
returns the contacts in the pixel coordinates of a display, in the
orientation set with Orientation, the same as DisplayDotMatrix: a point read
can be drawn at once.

TouchCfg_t gives the panel size in pixels with the display in landscape
(DISPL_ORIENT_LANDSCAPE, the native orientation of the panel) and how the
axes of the controller lie on it: TOUCH_FLAG_XY_SWAP when the X of the
controller runs along the panel height, then TOUCH_FLAG_X_INV and
TOUCH_FLAG_Y_INV to invert the panel axes. The controller range is scaled to
the panel size when the controller gives it (GT911), else the controller
reports pixels.

The reset pin (active low) and the interrupt pin are optional, PortNo -1
when not connected. Without the reset pin, Disable does nothing as the
controller could not be woken up.

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
#ifndef __TOUCH_H__
#define __TOUCH_H__

#include <stdint.h>

#include "device.h"
#include "coredev/iopincfg.h"
#include "display/display.h"

#define TOUCH_FLAG_XY_SWAP		(1U << 0)	//!< Controller X along the panel height
#define TOUCH_FLAG_X_INV		(1U << 1)	//!< Panel X inverted, after the swap
#define TOUCH_FLAG_Y_INV		(1U << 2)	//!< Panel Y inverted, after the swap

#pragma pack(push, 4)

typedef struct __Touch_Config {
	uint8_t DevAddr;			//!< 7 bit I2C address, 0 for the default of the controller
	uint16_t Width;				//!< Panel width in pixels, landscape
	uint16_t Height;			//!< Panel height in pixels, landscape
	uint32_t Flags;				//!< TOUCH_FLAG_xxx
	IOPinCfg_t RstPin;			//!< Reset, active low. PortNo -1: none
	IOPinCfg_t IntPin;			//!< Interrupt. PortNo -1: none
} TouchCfg_t;

typedef struct __Touch_Point {
	uint16_t X;					//!< Pixel column in the orientation set
	uint16_t Y;					//!< Pixel row in the orientation set
	uint8_t Id;					//!< Track id, kept while the contact stays
} TouchPoint_t;

#pragma pack(pop)

class Touch : public Device {
public:
	Touch();
	using Device::Read;

	/**
	 * @brief	Read the contacts.
	 *
	 * @param	pPoints		: Points read, in the orientation set
	 * @param	MaxPoints	: Size of pPoints
	 *
	 * @return	Number of points stored, 0 when not touched, -1 when the
	 * 			controller does not answer or is not set up
	 */
	virtual int Read(TouchPoint_t *pPoints, int MaxPoints) = 0;

	/**
	 * @brief	Set the orientation of the display the points are for.
	 *
	 * @param	Orient : Orientation, as DisplayDotMatrix::Orientation
	 */
	void Orientation(DISPL_ORIENT Orient);
	DISPL_ORIENT Orientation() const { return vOrient; }

	/// Panel size in the orientation set
	uint16_t Width() const;
	uint16_t Height() const;

protected:
	/**
	 * @brief	Check and keep the panel configuration.
	 *
	 * The interrupt pin is set up as input.
	 */
	bool SetPanel(const TouchCfg_t &Cfg, uint8_t DefaultAddr);

	/**
	 * @brief	Controller coordinate range, the panel size in pixels by
	 * 			default.
	 */
	void Range(uint16_t XRange, uint16_t YRange);

	/**
	 * @brief	Convert a point of the controller.
	 *
	 * @return	false when out of the controller range
	 */
	bool Convert(uint16_t X, uint16_t Y, uint8_t Id, TouchPoint_t &Pt) const;

	bool HasResetPin() const { return vCfg.RstPin.PortNo >= 0; }
	bool HasIntPin() const { return vCfg.IntPin.PortNo >= 0; }
	/// Drive the reset pin, which must be connected
	void ResetPin(bool bHigh);
	/// Drive the interrupt pin or release it, nothing when not connected
	void IntPinOut(bool bHigh);
	void IntPinIn();

	TouchCfg_t vCfg;
	uint16_t vXRange;			//!< Controller X range
	uint16_t vYRange;			//!< Controller Y range
	DISPL_ORIENT vOrient;
};

#endif // __TOUCH_H__
