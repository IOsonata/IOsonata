/**-------------------------------------------------------------------------
@file	iopincfg_re01.c

@brief	I/O pin configuration implementation on Renesas RE01 series

NOTE: Renesas has preassign interrupt pins for each specific gpio.

@author	Hoang Nguyen Hoan
@date	Nov. 11, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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

#include <stdio.h>
#include <stdbool.h>

#include "re01xxx.h"
#include "coredev/iopincfg.h"
#include "interrupt_re01.h"

#define PFS_PSEL_Pos              (24UL)                    /*!< PSEL (Bit 24)                                         */
#define PFS_PSEL_Msk              (0x1f000000UL)            /*!< PSEL (Bitfield-Mask: 0x1f)                            */
#define PFS_PMR_Pos               (16UL)                    /*!< PMR (Bit 16)                                          */
#define PFS_PMR_Msk               (0x10000UL)               /*!< PMR (Bitfield-Mask: 0x01)                             */
#define PFS_ASEL_Pos              (15UL)                    /*!< ASEL (Bit 15)                                         */
#define PFS_ASEL_Msk              (0x8000UL)                /*!< ASEL (Bitfield-Mask: 0x01)                            */
#define PFS_ISEL_Pos              (14UL)                    /*!< ISEL (Bit 14)                                         */
#define PFS_ISEL_Msk              (0x4000UL)                /*!< ISEL (Bitfield-Mask: 0x01)                            */
#define PFS_EOFR_Pos              (12UL)                    /*!< EOFR (Bit 12)                                         */
#define PFS_EOFR_Msk              (0x3000UL)                /*!< EOFR (Bitfield-Mask: 0x03)                            */
#define PFS_DSCR_Pos              (10UL)                    /*!< DSCR (Bit 10)                                         */
#define PFS_DSCR_Msk              (0xc00UL)                 /*!< DSCR (Bitfield-Mask: 0x03)                            */
#define PFS_PCODR_Pos             (7UL)                     /*!< PCODR (Bit 7)                                         */
#define PFS_PCODR_Msk             (0x80UL)                  /*!< PCODR (Bitfield-Mask: 0x01)                           */
#define PFS_NCODR_Pos             (6UL)                     /*!< NCODR (Bit 6)                                         */
#define PFS_NCODR_Msk             (0x40UL)                  /*!< NCODR (Bitfield-Mask: 0x01)                           */
#define PFS_PDCR_Pos              (5UL)                     /*!< PDCR (Bit 5)                                          */
#define PFS_PDCR_Msk              (0x20UL)                  /*!< PDCR (Bitfield-Mask: 0x01)                            */
#define PFS_PUCR_Pos              (4UL)                     /*!< PUCR (Bit 4)                                          */
#define PFS_PUCR_Msk              (0x10UL)                  /*!< PUCR (Bitfield-Mask: 0x01)                            */
#define PFS_PDR_Pos               (2UL)                     /*!< PDR (Bit 2)                                           */
#define PFS_PDR_Msk               (0x4UL)                   /*!< PDR (Bitfield-Mask: 0x01)                             */
#define PFS_PIDR_Pos              (1UL)                     /*!< PIDR (Bit 1)                                          */
#define PFS_PIDR_Msk              (0x2UL)                   /*!< PIDR (Bitfield-Mask: 0x01)                            */
#define PFS_PODR_Pos              (0UL)                     /*!< PODR (Bit 0)                                          */
#define PFS_PODR_Msk              (0x1UL)                   /*!< PODR (Bitfield-Mask: 0x01)                            */

#define ICU_IRQCR_IRQMD_Pos					(0UL)
#define ICU_IRQCR_IRQMD_Mask				(3UL)
#define ICU_IRQCR_IRQMD_FALLING_EDGE		(0UL)
#define ICU_IRQCR_IRQMD_RISING_EDGE			(1UL)
#define ICU_IRQCR_IRQMD_FALLING_RISING_EDGE	(2UL)
#define ICU_IRQCR_IRQMD_LOW_LEVEL			(3UL)


#define RE01_1500KB_PIN_MAX_INT		(10)
#define RE01_1500KB_MAX_PORT		(9)

static int s_MaxNbIOPins[RE01_1500KB_MAX_PORT] = {
#ifdef RE01_1500KB_DBN
	16, 15, 7, 8, 10, 15, 11, 5, 16
#elif defined(RE01_1500KB_CFB)
	16, 14, 7, 16, 10, 15, 11, 5, 16
#elif defined(RE01_1500KB_CFP)
	16, 14, 7, 5, 5, 5, 5, 5, 5
#endif
};

#pragma pack(push, 4)
typedef struct {
	int Idx;
	IOPINSENSE Sense;
	IOPinEvtHandler_t SensEvtCB;
    uint16_t PortPinNo;
    void *pCtx;
    IRQn_Type IrqNo;
} IOPINSENS_EVTHOOK;
#pragma pack(pop)

static IOPINSENS_EVTHOOK s_GpIOSenseEvt[RE01_1500KB_PIN_MAX_INT + 1] = {
	{0, 0, NULL}, {1, 0, NULL}, {2, 0, NULL}, {3, 0, NULL}, {4, 0, NULL},
	{5, 0, NULL}, {6, 0, NULL}, {7, 0, NULL}, {8, 0, NULL}, {9, 0, NULL}
};

static uint16_t s_GpIOPowerSupply[RE01_1500KB_MAX_PORT] = {0,};

// IRQ pins map are fixed
static const uint32_t s_Re01IntPins[] = {
	0x016, 0x04b, 0x056,	// IRQ0
	0x127, 0x151, 0x155,	// IRQ1
	0x24a, 0x288,			// IRQ2
	0x349, 0x387,			// IRQ3
	0x422, 0x458,			// IRQ4
	0x51d, 0x564,			// IRQ5
	0x60e, 0x61c,			// IRQ6
	0x70f, 0x717,			// IRQ7
	0x815, 0x825,			// IRQ8
	0x914, 0x924,			// IRQ9
};

static bool IsValidIOInterrupt(int IntNo, int PortNo, int PinNo)
{
	bool retval = false;
	int pinval = ((IntNo & 0xf) << 8) | ((PortNo & 0xf) << 4) | (PinNo & 0xf);

	for (int i = 0; i < sizeof(s_Re01IntPins) / sizeof(uint32_t); i++)
	{
		if (s_Re01IntPins[i] == pinval)
		{
			retval = true;
			break;
		}
	}

	return retval;
}

static int IOPinFindAvailInterrupt(int PortNo, int PinNo)
{
	int retval = -1;

	for (int i = 0; i < sizeof(s_Re01IntPins) / sizeof(uint32_t); i++)
	{
		int pinval = ((PortNo & 0xf) << 4) | (PinNo & 0xf);
		if ((s_Re01IntPins[i] & 0xff) == pinval)
		{
			retval = s_Re01IntPins[i] >> 8;
			break;
		}
	}

	return retval;
}

static void RE01IOPinIRQHandler(int IntNo, void *pCtx)
{
	IOPINSENS_EVTHOOK *evthook = (IOPINSENS_EVTHOOK*)pCtx;

	if (evthook->SensEvtCB)
	{
		evthook->SensEvtCB(evthook->Idx, evthook->pCtx);
	}
}

static void RE01IOPinSupplyEnable(int PortNo, int PinNo)
{
    SYSTEM->PRCR = 0xA502U;
	switch (PortNo)
	{
		case 0:
			if (PinNo < 10 || PinNo > 15)
			{
				return;
			}
		case 5:
			SYSTEM->VOCR_b.IV3CTL = 0;
			break;
		case 1:
			SYSTEM->VOCR_b.IV2CTL = 0;
			break;
		case 2:
			if (PinNo < 2 || PinNo > 4)
			{
				return;
			}
		case 3:
		case 6:
		case 7:
			SYSTEM->VOCR_b.IV1CTL = 0;
			break;
		case 8:
			SYSTEM->VOCR_b.IV0CTL = 0;
			break;
	}

	// Keep track of IO that needs power supply enabled
	s_GpIOPowerSupply[PortNo] |= 1 << PinNo;
    SYSTEM->PRCR = 0xA500U;
}

static void RE01IOPinSupplyDisable(int PortNo, int PinNo)
{
    SYSTEM->PRCR = 0xA502U;

	s_GpIOPowerSupply[PortNo] &= ~(1 << PinNo);

	if (s_GpIOPowerSupply[8] == 0)
	{
		SYSTEM->VOCR_b.IV0CTL = 1;
	}

	uint16_t d = s_GpIOPowerSupply[2] | s_GpIOPowerSupply[3] | s_GpIOPowerSupply[6] | s_GpIOPowerSupply[7];

	if (d == 0)
	{
		SYSTEM->VOCR_b.IV1CTL = 1;
	}

	if (s_GpIOPowerSupply[1] == 0)
	{
		SYSTEM->VOCR_b.IV2CTL = 1;
	}

	d = s_GpIOPowerSupply[0] | s_GpIOPowerSupply[5];

	if (d == 0)
	{
		SYSTEM->VOCR_b.IV3CTL = 1;
	}
    SYSTEM->PRCR = 0xA500U;
}

/**
 * @brief Configure individual I/O pin.
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						for RE01
 * 						IOPINOP_FUNC_0 - IOPINOP_FUNC_30 -> PSEL1-31
 * 						IOPINOP_FUNC_31 -> Analog
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	PORT0_Type *reg = (PORT0_Type *)(PORT0_BASE + PortNo * 0x20);

	if (PortNo == -1 || PinNo == -1 || PortNo > RE01_1500KB_MAX_PORT)
		return;

	RE01IOPinSupplyEnable(PortNo, PinNo);

	uint32_t psfval = PFS_DSCR_Msk;	// Default high drive

	if (PinOp == IOPINOP_GPIO)
	{
		if (Dir == IOPINDIR_OUTPUT)
		{
			psfval |= PFS_PDR_Msk;
		}
	}
	else if (PinOp < IOPINOP_FUNC31)
	{
		// Alternate function
		psfval |= (PinOp << PFS_PSEL_Pos) | PFS_PMR_Msk;
	}
	else
	{
		// Analog
		psfval |= PFS_ASEL_Msk;
	}

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:
		case IOPINRES_PULLUP:
			psfval |= PFS_PUCR_Msk;	// PUCR
			if (Type == IOPINTYPE_OPENDRAIN)
			{
				// N-Channel
				psfval |= PFS_NCODR_Msk;	// NCODR
			}
			break;
		case IOPINRES_PULLDOWN:
			psfval |=  PFS_PDCR_Msk;	// PDCR
			if (Type == IOPINTYPE_OPENDRAIN)
			{
				// P-Channel
				psfval |= PFS_PCODR_Msk;	// PCODR
			}
			break;
		case IOPINRES_NONE:
			break;
	}

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	int offset = (PinNo + (PortNo << 4)) << 2;
	*(__IOM uint32_t*)(PFS_BASE + offset) = psfval;

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;

	// Default high speed
	IOPinSetSpeed(PortNo, PinNo, IOPINSPEED_HIGH);
}

/**
 * @brief	Disable I/O pin
 *
 * Some hardware such as low power mcu allow I/O pin to be disconnected
 * in order to save power. There is no enable function. Reconfigure the
 * I/O pin to re-enable it.
 *
 * @param	PortNo 	: Port number
 * @param	PinNo	: Pin Number
 */
void IOPinDisable(int PortNo, int PinNo)
{
	if (PortNo == -1 || PinNo == -1)
		return;

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	int offset = (PinNo + (PortNo << 4)) << 2;
	*(__IOM uint32_t*)(PFS_BASE + offset) = 0;

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;

	RE01IOPinSupplyDisable(PortNo, PinNo);
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
	if (IntNo < 0 || IntNo >= RE01_1500KB_PIN_MAX_INT)
	{
		return;
	}

	Re01UnregisterIntHandler(s_GpIOSenseEvt[IntNo].IrqNo);

	int pinno = s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF;
	int portno = (s_GpIOSenseEvt[IntNo].PortPinNo >> 8) & 0xf;
	int offset = (pinno + (portno << 4)) << 2;
	__IOM uint32_t *psf = (__IOM uint32_t*)(PFS_BASE + offset);
	uint32_t psfval = *psf & ~(PFS_ISEL_Msk | PFS_EOFR_Msk);

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	*psf = psfval;

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;


    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;
    s_GpIOSenseEvt[IntNo].pCtx = NULL;
    s_GpIOSenseEvt[IntNo].IrqNo = -1;
}

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 * STM32 : IntNo must be same as PinNo.  It is directly related to hardware.
 *
 * @param	IntNo	: Interrupt number. -1 for port event interrupt
 * 			IntPrio : Interrupt priority
 * 			PortNo  : Port number (up to 32 ports)
 * 			PinNo   : Pin number (up to 32 pins)
 * 			Sense   : Sense type of event on the I/O pin
 * 			pEvtCB	: Pointer to callback function when event occurs
 * 			pCtx	: Pointer to context data to be pass to the handler function
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if (IntNo < 0 || IntNo >= RE01_1500KB_PIN_MAX_INT)// ||
//		IsValidIOInterrupt(IntNo, PortNo, PinNo) == false)
	{
		return false;
	}

	if (IsValidIOInterrupt(IntNo, PortNo, PinNo) == false)
	{
		IntNo = IOPinFindAvailInterrupt(PortNo, PinNo);
	}

	int offset = (PinNo + (PortNo << 4)) << 2;
	__IOM uint32_t *psf = (__IOM uint32_t*)(PFS_BASE + offset);
	uint32_t psfval = *psf & ~(PFS_EOFR_Msk);
	__IOM uint8_t *irqcr = (__IOM uint8_t*)&ICU->IRQCR0;


	int idx = IntNo;//irq - IEL0_IRQn;

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			psfval |= (2<<PFS_EOFR_Pos);
			irqcr[idx] = ICU_IRQCR_IRQMD_FALLING_EDGE;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			psfval |= (1<<PFS_EOFR_Pos);
			irqcr[idx] = ICU_IRQCR_IRQMD_RISING_EDGE;
			break;
		case IOPINSENSE_TOGGLE:
			psfval |= (3<<PFS_EOFR_Pos);
			irqcr[idx] = ICU_IRQCR_IRQMD_FALLING_RISING_EDGE;
			break;
	}

	psfval |= PFS_ISEL_Msk;

	s_GpIOSenseEvt[idx].Sense = Sense;
	s_GpIOSenseEvt[idx].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[idx].SensEvtCB = pEvtCB;
	s_GpIOSenseEvt[idx].pCtx = pCtx;

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	*psf = psfval;

	ICU->WUPEN |= (1 << idx);

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;

	IRQn_Type irq = Re01RegisterIntHandler(IntNo + RE01_EVTID_PORT_IRQ0, IntPrio, RE01IOPinIRQHandler, &s_GpIOSenseEvt[idx]);

	if (irq == -1)
	{
		// Can't allocate interrupt
		return false;
	}

	s_GpIOSenseEvt[idx].IrqNo = irq;

	return true;
}

/**
 * @brief	Allocate I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change. This function will automatically
 * allocate available interrupt number to use for the pin.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 *
 * @param	IntPrio : Interrupt priority
 * @param	PortNo  : Port number (up to 32 ports)
 * @param	PinNo   : Pin number (up to 32 pins)
 * @param	Sense   : Sense type of event on the I/O pin
 * @param	pEvtCB	: Pointer to callback function when event occurs
 * @param	pCtx	: Pointer to context data to be pass to the handler function
 *
 * @return	Interrupt number on success
 * 			-1 on failure.
 */
int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	int intno = IOPinFindAvailInterrupt(PortNo, PinNo);

	if (intno >= 0)
	{
		bool res = IOPinEnableInterrupt(intno, IntPrio, PortNo, PinNo, Sense, pEvtCB, pCtx);
		if (res == true)
			return intno;
	}

	return -1;
}

/**
 * @brief Set I/O pin sensing option
 *
 * Some hardware allow pin sensing to wake up or active other subsystem without
 * requiring enabling interrupts. This requires the I/O already configured
 *
 * @param	PortNo : Port number (up to 32 ports)
 * 			PinNo   : Pin number (up to 32 pins)
 * 			Sense   : Sense type of event on the I/O pin
 */
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
	int offset = (PinNo + (PortNo << 4)) << 2;
	__IOM uint32_t *psf = (__IOM uint32_t*)(PFS_BASE + offset);
	uint32_t psfval = *psf & ~PFS_EOFR_Msk;

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			psfval |= (2<<PFS_EOFR_Pos);
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			psfval |= (1<<PFS_EOFR_Pos);
			break;
		case IOPINSENSE_TOGGLE:
			psfval |= (3<<PFS_EOFR_Pos);
			break;
	}

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	*psf = psfval;

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;
}

/**
 * @brief Set I/O pin drive strength option
 *
 * Some hardware allow setting pin drive strength. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * 			PinNo  	: Pin number (up to 32 pins)
 * 			Strength: Pin drive strength
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	int offset = (PinNo + (PortNo << 4)) << 2;
	__IOM uint32_t *psf = (__IOM uint32_t*)(PFS_BASE + offset);
	uint32_t psfval = *psf & ~PFS_DSCR_Msk;

	psfval |= Strength == IOPINSTRENGTH_STRONG ? PFS_DSCR_Msk : (2<<PFS_DSCR_Pos);

	PMISC->PWPR = 0;
	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	*psf = psfval;

	PMISC->PWPR = 0;	// Write disable
	PMISC->PWPR = PMISC_PWPR_B0WI_Msk;
}

/**
 * @brief Set I/O pin speed option
 *
 * Some hardware allow setting pin speed. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @Param	PinNo  	: Pin number (up to 32 pins)
 * @Param	Speed	: Pin speed
 */
void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	// Not available on this Renesas
}


