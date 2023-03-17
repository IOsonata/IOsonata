/**-------------------------------------------------------------------------
@file	iopincfg_sam4l.c

@brief	I/O pin configuration implementation on SAM4L series

The SAM4 has one interrupt handler for each GPIO port.

@author	Hoang Nguyen Hoan
@date	June 1, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "sam4lxxx.h"
#include "coredev/iopincfg.h"

#define IOPIN_MAX_PINOP			(7)

#define IOPIN_MAX_PORT			(3)

#define IOPIN_MAX_INT			(IOPIN_MAX_PORT)

#pragma pack(push, 4)
typedef struct {
	IOPINSENSE Sense;
	IOPinEvtHandler_t SensEvtCB;//IOPinEvtHandler_t SensEvtCB;
    uint16_t PortPinNo;
    void *pCtx;
} IOPINSENS_EVTHOOK;
#pragma pack(pop)

static IOPINSENS_EVTHOOK s_GpIOSenseEvt[IOPIN_MAX_INT + 1] = { {0, NULL}, };

/**
 * @brief Configure individual I/O pin.
 *
 * @Param 	PortNo	: Port number
 * 						SAM4 ports are named A, B, C,...
 * 							0 = A, 1 = B, ...
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						for SAM4
 * 						0 :	GPIO mode
 * 						1-4 : Peripheral A-D
 *
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[PortNo]);

	if (PortNo == -1 || PinNo == -1 || PortNo > IOPIN_MAX_PORT)
		return;

	uint32_t pinmask = 1 << PinNo;

	// Enable peripheral clock
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
		| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_PBCMASK - (uint32_t)SAM4L_PM);
	SAM4L_PM->PM_PBCMASK |= PM_PBCMASK_GPIO;

	if (PinOp > 0 && PinOp <= IOPIN_MAX_PINOP)
	{
		// Configure as peripheral
		reg->GPIO_GPERC = pinmask;
		
		uint8_t psel = PinOp - 1;
		
		if (psel & 1)
		{
			reg->GPIO_PMR0S = pinmask;
		}
		else
		{
			reg->GPIO_PMR0C = pinmask;
		}
		
		if (psel & 2)
		{
			reg->GPIO_PMR1S = pinmask;
		}
		else
		{
			reg->GPIO_PMR1C = pinmask;
		}

		if (psel & 4)
		{
			reg->GPIO_PMR2S = pinmask;
		}
		else
		{
			reg->GPIO_PMR2C = pinmask;
		}
	}
	else
	{
		// Configure as GPIO	
		reg->GPIO_GPERS = pinmask;
		
		if (Dir == IOPINDIR_OUTPUT)
		{
			reg->GPIO_ODERS = pinmask;
		}
		else
		{
			reg->GPIO_ODERC = pinmask;
			reg->GPIO_STERS = pinmask;
		}
	}
	
	switch (Resistor)
	{
		case IOPINRES_FOLLOW:
		case IOPINRES_PULLUP:
			reg->GPIO_PDERC = pinmask;	// Disable pulldown
			reg->GPIO_PUERS = pinmask;	// Enable pullup
			break;
		case IOPINRES_PULLDOWN:
			reg->GPIO_PUERC = pinmask;	// Disable pullup
			reg->GPIO_PDERS = pinmask;	// Enable pulldown
			break;
		case IOPINRES_NONE:
			reg->GPIO_PUERC = pinmask;
			reg->GPIO_PDERC = pinmask;
			break;
	}

	/* SAM4L does not have opendrain setting
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		reg->PIO_MDER = pinmask;
	}
	else
	{
		reg->PIO_MDDR = pinmask;
	}*/
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

	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[PortNo]);

	uint32_t pinmask = 1 << PinNo;

	reg->GPIO_GPERS = pinmask;
	reg->GPIO_PUERC = pinmask;
	reg->GPIO_PDERC = pinmask;
	reg->GPIO_ODERC = pinmask;
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT)
	{
		return;
	}

	int portno = (s_GpIOSenseEvt[IntNo].PortPinNo >> 8) & 0xFF;

	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[portno]);

	uint32_t pinmask = 1 << (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF);

	reg->GPIO_IERC = pinmask;
	NVIC_DisableIRQ(GPIO_0_IRQn + IntNo);
	NVIC_ClearPendingIRQ(GPIO_0_IRQn + IntNo);

    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;
	s_GpIOSenseEvt[IntNo].pCtx = NULL;
}

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 * SAM4L : IntNo is divided by group of 8 pins & port.
 * 			0 - PA 0..7
 * 			1 - PA 8..15
 *			...
 *			4 - PB 0..7
 *
 * @param	IntNo	: Interrupt number. -1 for port event interrupt
 * 			IntPrio : Interrupt priority
 * 			PortNo  : Port number (up to 32 ports)
 * 			PinNo   : Pin number (up to 32 pins)
 * 			Sense   : Sense type of event on the I/O pin
 * 			pEvtCB	: Pointer to callback function when event occurs
 * 			pCtx	: Pointer to data context to pass to handler callback
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT || (IntNo >> 2) != PortNo || (IntNo & 3) != (PinNo >> 3))
	{
		return false;
	}

	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[PortNo]);

	uint32_t pinmask = 1 << PinNo;

	reg->GPIO_GFERS = pinmask;	// Enable glitch filter

	IOPinSetSense(PortNo, PinNo, Sense);
	
    s_GpIOSenseEvt[IntNo].Sense = Sense;
	s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;
	s_GpIOSenseEvt[IntNo].pCtx = pCtx;

	reg->GPIO_IERS = pinmask;
	
	NVIC_ClearPendingIRQ(GPIO_0_IRQn + IntNo);
	NVIC_SetPriority(GPIO_0_IRQn + IntNo, IntPrio);
	NVIC_EnableIRQ(GPIO_0_IRQn + IntNo);

    return true;
}

int IOPinFindAvailInterrupt()
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (s_GpIOSenseEvt[i].SensEvtCB == NULL)
		{
			return i;
		}
	}

	return -1;
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
 * @Param	IntPrio : Interrupt priority
 * @Param	PortNo  : Port number (up to 32 ports)
 * @Param	PinNo   : Pin number (up to 32 pins)
 * @Param	Sense   : Sense type of event on the I/O pin
 * @Param	pEvtCB	: Pointer to callback function when event occurs
 *
 * @return	Interrupt number on success
 * 			-1 on failure.
 */
int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	int intno = IOPinFindAvailInterrupt();

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
	if (PortNo < 0 || PinNo < 0 || PortNo > IOPIN_MAX_PORT || PinNo > 31)
	{
		return;
	}
	
	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[PortNo]);
	uint32_t pinmask = 1 << PinNo;
	reg->GPIO_GFERS = pinmask;	// Enable glitch filter

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			reg->GPIO_IMR0C = pinmask;
			reg->GPIO_IMR1S = pinmask;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			reg->GPIO_IMR0S = pinmask;
			reg->GPIO_IMR1C = pinmask;
			break;
		case IOPINSENSE_TOGGLE:
			reg->GPIO_IMR0C = pinmask;
			reg->GPIO_IMR1C = pinmask;
			break;
	}
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
	GpioPort *reg = (GpioPort *)((uint32_t)&SAM4L_GPIO->GPIO_PORT[PortNo]);
	uint32_t pinmask = 1 << PinNo;
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
	// Not avail
}

// GPIO handler 0 (PA 0..7)
void GPIO_0_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[0].GPIO_IFR & 0xFF;
	
	if (status)
	{
		if (s_GpIOSenseEvt[0].SensEvtCB)
			s_GpIOSenseEvt[0].SensEvtCB(status, s_GpIOSenseEvt[0].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[0].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_0_IRQn);
}

// GPIO handler 1 (PA 8..15)
void GPIO_1_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[0].GPIO_IFR & 0xFF00;
	
	if (status)
	{
		if (s_GpIOSenseEvt[1].SensEvtCB)
			s_GpIOSenseEvt[1].SensEvtCB(status, s_GpIOSenseEvt[1].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[0].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_1_IRQn);
}

// GPIO handler 2 (PA 16..23)
void GPIO_2_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[0].GPIO_IFR & 0xFF0000;
	
	if (status)
	{
		if (s_GpIOSenseEvt[2].SensEvtCB)
			s_GpIOSenseEvt[2].SensEvtCB(status, s_GpIOSenseEvt[2].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[0].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_2_IRQn);
}

// GPIO handler 3 (PA 24..31)
void GPIO_3_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[0].GPIO_IFR & 0xFF000000;
	
	if (status)
	{
		if (s_GpIOSenseEvt[3].SensEvtCB)
			s_GpIOSenseEvt[3].SensEvtCB(status, s_GpIOSenseEvt[3].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[0].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_3_IRQn);
}

// GPIO handler 4 (PB 0..7)
void GPIO_4_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[1].GPIO_IFR & 0xFF;
	
	if (status)
	{
		if (s_GpIOSenseEvt[4].SensEvtCB)
			s_GpIOSenseEvt[4].SensEvtCB(status, s_GpIOSenseEvt[4].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[1].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_4_IRQn);
}

// GPIO handler 5 (PB 8..15)
void GPIO_5_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[1].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[5].SensEvtCB)
			s_GpIOSenseEvt[5].SensEvtCB(status, s_GpIOSenseEvt[5].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[1].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_5_IRQn);
}

// GPIO handler 6 (PB 16..23)
void GPIO_6_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[1].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[6].SensEvtCB)
			s_GpIOSenseEvt[6].SensEvtCB(status, s_GpIOSenseEvt[6].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[1].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_6_IRQn);
}

// GPIO handler 7 (PB 24..31)
void GPIO_7_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[1].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[7].SensEvtCB)
			s_GpIOSenseEvt[7].SensEvtCB(status, s_GpIOSenseEvt[7].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[1].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_7_IRQn);
}

// GPIO handler 8 (PC 0..7)
void GPIO_8_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[2].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[8].SensEvtCB)
			s_GpIOSenseEvt[8].SensEvtCB(status, s_GpIOSenseEvt[8].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[2].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_8_IRQn);
}

// GPIO handler 10 (PC 16..23)
void GPIO_9_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[2].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[9].SensEvtCB)
			s_GpIOSenseEvt[9].SensEvtCB(status, s_GpIOSenseEvt[9].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[2].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_9_IRQn);
}

// GPIO handler 10 (PC 16..23)
void GPIO_10_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[2].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[10].SensEvtCB)
			s_GpIOSenseEvt[10].SensEvtCB(status, s_GpIOSenseEvt[10].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[2].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_10_IRQn);
}

//  GPIO handler 11 (PC 24..31)
void GPIO_11_Handler(void)
{
	uint32_t status = SAM4L_GPIO->GPIO_PORT[2].GPIO_IFR & 0xFF;

	if (status)
	{
		if (s_GpIOSenseEvt[11].SensEvtCB)
			s_GpIOSenseEvt[11].SensEvtCB(status, s_GpIOSenseEvt[11].pCtx);
	}

	SAM4L_GPIO->GPIO_PORT[2].GPIO_IFRC = status;

	NVIC_ClearPendingIRQ(GPIO_11_IRQn);
}
