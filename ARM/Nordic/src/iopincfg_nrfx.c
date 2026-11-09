/**-------------------------------------------------------------------------
@file	iopincfg_nrfx.c

@brief	I/O pin configuration implementation on nRF5x & nRF9x series

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

MIT

Copyright (c) 2011, I-SYST inc., all rights reserved

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

#include "nrf.h"
#include "nrf_peripherals.h"

#include "iopinctrl.h"

// PEI is optional. These are weak references so ordinary GPIO calls
// never extract the PEI matrix or PPI backend from the static HAL archive.
// A PEI application initializes PeiDev_t, which supplies the definitions.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

NRF_GPIO_Type *nRFGpioGetReg(int PortNo);
void __WEAK GPIOTE_IRQHandler(void);

#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#define IOPIN_MAX_INT			(GPIOTE20_GPIOTE_NCHANNELS_SIZE + GPIOTE30_GPIOTE_NCHANNELS_SIZE)
#elif defined(NRF54H20_XXAA)
#define IOPIN_MAX_INT			(GPIOTE130_GPIOTE_NCHANNELS_SIZE)
#else
#ifndef GPIOTE_CH_NUM
#define GPIOTE_CH_NUM	8
#endif

#define IOPIN_MAX_INT			(GPIOTE_CH_NUM)
#endif

// These legacy GPIOTE instance/channel checks apply to the nRF52 family.
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
#define IOPIN_NRF52_GPIOTE	1
#endif


#if defined(NRF54H20_XXAA)
#ifndef NRF_APPLICATION
#error "iopincfg_nrfx.c: nRF54H20 pin interrupts are written for the application core"
#endif
// GPIOTE130 is shared by the cores. The application core has its own
// interrupt line, interrupt enable register group and port event. Same
// assignment as nrfx (bsp/stable/soc/nrfx_mdk_fixups.h).
#ifdef NRF_TRUSTZONE_NONSECURE
#define H20_GPIOTE_IRQn			GPIOTE130_0_IRQn
#define H20_GPIOTE_IRQHandler	GPIOTE130_0_IRQHandler
#define H20_GPIOTE_INTENSET		INTENSET2
#define H20_GPIOTE_INTENCLR		INTENCLR2
#define H20_GPIOTE_PORT_Msk		GPIOTE_INTENSET2_PORT1NONSECURE_Msk
#define H20_GPIOTE_EVENTS_PORT	EVENTS_PORT[1].NONSECURE
#else
#define H20_GPIOTE_IRQn			GPIOTE130_1_IRQn
#define H20_GPIOTE_IRQHandler	GPIOTE130_1_IRQHandler
#define H20_GPIOTE_INTENSET		INTENSET3
#define H20_GPIOTE_INTENCLR		INTENCLR3
#define H20_GPIOTE_PORT_Msk		GPIOTE_INTENSET3_PORT1SECURE_Msk
#define H20_GPIOTE_EVENTS_PORT	EVENTS_PORT[1].SECURE
#endif
#endif

#pragma pack(push, 4)
typedef struct {
	IOPINSENSE Sense;
	IOPinEvtHandler_t SensEvtCB;
    uint16_t PortPinNo;
    void *pCtx;
} PinSenseEvtHook_t;
#pragma pack(pop)

__ALIGN(4) static PinSenseEvtHook_t s_GpIOSenseEvt[IOPIN_MAX_INT + 1] = { {0, NULL}, };

// Set by IOPinEnableInterrupt. The interrupt handlers at the end of this file
// are kept by the linker in every application, pin interrupt used or not.
// They reach the table above through this pointer, so that the table is
// linked only when a pin interrupt is set up.
static PinSenseEvtHook_t *s_pGpIOSenseEvt = NULL;

// Hardware GPIO task channels share the same GPIOTE channel pool as
// pin interrupts. One bitmap prevents the two users from overlapping.
static uint32_t s_GpIOTaskUsed;
static uint32_t s_GpIOEventUsed;

NRF_GPIO_Type *nRFGpioGetReg(int PortNo)
{
	NRF_GPIO_Type *reg = NULL;

#if defined(NRF54H20_XXAA)
	// Ports 0, 1, 2, 6, 7 and 9. The MDK names select the secure or non secure
	// instance.
	switch (PortNo)
	{
		case 0:
			reg = NRF_P0;
			break;
		case 1:
			reg = NRF_P1;
			break;
		case 2:
			reg = NRF_P2;
			break;
		case 6:
			reg = NRF_P6;
			break;
		case 7:
			reg = NRF_P7;
			break;
		case 9:
			reg = NRF_P9;
			break;
	}
#else
	if (PortNo == -1 || PortNo >= GPIO_COUNT)
		return NULL;

	switch (PortNo)
	{
		case 0:
#if defined(NRF51) || defined(NRF52_SERIES)
			reg = NRF_GPIO;
#elif defined(NRF5340_XXAA_NETWORK)
			reg = NRF_P0_NS;
#else
			if (PortNo & 0x80)
			{
				reg = NRF_P0_NS;
			}
			else
			{
				reg = NRF_P0_S;
			}
#endif
			break;
#if GPIO_COUNT > 1
		case 1:
#if defined(NRF52840_XXAA)
			reg = NRF_P1;
#elif defined(NRF5340_XXAA_NETWORK)
			reg = NRF_P1_NS;
#else
			if (PortNo & 0x80)
			{
				reg = NRF_P1_NS;
			}
			else
			{
				reg = NRF_P1_S;
			}
#endif
			break;
#endif
#if GPIO_COUNT > 2
		case 2:
#if defined(NRF5340_XXAA_NETWORK)
			reg = NRF_P2_NS;
#else
			if (PortNo & 0x80)
			{
				reg = NRF_P2_NS;
			}
			else
			{
				reg = NRF_P2_S;
			}
#endif
			break;
#endif
#if GPIO_COUNT > 3
		case 3:
#if defined(NRF54H20_XXAA_NETWORK)
			reg = NRF_P3_NS;
#else
			if (PortNo & 0x80)
			{
				reg = NRF_P3_NS;
			}
			else
			{
				reg = NRF_P3_S;
			}
#endif
			break;
#endif
	}
#endif

	return reg;
}

static NRF_GPIOTE_Type *nRFGpioteGetReg(int PortNo)
{
	NRF_GPIOTE_Type *reg = NULL;

#if defined(NRF54H20_XXAA)
	// One instance for all ports
	(void)PortNo;
	reg = NRF_GPIOTE130;
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	if (PortNo & 0x7F)
	{
	    if (PortNo & 0x80)
	    {
	    	reg = NRF_GPIOTE20_NS;
	    }
	    else
	    {
	    	reg = NRF_GPIOTE20_S;
	    }
	}
	else
	{
	    if (PortNo & 0x80)
	    {
	    	reg = NRF_GPIOTE30_NS;
	    }
	    else
	    {
	    	reg = NRF_GPIOTE30_S;
	    }
	}
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	reg = NRF_GPIOTE_NS;
#else
	reg = NRF_GPIOTE0_S;
	if (PortNo & 0x80)
	{
		reg = NRF_GPIOTE1_NS;
	}
#endif
#else
    reg = NRF_GPIOTE;
#endif

    return reg;
}

/**
 * @brief Configure individual I/O pin. nRF51 only have 1 port so PortNo is not used
 *
 * @Param 	PortNo	: Port number
 * 						Special port flag 0x80 (bit 7 set) for nRF91 secure port
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						ignore for Nordic
 *
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	uint32_t cnf = 0;
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	if (reg == NULL || PinNo == -1)
	{
		return;
	}

	if (Dir == IOPINDIR_OUTPUT)
	{
		cnf |= (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
               | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	}
	else
	{
		cnf |= (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			   | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
	}

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:	// nRF51 does not have follow mode, use pullup
		case IOPINRES_PULLUP:
			cnf |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
			break;
		case IOPINRES_PULLDOWN:
			cnf |= (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
			break;
		case IOPINRES_NONE:
			cnf |= (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
			break;
	}

	if (Type == IOPINTYPE_OPENDRAIN)
	{
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		cnf |= (GPIO_PIN_CNF_DRIVE0_S0 << GPIO_PIN_CNF_DRIVE0_Pos) | (GPIO_PIN_CNF_DRIVE1_D1 << GPIO_PIN_CNF_DRIVE1_Pos);
#else
		cnf |= (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
#endif
	}

#ifdef GPIO_PIN_CNF_MCUSEL_Msk
	// Which core owns the pin is decided by the application core and handed
	// over before the other core runs. A pin configuration from either core
	// must leave that choice alone, or the pin falls back to the application
	// core and goes silent on the one that drives it.
	cnf |= reg->PIN_CNF[PinNo] & GPIO_PIN_CNF_MCUSEL_Msk;
#endif
	reg->PIN_CNF[PinNo] = cnf;
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
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	if (reg == NULL || PinNo == -1)
	{
		return;
	}

	uint32_t cnf = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

#ifdef GPIO_PIN_CNF_MCUSEL_Msk
	cnf |= reg->PIN_CNF[PinNo] & GPIO_PIN_CNF_MCUSEL_Msk;
#endif
	reg->PIN_CNF[PinNo] = cnf;
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
    if (IntNo >= IOPIN_MAX_INT)
        return;

#if defined(IOPIN_NRF52_GPIOTE)
	// Never clear an unused IRQ slot or a PEI-owned GPIOTE channel.
	if (IntNo >= 0 &&
		(s_GpIOSenseEvt[IntNo].Sense == IOPINSENSE_DISABLE ||
		 NRF_GPIOTE->CONFIG[IntNo] == 0U))
	{
		return;
	}
#endif

	// A GPIOTE hardware task is not an interrupt and must never be
	// released through the interrupt API.
	if (IntNo >= 0 &&
		((s_GpIOTaskUsed | s_GpIOEventUsed) & (1UL << IntNo)) != 0U)
	{
		return;
	}

	// A negative IntNo is the port event, the last entry
	NRF_GPIOTE_Type *gpiotereg = nRFGpioteGetReg(s_GpIOSenseEvt[IntNo < 0 ? IOPIN_MAX_INT : IntNo].PortPinNo >> 8);

    if (IntNo < 0)
    {
        IntNo = IOPIN_MAX_INT;
#if defined(NRF54H20_XXAA)
		gpiotereg->H20_GPIOTE_INTENCLR = H20_GPIOTE_PORT_Msk;
		gpiotereg->H20_GPIOTE_EVENTS_PORT = 0;
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
        if (s_GpIOSenseEvt[IntNo].PortPinNo & 0x8000)
        {
        	if (IntNo > 4)
        	{
				gpiotereg->INTENCLR1 = GPIOTE_INTENCLR0_PORT0NONSECURE_Msk;
				gpiotereg->EVENTS_PORT[1].NONSECURE = 0;
        	}
        	else
        	{
				gpiotereg->INTENCLR0 = GPIOTE_INTENCLR0_PORT0NONSECURE_Msk;
				gpiotereg->EVENTS_PORT[0].NONSECURE = 0;
        	}
        }
        else
        {
        	if (IntNo > 4)
        	{
        		gpiotereg->INTENCLR1 = GPIOTE_INTENCLR0_PORT0SECURE_Msk;
    			gpiotereg->EVENTS_PORT[1].SECURE = 0;
        	}
        	else
        	{
        		gpiotereg->INTENCLR0 = GPIOTE_INTENCLR0_PORT0SECURE_Msk;
    			gpiotereg->EVENTS_PORT[0].SECURE = 0;
        	}
        }
#else
        gpiotereg->INTENCLR = GPIOTE_INTENSET_PORT_Msk;
        gpiotereg->EVENTS_PORT = 0;
#endif
    }
    else
    {
    	NRF_GPIO_Type *reg = nRFGpioGetReg(s_GpIOSenseEvt[IntNo].PortPinNo >> 8);

#if defined(NRF54H20_XXAA)
		gpiotereg->H20_GPIOTE_INTENCLR = (1 << IntNo);
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
    	if (IntNo > 4)
    	{
    		gpiotereg->INTENCLR1 = (1 << (IntNo - 4));
    	}
    	else
    	{
    		gpiotereg->INTENCLR0 = (1 << IntNo);
    	}
#else
    	gpiotereg->INTENCLR = (1 << IntNo);
#endif
        gpiotereg->CONFIG[IntNo] = 0;
#if defined(IOPIN_NRF52_GPIOTE)
		// Do not leave an old edge pending when this slot is reused.
		gpiotereg->EVENTS_IN[IntNo] = 0U;
#endif
        reg->PIN_CNF[s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF] &= ~GPIO_PIN_CNF_SENSE_Msk;
    }

    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;
    s_GpIOSenseEvt[IntNo].pCtx = NULL;

    for (int i = 0; i <= IOPIN_MAX_INT; i++)
    {
        if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
            return;
    }

//#if defined(NRF91_SERIES) || defined(NRF53_SERIES) ||
#if defined(NRF54H20_XXAA)
	NVIC_ClearPendingIRQ(H20_GPIOTE_IRQn);
	NVIC_DisableIRQ(H20_GPIOTE_IRQn);
	gpiotereg->H20_GPIOTE_INTENCLR = 0xFFFFFFFF;
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
    if (s_GpIOSenseEvt[IntNo].PortPinNo & 0x7F00)
    {
		if (s_GpIOSenseEvt[IntNo].PortPinNo & 0x8000)
		{
			NVIC_ClearPendingIRQ(GPIOTE20_0_IRQn);
			NVIC_DisableIRQ(GPIOTE20_0_IRQn);
		}
		else
		{
			NVIC_ClearPendingIRQ(GPIOTE20_1_IRQn);
			NVIC_DisableIRQ(GPIOTE20_1_IRQn);
		}
		gpiotereg->INTENCLR1 = 0xFFFFFFFF;
    }
    else
    {
		if (s_GpIOSenseEvt[IntNo].PortPinNo & 0x8000)
		{
			NVIC_ClearPendingIRQ(GPIOTE30_0_IRQn);
			NVIC_DisableIRQ(GPIOTE30_0_IRQn);
		}
		else
		{
			NVIC_ClearPendingIRQ(GPIOTE30_1_IRQn);
			NVIC_DisableIRQ(GPIOTE30_1_IRQn);
		}
		gpiotereg->INTENCLR1 = 0xFFFFFFFF;
    }

#else
#ifdef NRF5340_XXAA_NETWORK
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_DisableIRQ(GPIOTE_IRQn);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	// GPIOTE1 is the non secure instance, GPIOTE0 the secure one. PortPinNo
	// is already cleared here, the instance in use tells which one.
	if (gpiotereg == NRF_GPIOTE1_NS)
	{
		NVIC_ClearPendingIRQ(GPIOTE1_IRQn);
		NVIC_DisableIRQ(GPIOTE1_IRQn);
	}
	else
	{
		NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
		NVIC_DisableIRQ(GPIOTE0_IRQn);
	}
#else
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_DisableIRQ(GPIOTE_IRQn);
#endif

    gpiotereg->INTENCLR = 0xFFFFFFFF;
#endif
}

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 * NOTE : Port event interrupt is set when IntNo = -1.  Port event mode only
 * high transition is detected no matter the setting of pin sense
 *
 * @param	IntNo	: Interrupt number. -1 for port event interrupt
 * 			IntPrio : Interrupt priority
 * 			PortNo  : Port number (up to 32 ports). Bit 7 (0x80) set for Non Secure
 * 			PinNo   : Pin number (up to 32 pins). In port interrupt, this
 * 					  parameter contains pin mask instead of pin number
 * 			Sense   : Sense type of event on the I/O pin
 * 			pEvtCB	: Pointer to callback function when event occurs
 * 			pCtx	: Pointer to context data to be pass to the handler function
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
#if defined(IOPIN_NRF52_GPIOTE)
	uint32_t irqState = 0U;
	uint32_t otherCfg;
	uint32_t otherPin;
	uint32_t otherPort;
	int channelCount;
	int i;
	bool irqLocked = false;
#endif

	s_pGpIOSenseEvt = s_GpIOSenseEvt;

	if (IntNo < -1 || IntNo >= IOPIN_MAX_INT)
		return false;

#ifdef NRF5340_XXAA_NETWORK
	if ((PortNo >= GPIO_COUNT) || (IntNo >= 0 && PinNo >= 32))
		return false;
#endif

    NRF_GPIOTE_Type *gpiotereg = nRFGpioteGetReg(PortNo);
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

#if defined(NRF5340_XXAA_NETWORK) || defined(NRF54H20_XXAA)
	if (gpiotereg == NULL || reg == NULL)
		return false;
#endif

#ifdef GPIOTE_CONFIG_PORT_Msk
#define GPIOTE_CONFIG_PORT_PIN_Msk (GPIOTE_CONFIG_PORT_Msk | GPIOTE_CONFIG_PSEL_Msk)
#else
#define GPIOTE_CONFIG_PORT_PIN_Msk GPIOTE_CONFIG_PSEL_Msk
#endif

	uint32_t cfg = 0;
#if defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	// Two GPIOTE instances of 4 channels each, the second one starts over at 0
	int idx = IntNo > 3 ? IntNo - 4 : IntNo;
#else
	// One instance, so the channel is the interrupt number. Folding 4 to 7 back
	// onto 0 to 3 here made those four allocations program the config of a
	// channel they did not own while enabling the interrupt of one they did.
	int idx = IntNo;
#endif

	if (IntNo >= 0 &&
		((s_GpIOTaskUsed | s_GpIOEventUsed) & (1UL << IntNo)) != 0U)
	{
		return false;
	}

#if defined(IOPIN_NRF52_GPIOTE)
	// A conventional IRQ and PEI share the same physical GPIOTE
	// channels. Never overwrite another owner, or claim the same pin
	// through a different channel. Keep this check and setup atomic.
	if (IntNo >= 0)
	{
		if (reg == NULL || gpiotereg == NULL ||
			PortNo >= GPIO_COUNT || PinNo >= 32U ||
			Sense == IOPINSENSE_DISABLE)
		{
			return false;
		}
		irqState = DisableInterrupt();
		irqLocked = true;
		if (s_GpIOSenseEvt[IntNo].Sense != IOPINSENSE_DISABLE ||
			((s_GpIOTaskUsed | s_GpIOEventUsed) & (1UL << IntNo)) != 0U ||
			gpiotereg->CONFIG[IntNo] != 0U)
		{
			EnableInterrupt(irqState);
			return false;
		}
		channelCount = sizeof(gpiotereg->CONFIG) /
			sizeof(gpiotereg->CONFIG[0]);
		for (i = 0; i < channelCount; i++)
		{
			otherCfg = gpiotereg->CONFIG[i];
			if ((otherCfg & GPIOTE_CONFIG_MODE_Msk) == 0U)
			{
				continue;
			}
			otherPin = (otherCfg & GPIOTE_CONFIG_PSEL_Msk) >>
				GPIOTE_CONFIG_PSEL_Pos;
#ifdef GPIOTE_CONFIG_PORT_Msk
			otherPort = (otherCfg & GPIOTE_CONFIG_PORT_Msk) >>
				GPIOTE_CONFIG_PORT_Pos;
#else
			otherPort = 0U;
#endif
			if (otherPort == PortNo && otherPin == PinNo)
			{
				EnableInterrupt(irqState);
				return false;
			}
		}
	}
#endif

	if (IntNo < 0)
	{
		IntNo = IOPIN_MAX_INT;
#if defined(NRF54H20_XXAA)
		gpiotereg->H20_GPIOTE_EVENTS_PORT = 0;
		gpiotereg->H20_GPIOTE_INTENSET = H20_GPIOTE_PORT_Msk;
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)

		if (IntNo > 3)
		{
			if (PortNo & 0x80)
			{
				gpiotereg->EVENTS_PORT[1].NONSECURE = 0;
				gpiotereg->INTENSET1 = GPIOTE_INTENSET1_PORT0NONSECURE_Msk;
			}
			else
			{
				gpiotereg->EVENTS_PORT[1].SECURE = 0;
				gpiotereg->INTENSET1 = GPIOTE_INTENSET1_PORT0SECURE_Msk;
			}
		}
		else
		{
			if (PortNo & 0x80)
			{
				gpiotereg->EVENTS_PORT[0].NONSECURE = 0;
				gpiotereg->INTENSET0 = GPIOTE_INTENSET0_PORT0NONSECURE_Msk;
			}
			else
			{
				gpiotereg->EVENTS_PORT[0].SECURE = 0;
				gpiotereg->INTENSET0 = GPIOTE_INTENSET0_PORT0SECURE_Msk;
			}
		}
#else
		gpiotereg->EVENTS_PORT = 0;
		gpiotereg->INTENSET = GPIOTE_INTENSET_PORT_Msk;
#endif
		for (int i = 0; i <= 31; i++)
		{
			if ((1<<i) & PinNo)
			{
				reg->PIN_CNF[i] &= ~GPIO_PIN_CNF_SENSE_Msk;
				switch (Sense)
				{
					case IOPINSENSE_LOW_TRANSITION:
						cfg = ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
													| (((i | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
													| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
						reg->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
						break;
					case IOPINSENSE_HIGH_TRANSITION:
						cfg = ((GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
													| (((i | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
													| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
						reg->PIN_CNF[i] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
						break;
					case IOPINSENSE_TOGGLE:
						cfg = ((GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
													| (((i | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
													| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
						reg->PIN_CNF[i] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
						break;
					default:
						;
				}

				// A port event uses the pin sense, not a GPIOTE channel: idx is
				// negative here and there is no channel to configure
				if (idx >= 0)
				{
					gpiotereg->CONFIG[idx] = cfg;
				}

				s_GpIOSenseEvt[IntNo].Sense = Sense;
				s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | i; // For use when disable interrupt
				s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;
				s_GpIOSenseEvt[IntNo].pCtx = pCtx;
			}
		}
	}
	else
	{
		reg->PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_SENSE_Msk;
		switch (Sense)
		{
			case IOPINSENSE_LOW_TRANSITION:
				cfg = ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
											| (((PinNo | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
											| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
				reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
				break;
			case IOPINSENSE_HIGH_TRANSITION:
				cfg = ((GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
											| (((PinNo | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
											| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
				reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
				break;
			case IOPINSENSE_TOGGLE:
				cfg = ((GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
											| (((PinNo | (PortNo << 5)) << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
											| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
				reg->PIN_CNF[PinNo] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
				break;
			default:
				;
		}

		gpiotereg->CONFIG[idx] = cfg;
#if defined(IOPIN_NRF52_GPIOTE)
		// Prevent stale events from firing immediately after reuse.
		gpiotereg->EVENTS_IN[idx] = 0U;
#endif

#if defined(NRF54H20_XXAA)
		gpiotereg->H20_GPIOTE_INTENSET = (1 << IntNo);
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
		if (IntNo > 3)
		{
			if (PortNo & 0x80)
			{
				gpiotereg->INTENSET1 |= (1 << idx) | GPIOTE_INTENCLR1_PORT0NONSECURE_Msk;
			}
			else
			{
				gpiotereg->INTENSET1 |= (1 << idx) | GPIOTE_INTENCLR1_PORT0SECURE_Msk;
			}
		}
		else
		{
			if (PortNo & 0x80)
			{
				gpiotereg->INTENSET0 |= (1 << IntNo) | GPIOTE_INTENCLR1_PORT0NONSECURE_Msk;
			}
			else
			{
				gpiotereg->INTENSET0 |= (1 << IntNo) | GPIOTE_INTENCLR1_PORT0SECURE_Msk;
			}
		}

#else
		gpiotereg->INTENSET = (1 << IntNo);
#endif
	    s_GpIOSenseEvt[IntNo].Sense = Sense;
		s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
		s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;
		s_GpIOSenseEvt[IntNo].pCtx = pCtx;
	}

//#if defined(NRF91_SERIES) || defined(NRF53_SERIES) || defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#ifdef NRF5340_XXAA_NETWORK
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, IntPrio);
    NVIC_EnableIRQ(GPIOTE_IRQn);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	// GPIOTE1 is the non secure instance, GPIOTE0 the secure one
	if (PortNo & 0x80)
	{
		NVIC_ClearPendingIRQ(GPIOTE1_IRQn);
		NVIC_SetPriority(GPIOTE1_IRQn, IntPrio);
		NVIC_EnableIRQ(GPIOTE1_IRQn);
	}
	else
	{
		NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
		NVIC_SetPriority(GPIOTE0_IRQn, IntPrio);
		NVIC_EnableIRQ(GPIOTE0_IRQn);
	}
#elif defined(NRF54H20_XXAA)
	NVIC_ClearPendingIRQ(H20_GPIOTE_IRQn);
	NVIC_SetPriority(H20_GPIOTE_IRQn, IntPrio);
	NVIC_EnableIRQ(H20_GPIOTE_IRQn);
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	if ((PortNo & 0x7F) == 0)
	{
		if (PortNo & 0x80)
		{
			// Non secure
			NVIC_ClearPendingIRQ(GPIOTE30_0_IRQn);
			NVIC_SetPriority(GPIOTE30_0_IRQn, IntPrio);
			NVIC_EnableIRQ(GPIOTE30_0_IRQn);
		}
		else
		{
			// secure
			NVIC_ClearPendingIRQ(GPIOTE30_1_IRQn);
			NVIC_SetPriority(GPIOTE30_1_IRQn, IntPrio);
			NVIC_EnableIRQ(GPIOTE30_1_IRQn);
		}
	}
	else
	{
		if (PortNo & 0x80)
		{
			// Non secure
		    NVIC_ClearPendingIRQ(GPIOTE20_0_IRQn);
		    NVIC_SetPriority(GPIOTE20_0_IRQn, IntPrio);
		    NVIC_EnableIRQ(GPIOTE20_0_IRQn);
		}
		else
		{
			// secure
		    NVIC_ClearPendingIRQ(GPIOTE20_1_IRQn);
		    NVIC_SetPriority(GPIOTE20_1_IRQn, IntPrio);
		    NVIC_EnableIRQ(GPIOTE20_1_IRQn);
		}
	}
#else
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, IntPrio);
    NVIC_EnableIRQ(GPIOTE_IRQn);
#endif
#if defined(IOPIN_NRF52_GPIOTE)
	if (irqLocked)
	{
		EnableInterrupt(irqState);
	}
#endif
    return true;
}

// GPIO output task setup belongs to this driver. PEI connects the
// prepared endpoint but does not configure the pin or a GPIOTE channel.
static int IOPinOpenTask(int PortNo, int PinNo, IOPINTASK Task,
	PeiEndpoint_t *pEndpoint)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	NRF_GPIOTE_Type *gpiotereg;
	NRF_GPIO_Type *gpioreg;
	uint32_t state;
	uint32_t cfg;
	uint32_t polarity;
	uint32_t outinit;
	uint32_t mask;
	uint32_t old;
	uint32_t oldMode;
	uint32_t oldPin;
	uint32_t oldPort;
	int i;
	int chan;
	int channelCount;

	if (pEndpoint == NULL || PinNo < 0 || PinNo >= 32 ||
		PortNo < 0 || PortNo >= GPIO_COUNT)
	{
		return -1;
	}
	gpiotereg = nRFGpioteGetReg(PortNo);
	gpioreg = nRFGpioGetReg(PortNo);
	if (gpiotereg == NULL || gpioreg == NULL)
	{
		return -1;
	}
	switch (Task)
	{
		case IOPIN_TASK_TOGGLE:
			polarity = GPIOTE_CONFIG_POLARITY_Toggle;
			break;
		case IOPIN_TASK_SET:
			polarity = GPIOTE_CONFIG_POLARITY_LoToHi;
			break;
		case IOPIN_TASK_CLEAR:
			polarity = GPIOTE_CONFIG_POLARITY_HiToLo;
			break;
		default:
			return -1;
	}
	outinit = (gpioreg->OUT & (1UL << PinNo)) != 0U ?
		GPIOTE_CONFIG_OUTINIT_High : GPIOTE_CONFIG_OUTINIT_Low;
	cfg = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
		((polarity << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
		((outinit << GPIOTE_CONFIG_OUTINIT_Pos) & GPIOTE_CONFIG_OUTINIT_Msk) |
		(((uint32_t)PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
#ifdef GPIOTE_CONFIG_PORT_Msk
	cfg |= (((uint32_t)PortNo << GPIOTE_CONFIG_PORT_Pos) & GPIOTE_CONFIG_PORT_Msk);
#endif

	chan = -1;
	channelCount = sizeof(gpiotereg->CONFIG) / sizeof(gpiotereg->CONFIG[0]);
	if (channelCount > IOPIN_MAX_INT)
	{
		channelCount = IOPIN_MAX_INT;
	}
	state = DisableInterrupt();

	// Reject an existing GPIOTE assignment of this same pin, even if it
	// occupies another channel or is owned by code outside this driver.
	for (i = 0; i < channelCount; i++)
	{
		old = gpiotereg->CONFIG[i];
		oldMode = old & GPIOTE_CONFIG_MODE_Msk;
		oldPin = (old & GPIOTE_CONFIG_PSEL_Msk) >> GPIOTE_CONFIG_PSEL_Pos;
#ifdef GPIOTE_CONFIG_PORT_Msk
		oldPort = (old & GPIOTE_CONFIG_PORT_Msk) >> GPIOTE_CONFIG_PORT_Pos;
#else
		oldPort = 0U;
#endif
		if (oldMode != 0U && oldPin == (uint32_t)PinNo &&
			oldPort == (uint32_t)PortNo)
		{
			EnableInterrupt(state);
			return -1;
		}
	}
	for (i = 0; i < channelCount; i++)
	{
		mask = 1UL << i;
		if (((s_GpIOTaskUsed | s_GpIOEventUsed) & mask) == 0U &&
			s_GpIOSenseEvt[i].SensEvtCB == NULL &&
			gpiotereg->CONFIG[i] == 0U)
		{
			s_GpIOTaskUsed |= mask;
			gpiotereg->CONFIG[i] = cfg;
			chan = i;
			break;
		}
	}
	EnableInterrupt(state);

	if (chan < 0)
	{
		return -1;
	}
	*pEndpoint = (PeiEndpoint_t)&gpiotereg->TASKS_OUT[chan];
	return chan;
#else
	(void)PortNo;
	(void)PinNo;
	(void)Task;
	(void)pEndpoint;
	return -1;
#endif
}

static bool IOPinCloseTask(int TaskNo)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	uint32_t state;
	uint32_t bit;

	if (TaskNo < 0 || TaskNo >= IOPIN_MAX_INT ||
		TaskNo >= (int)(sizeof(NRF_GPIOTE->CONFIG) / sizeof(NRF_GPIOTE->CONFIG[0])))
	{
		return false;
	}
	bit = 1UL << TaskNo;
	state = DisableInterrupt();
	if ((s_GpIOTaskUsed & bit) == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	nRFGpioteGetReg(0)->CONFIG[TaskNo] = 0U;
	s_GpIOTaskUsed &= ~bit;
	EnableInterrupt(state);
	return true;
#else
	(void)TaskNo;
	return false;
#endif
}

// Hardware edge events are prepared by the GPIO driver. PEI only routes
// the resulting event endpoint to a peripheral task.
static int IOPinOpenEvent(int PortNo, int PinNo, IOPINSENSE Sense,
	PeiEndpoint_t *pEndpoint)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	NRF_GPIOTE_Type *gpiotereg;
	uint32_t state;
	uint32_t polarity;
	uint32_t cfg;
	uint32_t mask;
	uint32_t old;
	uint32_t oldMode;
	uint32_t oldPin;
	uint32_t oldPort;
	int i;
	int chan;
	int channelCount;

	if (pEndpoint == NULL || PinNo < 0 || PinNo >= 32 ||
		PortNo < 0 || PortNo >= GPIO_COUNT)
	{
		return -1;
	}
	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			polarity = GPIOTE_CONFIG_POLARITY_HiToLo;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			polarity = GPIOTE_CONFIG_POLARITY_LoToHi;
			break;
		case IOPINSENSE_TOGGLE:
			polarity = GPIOTE_CONFIG_POLARITY_Toggle;
			break;
		default:
			return -1;
	}
	gpiotereg = nRFGpioteGetReg(PortNo);
	if (gpiotereg == NULL)
	{
		return -1;
	}
	cfg = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
		((polarity << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
		(((uint32_t)PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
#ifdef GPIOTE_CONFIG_PORT_Msk
	cfg |= (((uint32_t)PortNo << GPIOTE_CONFIG_PORT_Pos) & GPIOTE_CONFIG_PORT_Msk);
#endif

	chan = -1;
	channelCount = sizeof(gpiotereg->CONFIG) / sizeof(gpiotereg->CONFIG[0]);
	if (channelCount > (int)(sizeof(gpiotereg->EVENTS_IN) /
		sizeof(gpiotereg->EVENTS_IN[0])))
	{
		channelCount = sizeof(gpiotereg->EVENTS_IN) /
			sizeof(gpiotereg->EVENTS_IN[0]);
	}
	if (channelCount > IOPIN_MAX_INT)
	{
		channelCount = IOPIN_MAX_INT;
	}
	state = DisableInterrupt();

	// Never claim a pin that another GPIOTE channel already owns.
	for (i = 0; i < channelCount; i++)
	{
		old = gpiotereg->CONFIG[i];
		oldMode = old & GPIOTE_CONFIG_MODE_Msk;
		oldPin = (old & GPIOTE_CONFIG_PSEL_Msk) >> GPIOTE_CONFIG_PSEL_Pos;
#ifdef GPIOTE_CONFIG_PORT_Msk
		oldPort = (old & GPIOTE_CONFIG_PORT_Msk) >> GPIOTE_CONFIG_PORT_Pos;
#else
		oldPort = 0U;
#endif
		if (oldMode != 0U && oldPin == (uint32_t)PinNo &&
			oldPort == (uint32_t)PortNo)
		{
			EnableInterrupt(state);
			return -1;
		}
	}
	for (i = 0; i < channelCount; i++)
	{
		mask = 1UL << i;
		if (((s_GpIOTaskUsed | s_GpIOEventUsed) & mask) == 0U &&
			s_GpIOSenseEvt[i].SensEvtCB == NULL &&
			gpiotereg->CONFIG[i] == 0U)
		{
			s_GpIOEventUsed |= mask;
			gpiotereg->INTENCLR = mask;
			gpiotereg->EVENTS_IN[i] = 0U;
			gpiotereg->CONFIG[i] = cfg;
			chan = i;
			break;
		}
	}
	EnableInterrupt(state);

	if (chan < 0)
	{
		return -1;
	}
	*pEndpoint = (PeiEndpoint_t)&gpiotereg->EVENTS_IN[chan];
	return chan;
#else
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
	(void)pEndpoint;
	return -1;
#endif
}

static bool IOPinCloseEvent(int EventNo)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	uint32_t bit;
	uint32_t state;

	if (EventNo < 0 || EventNo >= IOPIN_MAX_INT ||
		EventNo >= (int)(sizeof(NRF_GPIOTE->CONFIG) / sizeof(NRF_GPIOTE->CONFIG[0])))
	{
		return false;
	}
	bit = 1UL << EventNo;
	state = DisableInterrupt();
	if ((s_GpIOEventUsed & bit) == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	NRF_GPIOTE->CONFIG[EventNo] = 0U;
	NRF_GPIOTE->EVENTS_IN[EventNo] = 0U;
	s_GpIOEventUsed &= ~bit;
	EnableInterrupt(state);
	return true;
#else
	(void)EventNo;
	return false;
#endif
}

// C API for both C applications and the C++ Pei wrapper.
// All GPIOTE resource allocation stays inside this GPIO driver.
bool IOPinEnableEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	int PortNo, int PinNo, IOPINSENSE Sense)
{
	PeiEndpoint_t endpoint;
	int channel;

	if (pPei == NULL || pEvt == NULL || PeiBindEvent == NULL)
	{
		return false;
	}
	channel = IOPinOpenEvent(PortNo, PinNo, Sense, &endpoint);
	if (channel < 0)
	{
		return false;
	}
	if (!PeiBindEvent(pPei, pEvt, endpoint))
	{
		IOPinCloseEvent(channel);
		return false;
	}
	return true;
}

bool IOPinDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	PeiEndpoint_t endpoint;
	NRF_GPIOTE_Type *gpiotereg;
	int count;
	int i;

	if (pPei == NULL || PeiGetEventEndpoint == NULL ||
		PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = PeiGetEventEndpoint(pPei, Evt);
	gpiotereg = NRF_GPIOTE;
	if (endpoint == 0U)
	{
		return false;
	}
	count = sizeof(gpiotereg->EVENTS_IN) / sizeof(gpiotereg->EVENTS_IN[0]);
	if (count > IOPIN_MAX_INT)
	{
		count = IOPIN_MAX_INT;
	}
	for (i = 0; i < count; i++)
	{
		if ((s_GpIOEventUsed & (1UL << i)) == 0U ||
			endpoint != (PeiEndpoint_t)&gpiotereg->EVENTS_IN[i])
		{
			continue;
		}
		if (!PeiUnbindEvent(pPei, Evt, endpoint))
		{
			return false;
		}
		return IOPinCloseEvent(i);
	}
#endif
	return false;
}

// GPIOTE PORT is a shared source for GPIO SENSE/DETECT inputs.
// It has no dedicated pin or GPIOTE IN channel. The application owns
// pin SENSE and any existing PORT interrupt configuration.
bool IOPinEnablePortEvent(PeiDev_t *pPei, PeiEvent_t *pEvt)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	if (pPei == NULL || pEvt == NULL || PeiBindEvent == NULL)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt,
		(PeiEndpoint_t)&NRF_GPIOTE->EVENTS_PORT);
#else
	(void)pPei;
	(void)pEvt;
	return false;
#endif
}

// Unbind only this PEI source. GPIO SENSE, LATCH, PORT interrupts and
// other hardware sources or destinations retain their own ownership.
bool IOPinDisablePortEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiGetEventEndpoint == NULL ||
		PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = (PeiEndpoint_t)&NRF_GPIOTE->EVENTS_PORT;
	if (PeiGetEventEndpoint(pPei, Evt) != endpoint)
	{
		return false;
	}
	return PeiUnbindEvent(pPei, Evt, endpoint);
#else
	(void)pPei;
	(void)Evt;
	return false;
#endif
}

bool IOPinEnableTask(PeiDev_t *pPei, PeiEvent_t Evt,
	int PortNo, int PinNo, IOPINTASK Task)
{
	PeiEndpoint_t endpoint;
	int channel;

	if (pPei == NULL || PeiConnectTask == NULL)
	{
		return false;
	}
	channel = IOPinOpenTask(PortNo, PinNo, Task, &endpoint);
	if (channel < 0)
	{
		return false;
	}
	if (!PeiConnectTask(pPei, Evt, endpoint))
	{
		IOPinCloseTask(channel);
		return false;
	}
	return true;
}

bool IOPinDisableTask(PeiDev_t *pPei, PeiEvent_t Evt,
	int PortNo, int PinNo)
{
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA) || \
	defined(NRF52820_XXAA) || defined(NRF52832_XXAA) || \
	defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
	NRF_GPIOTE_Type *gpiotereg;
	uint32_t config;
	uint32_t pin;
	uint32_t port;
	int channelCount;
	int i;

	if (pPei == NULL || PeiDisconnectTask == NULL ||
		PortNo < 0 || PortNo >= GPIO_COUNT ||
		PinNo < 0 || PinNo >= 32)
	{
		return false;
	}
	gpiotereg = NRF_GPIOTE;
	channelCount = sizeof(gpiotereg->CONFIG) / sizeof(gpiotereg->CONFIG[0]);
	if (channelCount > IOPIN_MAX_INT)
	{
		channelCount = IOPIN_MAX_INT;
	}
	for (i = 0; i < channelCount; i++)
	{
		if ((s_GpIOTaskUsed & (1UL << i)) == 0U)
		{
			continue;
		}
		config = gpiotereg->CONFIG[i];
		if ((config & GPIOTE_CONFIG_MODE_Msk) !=
			(GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos))
		{
			continue;
		}
		pin = (config & GPIOTE_CONFIG_PSEL_Msk) >> GPIOTE_CONFIG_PSEL_Pos;
#ifdef GPIOTE_CONFIG_PORT_Msk
		port = (config & GPIOTE_CONFIG_PORT_Msk) >> GPIOTE_CONFIG_PORT_Pos;
#else
		port = 0U;
#endif
		if (pin != (uint32_t)PinNo || port != (uint32_t)PortNo)
		{
			continue;
		}
		if (!PeiDisconnectTask(pPei, Evt,
			(PeiEndpoint_t)&gpiotereg->TASKS_OUT[i]))
		{
			return false;
		}
		return IOPinCloseTask(i);
	}
#else
	(void)pPei;
	(void)Evt;
	(void)PortNo;
	(void)PinNo;
#endif
	return false;
}

static int IOPinFindAvailInterrupt(void)
{
	int i;

	for (i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (s_GpIOSenseEvt[i].SensEvtCB != NULL ||
			((s_GpIOTaskUsed | s_GpIOEventUsed) & (1UL << i)) != 0U)
		{
			continue;
		}
#if defined(IOPIN_NRF52_GPIOTE)
		// Reserve even a NULL-callback interrupt, and respect channels
		// configured directly by another subsystem.
		if (s_GpIOSenseEvt[i].Sense != IOPINSENSE_DISABLE ||
			NRF_GPIOTE->CONFIG[i] != 0U)
		{
			continue;
		}
#endif
		return i;
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
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	if (reg == NULL || PinNo == -1)
	{
		return;
	}

	// Clear sense
	reg->PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_SENSE_Msk;
	switch (Sense)
	{
		case IOPINSENSE_DISABLE:	// Disable pin sense
			// Already done above
			break;
		case IOPINSENSE_LOW_TRANSITION:	// Event on falling edge
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_HIGH_TRANSITION:// Event on raising edge
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_TOGGLE:			// Event on state change
			// Not supported, use sense low for now
			reg->PIN_CNF[PinNo] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
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
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	if (reg == NULL || PinNo == -1)
	{
		return;
	}

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	// DRIVE0 and DRIVE1 are separate 2 bit fields. A side set to disconnect
	// (open drain) is kept, the other sides go to standard or high drive.
	uint32_t cnf = reg->PIN_CNF[PinNo];
	uint32_t d0 = (cnf & GPIO_PIN_CNF_DRIVE0_Msk) >> GPIO_PIN_CNF_DRIVE0_Pos;
	uint32_t d1 = (cnf & GPIO_PIN_CNF_DRIVE1_Msk) >> GPIO_PIN_CNF_DRIVE1_Pos;
	bool bStrong = (Strength == IOPINSTRENGTH_STRONG);

	if (d0 != GPIO_PIN_CNF_DRIVE0_D0)
	{
		d0 = bStrong ? GPIO_PIN_CNF_DRIVE0_H0 : GPIO_PIN_CNF_DRIVE0_S0;
	}
	if (d1 != GPIO_PIN_CNF_DRIVE1_D1)
	{
		d1 = bStrong ? GPIO_PIN_CNF_DRIVE1_H1 : GPIO_PIN_CNF_DRIVE1_S1;
	}

	cnf &= ~(GPIO_PIN_CNF_DRIVE0_Msk | GPIO_PIN_CNF_DRIVE1_Msk);
	cnf |= (d0 << GPIO_PIN_CNF_DRIVE0_Pos) | (d1 << GPIO_PIN_CNF_DRIVE1_Pos);
	reg->PIN_CNF[PinNo] = cnf;
#else
	uint32_t val = ((reg->PIN_CNF[PinNo] & GPIO_PIN_CNF_DRIVE_Msk) >> GPIO_PIN_CNF_DRIVE_Pos) & 6;
	reg->PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_DRIVE_Msk;
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		// Stronger drive strength
		val++;
	}

	reg->PIN_CNF[PinNo] |= (val << GPIO_PIN_CNF_DRIVE_Pos);
#endif
}

//#if defined(NRF91_SERIES) || defined(NRF53_SERIES) || defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#ifdef NRF5340_XXAA_NETWORK
void __WEAK GPIOTE_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE_NS->EVENTS_IN[i])
		{
			NRF_GPIOTE_NS->EVENTS_IN[i] = 0;
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
		}
	}
	if (NRF_GPIOTE_NS->EVENTS_PORT)
	{
		NRF_GPIOTE_NS->EVENTS_PORT = 0;
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}


}
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
void __WEAK GPIOTE0_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE0_S->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE0_S->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE0_S->EVENTS_PORT)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE0_S->EVENTS_PORT = 0;
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}

	NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
}

void __WEAK GPIOTE1_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE1_NS->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE1_NS->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE1_NS->EVENTS_PORT)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE1_NS->EVENTS_PORT = 0;
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}

	NVIC_ClearPendingIRQ(GPIOTE1_IRQn);
}
#elif defined(NRF54H20_XXAA)
void __WEAK H20_GPIOTE_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	// Other cores may own the remaining GPIOTE130 channels: handle and clear
	// only the channels set up here.
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (s_pGpIOSenseEvt[i].SensEvtCB && NRF_GPIOTE130->EVENTS_IN[i])
		{
			s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE130->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE130->H20_GPIOTE_EVENTS_PORT)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);

		NRF_GPIOTE130->H20_GPIOTE_EVENTS_PORT = 0;
	}

	NVIC_ClearPendingIRQ(H20_GPIOTE_IRQn);
}
#elif defined(NRF54L15_XXAA) || defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)

// Unsecure
void __WEAK GPIOTE20_0_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < GPIOTE20_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE20_NS->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i + 4].SensEvtCB)
				s_pGpIOSenseEvt[i + 4].SensEvtCB(i + 4, s_pGpIOSenseEvt[i + 4].pCtx);
			NRF_GPIOTE20_NS->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE20_NS->EVENTS_PORT[0].NONSECURE)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE20_NS->EVENTS_PORT[0].NONSECURE = 0;
		NRF_P1_NS->LATCH = 0xFFFFFFFF;
	}

	NVIC_ClearPendingIRQ(GPIOTE20_0_IRQn);
}

// Secure
void __WEAK GPIOTE20_1_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}


	for (int i = 0; i < GPIOTE20_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE20_S->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i + 4].SensEvtCB)
				s_pGpIOSenseEvt[i + 4].SensEvtCB(i + 4, s_pGpIOSenseEvt[i + 4].pCtx);
			NRF_GPIOTE20_S->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE20_S->EVENTS_PORT[0].SECURE)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE20_S->EVENTS_PORT[0].SECURE = 0;
		NRF_P1_S->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE20_1_IRQn);
}

void __WEAK GPIOTE30_0_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < GPIOTE30_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE30_NS->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE30_NS->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE30_NS->EVENTS_PORT[0].NONSECURE)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE30_NS->EVENTS_PORT[0].NONSECURE = 0;
		NRF_P0_NS->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE30_0_IRQn);
}

void __WEAK GPIOTE30_1_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < GPIOTE30_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE30_S->EVENTS_IN[i])
		{
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE30_S->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE30_S->EVENTS_PORT[0].SECURE)
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE30_S->EVENTS_PORT[0].SECURE = 0;
		NRF_P0_S->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE30_1_IRQn);
}
#else
void __WEAK GPIOTE_IRQHandler(void)
{
	if (s_pGpIOSenseEvt == NULL)
	{
		// No pin interrupt was set up
		return;
	}

	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		// An nRF52 conventional IRQ does not acknowledge PEI-owned
		// events. Keep the older handler for non-nRF52 targets.
#if defined(IOPIN_NRF52_GPIOTE)
		if (s_pGpIOSenseEvt[i].Sense != IOPINSENSE_DISABLE &&
			NRF_GPIOTE->EVENTS_IN[i])
#else
		if (NRF_GPIOTE->EVENTS_IN[i])
#endif
		{
			if (s_pGpIOSenseEvt[i].SensEvtCB)
				s_pGpIOSenseEvt[i].SensEvtCB(i, s_pGpIOSenseEvt[i].pCtx);
			NRF_GPIOTE->EVENTS_IN[i] = 0;
		}
	}
#if defined(IOPIN_NRF52_GPIOTE)
	if (s_pGpIOSenseEvt[IOPIN_MAX_INT].Sense != IOPINSENSE_DISABLE &&
		NRF_GPIOTE->EVENTS_PORT)
#else
	if (NRF_GPIOTE->EVENTS_PORT)
#endif
	{
		if (s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
			s_pGpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_pGpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE->EVENTS_PORT = 0;
#ifdef NRF52_SERIES
	    NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
#endif
	}

	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}
#endif


bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
#if defined(NRF5340_XXAA_APPLICATION) && !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_GPIO_Type *reg;
	uint32_t select;
	uint32_t mask;
	uint32_t state;
	bool assigned;

	if ((unsigned)CoreNo > 1U || (unsigned)PortNo > 1U ||
		(unsigned)PinNo >= (PortNo == 0 ? P0_PIN_NUM : P1_PIN_NUM))
	{
		return false;
	}

	reg = nRFGpioGetReg(PortNo);
	select = (CoreNo == 0 ? GPIO_PIN_CNF_MCUSEL_AppMCU :
		GPIO_PIN_CNF_MCUSEL_NetworkMCU) << GPIO_PIN_CNF_MCUSEL_Pos;
	mask = 1UL << (unsigned)PinNo;
	state = DisableInterrupt();

	if (CoreNo == 1)
	{
		// Net has no secure execution state. Check access before handing
		// over the pin; a locked secure permission must leave it untouched.
		if ((NRF_SPU_S->GPIOPORT[PortNo].PERM & mask) != 0U)
		{
			if ((NRF_SPU_S->GPIOPORT[PortNo].LOCK & SPU_GPIOPORT_LOCK_LOCK_Msk) != 0U)
			{
				EnableInterrupt(state);
				return false;
			}
			NRF_SPU_S->GPIOPORT[PortNo].PERM &= ~mask;
			__DSB();
			if ((NRF_SPU_S->GPIOPORT[PortNo].PERM & mask) != 0U)
			{
				EnableInterrupt(state);
				return false;
			}
		}
	}

	reg->PIN_CNF[PinNo] = (reg->PIN_CNF[PinNo] & ~GPIO_PIN_CNF_MCUSEL_Msk) | select;
	__DSB();
	assigned = (reg->PIN_CNF[PinNo] & GPIO_PIN_CNF_MCUSEL_Msk) == select;
	EnableInterrupt(state);
	return assigned;
#elif defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || \
	defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA) || \
	defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
	// Net/non-secure App cannot grant ownership. nRF54 auxiliary-core
	// routing is not implemented here; do not report a successful handoff.
	(void)PortNo;
	(void)PinNo;
	(void)CoreNo;
	return false;
#else
	if (CoreNo != 0 || PortNo < 0 || PortNo >= GPIO_COUNT || PinNo < 0)
	{
		return false;
	}
	switch (PortNo)
	{
		case 0:
			return PinNo < P0_PIN_NUM;
#if GPIO_COUNT > 1
		case 1:
			return PinNo < P1_PIN_NUM;
#endif
		default:
			return false;
	}
#endif
}
