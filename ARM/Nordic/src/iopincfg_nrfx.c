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

#if defined(NRF51)
#include "nrf_gpiote.h"
#else
#include "nrf_peripherals.h"
#endif

#include "coredev/iopincfg.h"

#if defined(NRF54L15_XXAA)
#define IOPIN_MAX_INT			(GPIOTE20_GPIOTE_NCHANNELS_SIZE + GPIOTE30_GPIOTE_NCHANNELS_SIZE)
#define GPIO_PIN_CNF_DRIVE_Pos (8UL)              //!< Position of DRIVE0 field.
#define GPIO_PIN_CNF_DRIVE_Msk (0xFUL << GPIO_PIN_CNF_DRIVE0_Pos) //!< Bit mask of DRIVE0 field.
#elif defined(NRF54H20_XXAA)
#define IOPIN_MAX_INT			(GPIOTE130_GPIOTE_NCHANNELS_SIZE)
#define GPIO_PIN_CNF_DRIVE_Pos (8UL)              //!< Position of DRIVE0 field.
#define GPIO_PIN_CNF_DRIVE_Msk (0xFUL << GPIO_PIN_CNF_DRIVE0_Pos) //!< Bit mask of DRIVE0 field.
#else
#ifndef GPIOTE_CH_NUM
#define GPIOTE_CH_NUM	8
#endif

#define IOPIN_MAX_INT			(GPIOTE_CH_NUM)
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

NRF_GPIO_Type *nRFGpioGetReg(int PortNo)
{
	NRF_GPIO_Type *reg = NULL;

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

	return reg;
}

NRF_GPIOTE_Type *nRFGpioteGetReg(int PortNo)
{
	NRF_GPIOTE_Type *reg = NULL;

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
		cnf |= (GPIO_PIN_CNF_DRIVE0_S0 << GPIO_PIN_CNF_DRIVE0_Pos) | (GPIO_PIN_CNF_DRIVE1_D1 << GPIO_PIN_CNF_DRIVE1_Pos);
#else
		cnf |= (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
#endif
	}

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

	reg->PIN_CNF[PinNo] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
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

    NRF_GPIOTE_Type *gpiotereg = nRFGpioteGetReg(s_GpIOSenseEvt[IntNo].PortPinNo >> 8);

    if (IntNo < 0)
    {
        IntNo = IOPIN_MAX_INT;
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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
    if (s_GpIOSenseEvt[IntNo].PortPinNo & 0x8000)
    {
    	NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
        NVIC_DisableIRQ(GPIOTE0_IRQn);
    }
    else
    {
    	NVIC_ClearPendingIRQ(GPIOTE1_IRQn);
        NVIC_DisableIRQ(GPIOTE1_IRQn);
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
    if (IntNo >= IOPIN_MAX_INT)
		return false;

    NRF_GPIOTE_Type *gpiotereg = nRFGpioteGetReg(PortNo);
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

#ifdef GPIOTE_CONFIG_PORT_Msk
#define GPIOTE_CONFIG_PORT_PIN_Msk (GPIOTE_CONFIG_PORT_Msk | GPIOTE_CONFIG_PSEL_Msk)
#else
#define GPIOTE_CONFIG_PORT_PIN_Msk GPIOTE_CONFIG_PSEL_Msk
#endif

	uint32_t cfg = 0;
	int idx = IntNo > 3 ? IntNo - 4 : IntNo;

	if (IntNo < 0)
	{
		IntNo = IOPIN_MAX_INT;
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)

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
				reg->PIN_CNF[i] &= ~(GPIO_PIN_CNF_SENSE_Msk << GPIO_PIN_CNF_SENSE_Pos);
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

				gpiotereg->CONFIG[idx] = cfg;

				s_GpIOSenseEvt[IntNo].Sense = Sense;
				s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | i; // For use when disable interrupt
				s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;
				s_GpIOSenseEvt[IntNo].pCtx = pCtx;
			}
		}
	}
	else
	{
		reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_SENSE_Msk << GPIO_PIN_CNF_SENSE_Pos);
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

#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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
	if (PortNo & 0x80)
	{
	    NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
	    NVIC_SetPriority(GPIOTE0_IRQn, IntPrio);
	    NVIC_EnableIRQ(GPIOTE0_IRQn);
	}
	else
	{
	    NVIC_ClearPendingIRQ(GPIOTE1_IRQn);
	    NVIC_SetPriority(GPIOTE1_IRQn, IntPrio);
	    NVIC_EnableIRQ(GPIOTE1_IRQn);
	}
#elif defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
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
	reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_SENSE_Msk << GPIO_PIN_CNF_SENSE_Pos);
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

	uint32_t val = ((reg->PIN_CNF[PinNo] >> GPIO_PIN_CNF_DRIVE_Pos) & GPIO_PIN_CNF_DRIVE_Msk) & 6;
	reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_DRIVE_Msk << GPIO_PIN_CNF_DRIVE_Pos);
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		// Stronger drive strength
		val++;
	}

	reg->PIN_CNF[PinNo] |= (val << GPIO_PIN_CNF_DRIVE_Pos);
}

//#if defined(NRF91_SERIES) || defined(NRF53_SERIES) || defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
#ifdef NRF5340_XXAA_NETWORK
void __WEAK GPIOTE_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE_NS->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE_NS->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE_NS->EVENTS_PORT)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE_NS->EVENTS_PORT = 0;
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}

	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
void __WEAK GPIOTE0_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE0_S->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE0_S->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE0_S->EVENTS_PORT)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE0_S->EVENTS_PORT = 0;
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}

	NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
}

void __WEAK GPIOTE1_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE1_NS->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE1_NS->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE1_NS->EVENTS_PORT)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE1_NS->EVENTS_PORT = 0;
	    //NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
	}

	NVIC_ClearPendingIRQ(GPIOTE0_IRQn);
}
#elif defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)

// Unsecure
void __WEAK GPIOTE20_0_IRQHandler(void)
{
	for (int i = 0; i < GPIOTE20_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE20_NS->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i + 4].SensEvtCB)
				s_GpIOSenseEvt[i + 4].SensEvtCB(i + 4, s_GpIOSenseEvt[i + 4].pCtx);
			NRF_GPIOTE20_NS->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE20_NS->EVENTS_PORT[0].NONSECURE)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE20_NS->EVENTS_PORT[0].NONSECURE = 0;
		NRF_P1_NS->LATCH = 0xFFFFFFFF;
	}

	NVIC_ClearPendingIRQ(GPIOTE20_0_IRQn);
}

// Secure
void __WEAK GPIOTE20_1_IRQHandler(void)
{

	for (int i = 0; i < GPIOTE20_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE20_S->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i + 4].SensEvtCB)
				s_GpIOSenseEvt[i + 4].SensEvtCB(i + 4, s_GpIOSenseEvt[i + 4].pCtx);
			NRF_GPIOTE20_S->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE20_S->EVENTS_PORT[0].SECURE)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE20_S->EVENTS_PORT[0].SECURE = 0;
		NRF_P1_S->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE20_1_IRQn);
}

void __WEAK GPIOTE30_0_IRQHandler(void)
{
	for (int i = 0; i < GPIOTE30_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE30_NS->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE30_NS->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE30_NS->EVENTS_PORT[0].NONSECURE)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE30_NS->EVENTS_PORT[0].NONSECURE = 0;
		NRF_P0_NS->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE30_0_IRQn);
}

void __WEAK GPIOTE30_1_IRQHandler(void)
{
	for (int i = 0; i < GPIOTE30_GPIOTE_NCHANNELS_SIZE; i++)
	{
		if (NRF_GPIOTE30_S->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE30_S->EVENTS_IN[i] = 0;
		}
	}

	if (NRF_GPIOTE30_S->EVENTS_PORT[0].SECURE)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);

        NRF_GPIOTE30_S->EVENTS_PORT[0].SECURE = 0;
		NRF_P0_S->LATCH = 0xFFFFFFFF;
	}
	NVIC_ClearPendingIRQ(GPIOTE30_1_IRQn);
}
#else
void __WEAK GPIOTE_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);
			NRF_GPIOTE->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE->EVENTS_PORT)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1, s_GpIOSenseEvt[IOPIN_MAX_INT].pCtx);
	    NRF_GPIOTE->EVENTS_PORT = 0;
#ifdef NRF52_SERIES
	    NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
#endif
	}

	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}
#endif


