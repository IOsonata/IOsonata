/**-------------------------------------------------------------------------
@file	iopincfg_re01.c

@brief	I/O pin configuration implementation on Renesas RE01 series

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


#define RE01_1500KB_PIN_MAX_INT		(9)
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
	IOPINSENSE Sense;
	IOPinEvtHandler_t SensEvtCB;
    uint16_t PortPinNo;
    void *pCtx;
} IOPINSENS_EVTHOOK;
#pragma pack(pop)

static IOPINSENS_EVTHOOK s_GpIOSenseEvt[RE01_1500KB_PIN_MAX_INT + 1] = { {0, NULL}, };

/**
 * @brief Configure individual I/O pin.
 *
 * @Param 	PortNo	: Port number
 * 						STM32 ports are named A, B, C,...
 * 							0 = A, 1 = B, ...
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						for STM32
 * 						IOPINOP_FUNC_0 - IOPINOP_FUNC_15 -> AF0 - AF15
 * 						IOPINOP_FUNC_16 -> Analog
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	PORT0_Type *reg = (PORT0_Type *)(PORT0_BASE + PortNo * 0x20);

	if (PortNo == -1 || PinNo == -1 || PortNo > RE01_1500KB_MAX_PORT)
		return;

	uint32_t tmp;

	uint32_t pos = 1 << PinNo;
	tmp = reg->PDR & ~pos;

	if (PinOp == IOPINOP_GPIO)
	{
		if (Dir == IOPINDIR_OUTPUT)
		{
			tmp |= pos;
		}
	}
	else if (PinOp < IOPINOP_FUNC16)
	{
		// Alternate function
	}
	else
	{
		// Analog
		tmp |= 3 << pos;
	}

	reg->PDR = tmp;

	//pos = PinNo << 1;

	PMISC->PWPR = PMISC_PWPR_PFSWE_Msk;	// Write enable

	int idx = PinNo + (PortNo * 16);
	__IOM uint32_t *psf = (__IOM uint32_t*)PFS_BASE;

	uint32_t pull = psf[idx] & (0xFFFFFF0F);

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:
		case IOPINRES_PULLUP:
			pull |= 0x10;	// PUCR
			if (Type == IOPINTYPE_OPENDRAIN)
			{
				pull |= 0x40;	// NCODR
			}
			break;
		case IOPINRES_PULLDOWN:
			pull |=  0x20;	// PDCR
			if (Type == IOPINTYPE_OPENDRAIN)
			{
				pull |= 0x80;	// PCODR
			}
			break;
		case IOPINRES_NONE:
			break;
	}

	psf[idx] = pull;

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
#if 0
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);
	uint32_t pos = PinNo << 1;

	reg->MODER |=  (GPIO_MODER_MODE0 << pos);
	reg->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << pos);
	reg->PUPDR &= ~(GPIO_PUPDR_PUPD0 << pos);

	pos = (PinNo & 0x7) << 2;
	int idx = PinNo >> 3;

	reg->AFR[idx] &= ~(0xf << pos);
	reg->OTYPER &= ~(GPIO_OTYPER_OT0 << PinNo);

#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)
      /* Deactivate the Control bit of Analog mode for the current IO */
	reg->ASCR &= ~(GPIO_ASCR_ASC0<< PinNo);
#endif /* STM32L471xx || STM32L475xx || STM32L476xx || STM32L485xx || STM32L486xx */

 	if (PortNo == 6 && PinNo > 0 && (reg->MODER & 0x55555554) == 0 )
	{
		// Port G requires VDDIO2
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
		PWR->CR2 &= ~PWR_CR2_IOSV;
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
	}
#endif
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
/*
	int idx = (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF) >> 2;
	uint32_t pos = (s_GpIOSenseEvt[IntNo].PortPinNo & 0xF) << 2;
	uint32_t mask = 0xF << pos;

	SYSCFG->EXTICR[idx] &= ~mask;

	mask = ~(1 << (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF));

	EXTI->RTSR1 &= mask;
	EXTI->FTSR1 &= mask;
	EXTI->IMR1 &= mask;

    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;
    s_GpIOSenseEvt[IntNo].pCtx = NULL;

    switch (IntNo)
    {
    	case 0:
    		NVIC_DisableIRQ(EXTI0_IRQn);
    		NVIC_ClearPendingIRQ(EXTI0_IRQn);
    		break;
    	case 1:
    		NVIC_DisableIRQ(EXTI1_IRQn);
    		NVIC_ClearPendingIRQ(EXTI1_IRQn);
    		break;
    	case 2:
    		NVIC_DisableIRQ(EXTI2_IRQn);
    		NVIC_ClearPendingIRQ(EXTI2_IRQn);
    		break;
    	case 3:
    		NVIC_DisableIRQ(EXTI3_IRQn);
    		NVIC_ClearPendingIRQ(EXTI3_IRQn);
    		break;
    	case 4:
    		NVIC_DisableIRQ(EXTI4_IRQn);
    		NVIC_ClearPendingIRQ(EXTI4_IRQn);
    		break;
    	default:
    		if (IntNo > 4 && IntNo < 10)
    		{
				// Shared interrupt, make sure no one still using it
    			for (int i = 5; i < 10; i++)
    			{
    				if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
    				{
    					return;
    				}
    			}
        		NVIC_DisableIRQ(EXTI9_5_IRQn);
        		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    		}
    		else
    		{
				// Shared interrupt, make sure no one still using it
    			for (int i = 10; i < IOPIN_MAX_INT; i++)
    			{
    				if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
    				{
    					return;
    				}
    			}
        		NVIC_DisableIRQ(EXTI15_10_IRQn);
        		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    		}
    }*/
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
bool IOPinEnableInterrupt(int IntNo, int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPinEvtHandler_t pEvtCB, void *pCtx)
{
	if (IntNo < 0 || IntNo >= RE01_1500KB_PIN_MAX_INT || IntNo != PinNo)
	{
		return false;
	}
/*
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	int idx = IntNo >> 2;
	uint32_t pos = (IntNo & 0x3) << 2;
	uint32_t mask = 0xF << pos;

	SYSCFG->EXTICR[idx] &= ~mask;
	SYSCFG->EXTICR[idx] |= PortNo << pos;

	mask = (1 << (PinNo & 0xFF));

	EXTI->IMR1 |= mask;

	mask &= 0x7DFFFF;

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			EXTI->RTSR1 &= ~mask;
			EXTI->FTSR1 |= mask;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			EXTI->RTSR1 |= mask;
			EXTI->FTSR1 &= ~mask;
			break;
		case IOPINSENSE_TOGGLE:
			EXTI->RTSR1 |= mask;
			EXTI->FTSR1 |= mask;
			break;
	}

    s_GpIOSenseEvt[IntNo].Sense = Sense;
	s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;
	s_GpIOSenseEvt[IntNo].pCtx = pCtx;


	switch (IntNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(EXTI0_IRQn);
			NVIC_SetPriority(EXTI0_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(EXTI1_IRQn);
			NVIC_SetPriority(EXTI1_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(EXTI2_IRQn);
			NVIC_SetPriority(EXTI2_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI2_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(EXTI3_IRQn);
			NVIC_SetPriority(EXTI3_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(EXTI4_IRQn);
			NVIC_SetPriority(EXTI4_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI4_IRQn);
			break;
		default:
			if (IntNo > 4 && IntNo < 10)
			{
				NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
				NVIC_SetPriority(EXTI9_5_IRQn, IntPrio);
				NVIC_EnableIRQ(EXTI9_5_IRQn);
			}
			else
			{
				NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
				NVIC_SetPriority(EXTI15_10_IRQn, IntPrio);
				NVIC_EnableIRQ(EXTI15_10_IRQn);
			}
    }
*/
    return true;
}

int IOPinFindAvailInterrupt()
{
	for (int i = 0; i < RE01_1500KB_PIN_MAX_INT; i++)
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
	// Pin sense is not avail on this Renesas
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
	// Not available on this Renesas
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
#if 0
void EXTI0_IRQHandler(void)
{
	if (EXTI->PR1 & 1)
	{
		EXTI->PR1 = 1;

		if (s_GpIOSenseEvt[0].SensEvtCB)
			s_GpIOSenseEvt[0].SensEvtCB(0, s_GpIOSenseEvt[0].pCtx);

	}

	NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

void EXTI1_IRQHandler(void)
{
	if (EXTI->PR1 & 2)
	{
		EXTI->PR1 = 2;

		if (s_GpIOSenseEvt[1].SensEvtCB)
			s_GpIOSenseEvt[1].SensEvtCB(1, s_GpIOSenseEvt[1].pCtx);

	}

	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void)
{
	if (EXTI->PR1 & 4)
	{
		EXTI->PR1 = 4;

		if (s_GpIOSenseEvt[2].SensEvtCB)
			s_GpIOSenseEvt[2].SensEvtCB(2, s_GpIOSenseEvt[2].pCtx);

	}

	NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void)
{
	if (EXTI->PR1 & 8)
	{
		EXTI->PR1 = 8;

		if (s_GpIOSenseEvt[3].SensEvtCB)
			s_GpIOSenseEvt[3].SensEvtCB(3, s_GpIOSenseEvt[3].pCtx);

	}

	NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void EXTI4_IRQHandler(void)
{
	if (EXTI->PR1 & 0x10)
	{
		EXTI->PR1 = 0x10;

		if (s_GpIOSenseEvt[4].SensEvtCB)
			s_GpIOSenseEvt[4].SensEvtCB(4, s_GpIOSenseEvt[4].pCtx);

	}

	NVIC_ClearPendingIRQ(EXTI4_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
	uint32_t mask = 1 << 5;

	for (int i = 5; i < 10; i++)
	{
		if (EXTI->PR1 & mask)
		{
			EXTI->PR1 = mask;
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);

		}
		mask <<= 1;
	}

	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	uint32_t mask = 1 << 10;

	for (int i = 10; i < IOPIN_MAX_INT; i++)
	{
		if (EXTI->PR1 & mask)
		{
			EXTI->PR1 = mask;
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i, s_GpIOSenseEvt[i].pCtx);

		}
		mask <<= 1;
	}

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}
#endif
