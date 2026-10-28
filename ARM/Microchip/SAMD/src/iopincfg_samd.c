/**-------------------------------------------------------------------------
@file	iopincfg_samd.c

@brief	SAM D11/D21 PORT pin configuration.

@license
MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "coredev/iopincfg.h"

#if defined(__SAMD11D14A__)
#define SAMD_PORT_COUNT	1
#elif defined(__SAMD21J18A__)
#define SAMD_PORT_COUNT	2
#else
#error Unsupported SAM D device
#endif

#define SAMD_PORT_BASE		0x41004400UL
#define SAMD_PORT_GROUP_SIZE	0x80UL

#define SAMD_PORT_DIRCLR	0x04UL
#define SAMD_PORT_DIRSET	0x08UL
#define SAMD_PORT_OUTCLR	0x14UL
#define SAMD_PORT_OUTSET	0x18UL
#define SAMD_PORT_PMUX		0x30UL
#define SAMD_PORT_PINCFG	0x40UL

#define SAMD_PINCFG_PMUXEN	(1U << 0)
#define SAMD_PINCFG_INEN	(1U << 1)
#define SAMD_PINCFG_PULLEN	(1U << 2)
#define SAMD_PINCFG_DRVSTR	(1U << 6)

static volatile uint32_t *SamdPortReg(int PortNo, uint32_t Offset)
{
	return (volatile uint32_t *)(uintptr_t)(SAMD_PORT_BASE +
		(uint32_t)PortNo * SAMD_PORT_GROUP_SIZE + Offset);
}

static volatile uint8_t *SamdPortByteReg(int PortNo, uint32_t Offset)
{
	return (volatile uint8_t *)(uintptr_t)(SAMD_PORT_BASE +
		(uint32_t)PortNo * SAMD_PORT_GROUP_SIZE + Offset);
}

static bool SamdPinValid(int PortNo, int PinNo)
{
	return PortNo >= 0 && PortNo < SAMD_PORT_COUNT &&
		PinNo >= 0 && PinNo < 32;
}

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	return CoreNo == 0 && SamdPinValid(PortNo, PinNo);
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir,
	IOPINRES Resistor, IOPINTYPE Type)
{
	uint32_t mask;
	uint8_t cfg;
	uint8_t mux;
	uint32_t index;
	volatile uint8_t *pMux;
	volatile uint8_t *pCfg;

	if (!SamdPinValid(PortNo, PinNo) || PinOp < IOPINOP_GPIO ||
		PinOp > IOPINOP_PERIPHH || Type != IOPINTYPE_NORMAL)
	{
		return;
	}

	mask = 1UL << PinNo;
	pCfg = SamdPortByteReg(PortNo, SAMD_PORT_PINCFG + (uint32_t)PinNo);
	cfg = *pCfg & SAMD_PINCFG_DRVSTR;

	if (Resistor == IOPINRES_PULLUP || Resistor == IOPINRES_FOLLOW)
	{
		*SamdPortReg(PortNo, SAMD_PORT_OUTSET) = mask;
		cfg |= SAMD_PINCFG_PULLEN;
	}
	else if (Resistor == IOPINRES_PULLDOWN)
	{
		*SamdPortReg(PortNo, SAMD_PORT_OUTCLR) = mask;
		cfg |= SAMD_PINCFG_PULLEN;
	}

	if (PinOp != IOPINOP_GPIO)
	{
		index = (uint32_t)PinNo / 2U;
		pMux = SamdPortByteReg(PortNo, SAMD_PORT_PMUX + index);
		mux = *pMux;
		if ((PinNo & 1) != 0)
		{
			mux = (mux & 0x0FU) | (uint8_t)((PinOp - 1) << 4);
		}
		else
		{
			mux = (mux & 0xF0U) | (uint8_t)(PinOp - 1);
		}
		*pMux = mux;
		cfg |= SAMD_PINCFG_PMUXEN | SAMD_PINCFG_INEN;
	}
	else if (Dir == IOPINDIR_OUTPUT)
	{
		*SamdPortReg(PortNo, SAMD_PORT_DIRSET) = mask;
	}
	else
	{
		*SamdPortReg(PortNo, SAMD_PORT_DIRCLR) = mask;
		cfg |= SAMD_PINCFG_INEN;
	}

	*pCfg = cfg;
}

void IOPinDisable(int PortNo, int PinNo)
{
	uint32_t mask;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	mask = 1UL << PinNo;
	*SamdPortReg(PortNo, SAMD_PORT_DIRCLR) = mask;
	*SamdPortByteReg(PortNo, SAMD_PORT_PINCFG + (uint32_t)PinNo) = 0;
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	uint8_t cfg;
	volatile uint8_t *pCfg;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	pCfg = SamdPortByteReg(PortNo, SAMD_PORT_PINCFG + (uint32_t)PinNo);
	cfg = *pCfg;
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		cfg |= SAMD_PINCFG_DRVSTR;
	}
	else
	{
		cfg &= (uint8_t)~SAMD_PINCFG_DRVSTR;
	}
	*pCfg = cfg;
}
