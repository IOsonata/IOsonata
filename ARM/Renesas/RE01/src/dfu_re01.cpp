/**-------------------------------------------------------------------------
@file	dfu_re01.cpp

@brief	DFU target layer for Renesas RE01 1500KB: code flash through the FACI
		flash sequencer.

The code flash is programmed and erased with FACI commands while it is in
P/E mode, during which it cannot be read: the command code runs from RAM,
the data comes from a RAM buffer and interrupts are held off. The Cortex-M0+
of this part has VTOR, so the application runs with its vectors in slot 0.

Not confirmed from the repository (see the notes at each step): the command
issuing area address, the 128 byte program unit, the block sizes, and the
sequencer clock being the core clock. Every status error, lock or timeout
makes the call fail.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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
#include "re01xxx.h"

#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

// ---------------------------------------------------------------------------
// FACI flash sequencer
//
// The registers are those of RE01_1500KB.h (FLASH_Type). The command issuing
// area is not in the header: 0x407E0000 is where the same sequencer, with
// the same register addresses, takes its commands on the other FACI parts.
// The command codes, key values and error handling follow the FACI program
// and erase flow of those parts.
// ---------------------------------------------------------------------------

#define DFU_TGT_FACI_CMD_AREA		0x407E0000UL
#define DFU_TGT_FACI_CMD8			(*(volatile uint8_t *)DFU_TGT_FACI_CMD_AREA)
#define DFU_TGT_FACI_CMD16			(*(volatile uint16_t *)DFU_TGT_FACI_CMD_AREA)

#define DFU_TGT_CMD_PROGRAM			0xE8U
#define DFU_TGT_CMD_ERASE			0x20U
#define DFU_TGT_CMD_FINAL			0xD0U
#define DFU_TGT_CMD_STATUS_CLEAR	0x50U
#define DFU_TGT_CMD_FORCED_STOP		0xB3U

#define DFU_TGT_FENTRYR_CF_PE		0xAA01U		// Key, code flash P/E mode
#define DFU_TGT_FENTRYR_READ		0xAA00U		// Key, read mode
#define DFU_TGT_FPCKAR_KEY			0x1E00U

#define DFU_TGT_FSTATR_ERR			(FLASH_FSTATR_ILGCOMERR_Msk | \
									 FLASH_FSTATR_FESETERR_Msk | \
									 FLASH_FSTATR_SECERR_Msk | \
									 FLASH_FSTATR_OTERR_Msk | \
									 FLASH_FSTATR_ILGLERR_Msk | \
									 FLASH_FSTATR_ERSERR_Msk | \
									 FLASH_FSTATR_PRGERR_Msk | \
									 FLASH_FSTATR_FLWEERR_Msk)

// Code flash, 1.5 MB from 0.
#define DFU_TGT_CF_END				0x00180000UL

// One program command takes 128 bytes, 64 halfwords, as on the other FACI
// code flash.
#define DFU_TGT_WR_UNIT				128U

// The unit reported to the DFU layer. Every DFU region is aligned on it and
// it is at least the largest block the FACI code flash has (32 KB), so an
// erase never reaches outside the unit it was asked for, whatever the block
// size really is. Inside the unit, a block erase is issued at each 2 KB
// step that does not already read as ones: one command where the block is
// 32 KB, one per block where it is smaller.
#define DFU_TGT_ERASE_UNIT			0x8000U
#define DFU_TGT_ERASE_STEP			0x800U

// Waits are bounded by loop counts, per MHz of the core clock, long enough
// at one loop per 4 clocks: 2 s for a block erase (the FACI data gives about
// 1 s for a 32 KB block), 32 ms for a 128 byte program (15.8 ms), 64 us for
// the data buffer (2 us) and for the mode changes.
#define DFU_TGT_WAIT_ERASE			500000UL
#define DFU_TGT_WAIT_PROGRAM		8000UL
#define DFU_TGT_WAIT_SHORT			16UL

// Everything below that runs with the flash in P/E mode is in RAM: .fastrun
// goes to .data, which ResetEntry copies. Nothing there calls back into
// flash, and the calls into it are long calls.
#define DFU_TGT_RAMFUNC		__attribute__((section(".fastrun"), noinline, long_call))

// Program source, in RAM: the code flash cannot be read while it programs.
alignas(4) static uint16_t s_DfuTgtBuf[DFU_TGT_WR_UNIT / 2];

DFU_TGT_RAMFUNC static bool DfuTgtWaitReady(uint32_t Loops)
{
	while ((FLASH->FSTATR & FLASH_FSTATR_FRDY_Msk) == 0)
	{
		if (Loops-- == 0)
		{
			return false;
		}
	}

	return true;
}

// Stop what the sequencer does and clear its error state, see the FACI
// command locked recovery.
DFU_TGT_RAMFUNC static void DfuTgtRecover(uint32_t Mhz)
{
	DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_FORCED_STOP;
	(void)DfuTgtWaitReady(Mhz * DFU_TGT_WAIT_ERASE);
	if (FLASH->FASTAT & FLASH_FASTAT_CFAE_Msk)
	{
		FLASH->FASTAT = 0;
	}
	DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_STATUS_CLEAR;
	(void)DfuTgtWaitReady(Mhz * DFU_TGT_WAIT_SHORT);
}

DFU_TGT_RAMFUNC static bool DfuTgtPeEnter(uint32_t Mhz)
{
	uint32_t n = Mhz * DFU_TGT_WAIT_SHORT;

	FLASH->FENTRYR = DFU_TGT_FENTRYR_CF_PE;
	while (FLASH->FENTRYR != (DFU_TGT_FENTRYR_CF_PE & 0xFFU))
	{
		if (n-- == 0)
		{
			return false;
		}
	}

	// Sequencer clock, in MHz, rounded up.
	FLASH->FPCKAR = (uint16_t)(DFU_TGT_FPCKAR_KEY | (Mhz & 0xFFU));

	if (FLASH->FASTAT & FLASH_FASTAT_CMDLK_Msk)
	{
		DfuTgtRecover(Mhz);
	}

	return (FLASH->FASTAT & FLASH_FASTAT_CMDLK_Msk) == 0 &&
		   (FLASH->FSTATR & DFU_TGT_FSTATR_ERR) == 0;
}

DFU_TGT_RAMFUNC static bool DfuTgtPeExit(uint32_t Mhz)
{
	uint32_t n = Mhz * DFU_TGT_WAIT_SHORT;

	FLASH->FENTRYR = DFU_TGT_FENTRYR_READ;
	while (FLASH->FENTRYR != 0)
	{
		if (n-- == 0)
		{
			return false;
		}
	}

	return true;
}

// End of a command: ready within the wait, no lock, no error. Anything else
// stops the sequencer and fails.
DFU_TGT_RAMFUNC static bool DfuTgtCmdEnd(uint32_t Mhz, uint32_t Wait)
{
	bool ok = DfuTgtWaitReady(Mhz * Wait) &&
			  (FLASH->FASTAT & FLASH_FASTAT_CMDLK_Msk) == 0 &&
			  (FLASH->FSTATR & DFU_TGT_FSTATR_ERR) == 0;

	if (ok == false)
	{
		DfuTgtRecover(Mhz);
	}

	return ok;
}

// Erase the block holding Addr. P/E mode in and out around the command.
DFU_TGT_RAMFUNC static bool DfuTgtRamErase(uint32_t Addr, uint32_t Mhz)
{
	bool ok = DfuTgtPeEnter(Mhz);

	if (ok)
	{
		FLASH->FSADDR = Addr;
		DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_ERASE;
		DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_FINAL;
		ok = DfuTgtCmdEnd(Mhz, DFU_TGT_WAIT_ERASE);
	}

	return DfuTgtPeExit(Mhz) && ok;
}

// Program one unit at Addr from s_DfuTgtBuf.
DFU_TGT_RAMFUNC static bool DfuTgtRamProgram(uint32_t Addr, uint32_t Mhz)
{
	bool ok = DfuTgtPeEnter(Mhz);

	if (ok)
	{
		FLASH->FSADDR = Addr;
		DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_PROGRAM;
		DFU_TGT_FACI_CMD8 = (uint8_t)(DFU_TGT_WR_UNIT / 2);
		for (uint32_t i = 0; ok && i < DFU_TGT_WR_UNIT / 2; i++)
		{
			DFU_TGT_FACI_CMD16 = s_DfuTgtBuf[i];

			uint32_t n = Mhz * DFU_TGT_WAIT_SHORT;
			while (FLASH->FSTATR & FLASH_FSTATR_DBFULL_Msk)
			{
				if (n-- == 0)
				{
					ok = false;
					break;
				}
			}
		}
		if (ok)
		{
			DFU_TGT_FACI_CMD8 = DFU_TGT_CMD_FINAL;
			ok = DfuTgtCmdEnd(Mhz, DFU_TGT_WAIT_PROGRAM);
		}
		else
		{
			DfuTgtRecover(Mhz);
		}
	}

	return DfuTgtPeExit(Mhz) && ok;
}

// Core clock range of the part; the FACI sequencer needs 4 MHz or more.
#define DFU_TGT_MHZ_MIN				4U
#define DFU_TGT_MHZ_MAX				64U

// The vector table and every handler are in the code flash, so nothing may
// interrupt while it is in P/E mode.
static bool DfuTgtRun(bool (*pFunc)(uint32_t, uint32_t), uint32_t Addr)
{
	// Core clock in MHz, rounded up: what FPCKAR is told (the sequencer is
	// taken to run from the core clock here) and what the waits scale with.
	// Out of range, nothing is programmed with a wrong setting.
	uint32_t mhz = (SystemCoreClock + 999999UL) / 1000000UL;

	if (mhz < DFU_TGT_MHZ_MIN || mhz > DFU_TGT_MHZ_MAX)
	{
		return false;
	}

	uint32_t pm = __get_PRIMASK();

	__disable_irq();

	// Code flash P/E permitted (FWEPROR.FLWE = 01).
	FLASH->FWEPROR = 1;

	bool ok = pFunc(Addr, mhz);

	__DSB();
	__ISB();
	__set_PRIMASK(pm);

	return ok;
}

// ---------------------------------------------------------------------------
// Internal memory
// ---------------------------------------------------------------------------

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	return Addr < DFU_TGT_CF_END ? DFU_TGT_ERASE_UNIT : 0;
}

static bool DfuTgtErased(uintptr_t Addr, uint32_t Len)
{
	const volatile uint32_t *p = (const volatile uint32_t *)Addr;

	for (uint32_t i = 0; i < Len / 4; i++)
	{
		if (p[i] != 0xFFFFFFFFUL)
		{
			return false;
		}
	}

	return true;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if ((Addr % DFU_TGT_ERASE_UNIT) != 0 || Addr >= DFU_TGT_CF_END)
	{
		return false;
	}

	for (uint32_t off = 0; off < DFU_TGT_ERASE_UNIT; off += DFU_TGT_ERASE_STEP)
	{
		if (DfuTgtErased(Addr + off, DFU_TGT_ERASE_STEP) == false &&
			DfuTgtRun(DfuTgtRamErase, Addr + off) == false)
		{
			return false;
		}
	}

	return DfuTgtErased(Addr, DFU_TGT_ERASE_UNIT);
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr % DFU_TGT_WR_UNIT) != 0 || (Len % DFU_TGT_WR_UNIT) != 0 ||
		Addr >= DFU_TGT_CF_END || Len > DFU_TGT_CF_END - Addr)
	{
		return false;
	}

	const uint8_t *p = (const uint8_t *)pData;

	for (uint32_t o = 0; o < Len; o += DFU_TGT_WR_UNIT)
	{
		// Through RAM: the source may be anywhere, the code flash included,
		// and unaligned.
		for (uint32_t i = 0; i < DFU_TGT_WR_UNIT / 2; i++, p += 2)
		{
			s_DfuTgtBuf[i] = (uint16_t)(p[0] | (p[1] << 8));
		}
		if (DfuTgtRun(DfuTgtRamProgram, Addr + o) == false)
		{
			return false;
		}
	}

	return true;
}

// ---------------------------------------------------------------------------
// Start and reset
// ---------------------------------------------------------------------------

// On chip SRAM, 256 KB.
#define DFU_TGT_RAM_START		0x20000000UL
#define DFU_TGT_RAM_END			0x20040000UL

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_TGT_RAM_START,
						   DFU_TGT_RAM_END);
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

// This Cortex-M0+ has VTOR (__VTOR_PRESENT), so the vectors of slot 0 are
// used where they are.
void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();
	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
