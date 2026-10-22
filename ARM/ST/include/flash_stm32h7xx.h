/**
 * STM32H753 flash operations shared by the NVM and DFU target drivers.
 * Copyright (c) 2026 I-SYST inc. SPDX-License-Identifier: MIT
 *
 * Internal implementation, not a second storage interface. Include after
 * stm32.h. Operations are synchronous; callers serialize controller access
 * and reserve the sectors they modify. Do not erase executing code.
 */
#ifndef __FLASH_STM32H7XX_H__
#define __FLASH_STM32H7XX_H__

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include "coredev/interrupt.h"

#if !defined(STM32H753xx)
#error "Flash geometry and programming sequence implemented for STM32H753 only"
#endif

#define ST_H7_FLASH_UNIT 32U
#define ST_H7_FLASH_SECTOR 0x20000UL
#define ST_H7_FLASH_BANK 0x100000UL
#define ST_H7_FLASH_BUSY (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)
#define ST_H7_FLASH_ERR (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | \
	FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | \
	FLASH_SR_DBECCERR)

static inline uint32_t StH7FlashSize(void)
{
	uint32_t kb = *(const volatile uint16_t *)FLASHSIZE_BASE;
	return kb > 0 && kb <= 2048 ? kb << 10 : 0;
}

static inline bool StH7FlashIn(uintptr_t Addr, uint32_t Len)
{
	uint32_t size = StH7FlashSize();
	return Addr >= FLASH_BASE && Addr - FLASH_BASE <= size &&
		Len <= size - (Addr - FLASH_BASE);
}

static inline bool StH7FlashSecond(uintptr_t Addr)
{
	// OPTCR is the active mapping; OPTSR_CUR can hold a pending option.
	return ((Addr - FLASH_BASE) >= ST_H7_FLASH_BANK) !=
		((FLASH->OPTCR & FLASH_OPTCR_SWAP_BANK) != 0);
}

static inline bool StH7FlashWait(volatile uint32_t &Sr, uint32_t Spin)
{
	while (Sr & ST_H7_FLASH_BUSY)
	{
		if (Spin-- == 0) { return false; }
	}
	return true;
}

static inline void StH7FlashSync(uintptr_t Addr, uint32_t Len)
{
	__DSB();
	// All callers supply whole cache lines. Only the changed flash range
	// is discarded, leaving unrelated dirty SRAM lines intact.
	if (SCB->CCR & SCB_CCR_DC_Msk)
	{
		SCB_InvalidateDCache_by_Addr((uint32_t *)Addr, (int32_t)Len);
	}
	if (SCB->CCR & SCB_CCR_IC_Msk) { SCB_InvalidateICache(); }
	__DSB();
	__ISB();
}

static inline int StH7FlashBegin(bool Second, uint32_t Spin)
{
	volatile uint32_t &sr = Second ? FLASH->SR2 : FLASH->SR1;
	volatile uint32_t &cr = Second ? FLASH->CR2 : FLASH->CR1;
	volatile uint32_t &key = Second ? FLASH->KEYR2 : FLASH->KEYR1;
	volatile uint32_t &ccr = Second ? FLASH->CCR2 : FLASH->CCR1;
	if (!StH7FlashWait(sr, Spin)) { return -ETIMEDOUT; }
	// A timed out/foreign operation must not be silently restarted.
	if (cr & (FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_BER)) { return -EBUSY; }
	ccr = ST_H7_FLASH_ERR | FLASH_SR_EOP;
	if (cr & FLASH_CR_LOCK)
	{
		key = 0x45670123UL;
		key = 0xCDEF89ABUL;
	}
	return (cr & FLASH_CR_LOCK) ? -EACCES : 0;
}

static inline int StH7FlashEnd(bool Second, uint32_t Spin,
		uintptr_t Addr, uint32_t Len)
{
	volatile uint32_t &sr = Second ? FLASH->SR2 : FLASH->SR1;
	volatile uint32_t &cr = Second ? FLASH->CR2 : FLASH->CR1;
	volatile uint32_t &ccr = Second ? FLASH->CCR2 : FLASH->CCR1;
	if (!StH7FlashWait(sr, Spin))
	{
		// Do not change CR while hardware is still active. Recovery requires
		// a reset; Begin refuses the retained operation bits meanwhile.
		return -ETIMEDOUT;
	}
	uint32_t status = sr;
	ccr = status & (ST_H7_FLASH_ERR | FLASH_SR_EOP);
	cr &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB);
	cr |= FLASH_CR_LOCK;
	StH7FlashSync(Addr, Len);
	return (status & ST_H7_FLASH_ERR) ? -EIO : 0;
}

static inline int StH7FlashErase(uintptr_t Addr, uint32_t Spin)
{
	if ((Addr % ST_H7_FLASH_SECTOR) || !StH7FlashIn(Addr, ST_H7_FLASH_SECTOR))
	{
		return -EINVAL;
	}
	bool second = StH7FlashSecond(Addr);
	int res = StH7FlashBegin(second, Spin);
	if (res != 0) { return res; }
	volatile uint32_t &cr = second ? FLASH->CR2 : FLASH->CR1;
	uint32_t sector = ((Addr - FLASH_BASE) % ST_H7_FLASH_BANK) / ST_H7_FLASH_SECTOR;
	// x32 parallelism: VDD must be 2.7-3.6 V (NUCLEO-H753ZI is 3.3 V).
	cr = (cr & ~(FLASH_CR_PSIZE | FLASH_CR_SNB)) |
		FLASH_CR_PSIZE_1 | FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
	cr |= FLASH_CR_START;
	__DSB();
	return StH7FlashEnd(second, Spin, Addr, ST_H7_FLASH_SECTOR);
}

static inline int StH7FlashProgram(uintptr_t Addr, const void *pData,
		uint32_t Len, uint32_t Spin)
{
	if (pData == nullptr || Addr % ST_H7_FLASH_UNIT || Len % ST_H7_FLASH_UNIT ||
		!StH7FlashIn(Addr, Len)) { return -EINVAL; }
	const uint8_t *src = (const uint8_t *)pData;
	for (uint32_t off = 0; off < Len; off += ST_H7_FLASH_UNIT)
	{
		// Stage the entire ECC word before PG: source may be unaligned or
		// in the same flash bank. Never read it between the eight stores.
		uint32_t word[8];
		memcpy(word, src + off, sizeof(word));
		uintptr_t addr = Addr + off;
		bool second = StH7FlashSecond(addr);
		int res = StH7FlashBegin(second, Spin);
		if (res != 0) { return res; }
		volatile uint32_t &cr = second ? FLASH->CR2 : FLASH->CR1;
		volatile uint32_t *dst = (volatile uint32_t *)addr;
		bool ones = true;
		bool erased = true;
		StH7FlashSync(addr, ST_H7_FLASH_UNIT);
		for (unsigned i = 0; i < 8; i++)
		{
			ones &= word[i] == 0xFFFFFFFFUL;
			erased &= dst[i] == 0xFFFFFFFFUL;
		}
		if (!erased || ones)
		{
			cr |= FLASH_CR_LOCK;
			if (!erased) { return -EIO; }
			continue; // Preserve erased ECC for an all-ones unit.
		}
		uint32_t state = DisableInterrupt();
		cr |= FLASH_CR_PG;
		__DSB();
		__ISB();
		for (unsigned i = 0; i < 8; i++) { dst[i] = word[i]; }
		__DSB();
		__ISB();
		EnableInterrupt(state);
		res = StH7FlashEnd(second, Spin, addr, ST_H7_FLASH_UNIT);
		if (res != 0) { return res; }
		if (memcmp((const void *)addr, word, sizeof(word))) { return -EIO; }
	}
	return 0;
}
#endif
