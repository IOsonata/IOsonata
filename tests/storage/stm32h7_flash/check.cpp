// Host register-model checks using the real H753 device register definitions.
#include <cassert>
#include <cstdio>
#include <sys/mman.h>
#include "stm32h753xx.h"
#include "flash_stm32h7xx.h"

ScbMock scb_mock = {};
static unsigned erases, invalidations;
static uint32_t fault;
static bool timeout;
void SCB_InvalidateICache() {}
void SCB_InvalidateDCache_by_Addr(uint32_t *p, int32_t n)
{
	assert(((uintptr_t)p % 32) == 0 && n % 32 == 0);
	invalidations++;
}
void __DSB()
{
	for (unsigned bank = 0; bank < 2; bank++)
	{
		volatile uint32_t &cr = bank ? FLASH->CR2 : FLASH->CR1;
		volatile uint32_t &sr = bank ? FLASH->SR2 : FLASH->SR1;
		if (cr & FLASH_CR_START)
		{
			assert(!(cr & FLASH_CR_LOCK) && (cr & FLASH_CR_SER));
			assert((cr & FLASH_CR_PSIZE) == FLASH_CR_PSIZE_1);
			unsigned mapped = bank ^ ((FLASH->OPTCR & FLASH_OPTCR_SWAP_BANK) != 0);
			uintptr_t addr = FLASH_BASE + mapped * ST_H7_FLASH_BANK +
				((cr & FLASH_CR_SNB) >> FLASH_CR_SNB_Pos) * ST_H7_FLASH_SECTOR;
			memset((void *)addr, 0xff, ST_H7_FLASH_SECTOR);
			cr &= ~FLASH_CR_START;
			erases++;
		}
		if (cr & (FLASH_CR_SER | FLASH_CR_PG))
		{
			sr = fault | (timeout ? FLASH_SR_QW : FLASH_SR_EOP);
		}
	}
}
static void reset()
{
	memset((void *)FLASH, 0, sizeof(*FLASH));
	fault = 0;
	timeout = false;
}
static void map(uintptr_t addr, size_t size)
{
	assert(mmap((void *)addr, size, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)addr);
}
int main()
{
	map(FLASH_BASE, 0x200000);
	map(FLASH_R_BASE & ~4095UL, 4096);
	map(FLASHSIZE_BASE & ~4095UL, 4096);
	*(uint16_t *)FLASHSIZE_BASE = 2048;
	assert(StH7FlashSize() == 0x200000);
	assert(!StH7FlashIn(FLASH_BASE - 1, 1));
	assert(!StH7FlashIn(UINTPTR_MAX - 31, 32));
	assert(!StH7FlashIn(FLASH_BASE + 0x200000, 32));
	uint8_t source[65];
	memset(source, 0x5a, sizeof(source));
	scb_mock.CCR = SCB_CCR_DC_Msk | SCB_CCR_IC_Msk;
	for (unsigned swap = 0; swap < 2; swap++)
	{
		for (unsigned sector = 0; sector < 16; sector++)
		{
			reset();
			FLASH->OPTCR = swap ? FLASH_OPTCR_SWAP_BANK : 0;
			uintptr_t addr = FLASH_BASE + sector * ST_H7_FLASH_SECTOR;
			bool second = (sector >= 8) != (swap != 0);
			assert(StH7FlashSecond(addr) == second);
			memset((void *)addr, 0, ST_H7_FLASH_SECTOR);
			assert(StH7FlashErase(addr, 100) == 0);
			assert(*(uint32_t *)addr == 0xffffffff);
			assert((second ? FLASH->CR2 : FLASH->CR1) & FLASH_CR_LOCK);
			// Model an unlocked bank; key unlock itself is silicon behavior.
			(second ? FLASH->CR2 : FLASH->CR1) = 0;
			assert(StH7FlashProgram(addr, source + 1, 32, 100) == 0);
			assert(memcmp((void *)addr, source + 1, 32) == 0);
			assert((second ? FLASH->CR2 : FLASH->CR1) & FLASH_CR_LOCK);
			(second ? FLASH->CR2 : FLASH->CR1) = 0;
			assert(StH7FlashProgram(addr, source, 32, 100) == -EIO);
		}
	}
	reset();
	assert(StH7FlashProgram(FLASH_BASE + 1, source, 32, 2) == -EINVAL);
	assert(StH7FlashProgram(FLASH_BASE, source, 31, 2) == -EINVAL);
	assert(StH7FlashErase(FLASH_BASE + 32, 2) == -EINVAL);
	FLASH->SR1 = FLASH_SR_QW;
	assert(StH7FlashErase(FLASH_BASE, 2) == -ETIMEDOUT);
	reset();
	FLASH->CR1 = FLASH_CR_LOCK;
	assert(StH7FlashErase(FLASH_BASE, 2) == -EACCES);
	assert(FLASH->KEYR1 == 0xCDEF89ABUL);
	reset();
	fault = FLASH_SR_WRPERR;
	assert(StH7FlashErase(FLASH_BASE, 2) == -EIO);
	assert(FLASH->CCR1 == (FLASH_SR_WRPERR | FLASH_SR_EOP));
	assert(FLASH->CR1 & FLASH_CR_LOCK);
	reset();
	timeout = true;
	assert(StH7FlashErase(FLASH_BASE, 2) == -ETIMEDOUT);
	assert(FLASH->CR1 & FLASH_CR_SER);
	timeout = false;
	FLASH->SR1 = 0;
	assert(StH7FlashErase(FLASH_BASE, 2) == -EBUSY);
	reset();
	memset((void *)FLASH_BASE, 0xff, 32);
	memset(source, 0xff, 32);
	assert(StH7FlashProgram(FLASH_BASE, source, 32, 2) == 0);
	assert(FLASH->SR1 == 0); // no program was issued for all ones
	assert(erases == 34 && invalidations > 32);
	puts("H753 flash register model: PASS (both mappings, all sectors, writes, errors, timeouts)");
}
