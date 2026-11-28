// RP2 Arm core 0, using Raspberry Pi pico-sdk hardware_regs.
#ifndef IOSONATA_RP2_H
#define IOSONATA_RP2_H
#include <stdint.h>
#include "hardware/regs/addressmap.h"
#include "hardware/regs/resets.h"

#ifdef RP2350
#define RP_PRIO_BITS 4U
#else
#define RP_PRIO_BITS 2U
#endif

#define RP_REG(base, off) (*(volatile uint32_t *)((uintptr_t)(base) + (off)))
#define RP_SET(base, off) RP_REG((base) + REG_ALIAS_SET_BITS, off)
#define RP_CLR(base, off) RP_REG((base) + REG_ALIAS_CLR_BITS, off)

static inline void RpReleaseReset(uint32_t mask)
{
	RP_CLR(RESETS_BASE, RESETS_RESET_OFFSET) = mask;
	while ((RP_REG(RESETS_BASE, RESETS_RESET_DONE_OFFSET) & mask) != mask)
	{
	}
}

static inline void RpEnableIrq(unsigned irq, unsigned priority)
{
	unsigned shift = (irq % 4U) * 8U;
	uint32_t field = (uint8_t)(priority << (8U - RP_PRIO_BITS));
	unsigned offset = (irq / 4U) * 4U;
	uint32_t word = RP_REG(0xe000e400UL, offset);

	RP_REG(0xe000e400UL, offset) = (word & ~(255UL << shift)) | (field << shift);
	RP_REG(0xe000e100UL, 4 * (irq / 32)) = 1UL << (irq % 32);
}
#endif
