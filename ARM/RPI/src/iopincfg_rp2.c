// IO bank 0 function and pad selection.
#include "iopinctrl.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/pads_bank0.h"

void IOPinConfig(int port, int pin, int op, IOPINDIR dir, IOPINRES res, IOPINTYPE type)
{
	uint32_t pad, offset;
	if (!RpPinValid(port, pin) || op < 0 || op > IOPINOP_FUNC31)
	{
		return;
	}
	offset = PADS_BANK0_GPIO0_OFFSET + 4U * pin;
	pad = RP_REG(PADS_BANK0_BASE, offset);
	pad &= ~(PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS | PADS_BANK0_GPIO0_OD_BITS);
	pad |= PADS_BANK0_GPIO0_IE_BITS;
	if (res == IOPINRES_PULLUP) pad |= PADS_BANK0_GPIO0_PUE_BITS;
	if (res == IOPINRES_PULLDOWN) pad |= PADS_BANK0_GPIO0_PDE_BITS;
	(void)type;
	IOPinSetDir(port, pin, dir);
	RP_REG(IO_BANK0_BASE, IO_BANK0_GPIO0_CTRL_OFFSET + 8U * pin) = op == IOPINOP_GPIO ? 5U : (uint32_t)op - 1U;
#ifdef PADS_BANK0_GPIO0_ISO_BITS
	pad &= ~PADS_BANK0_GPIO0_ISO_BITS;
#endif
	RP_REG(PADS_BANK0_BASE, offset) = pad;
}

void IOPinDisable(int port, int pin)
{
	if (RpPinValid(port, pin))
	{
		IOPinSetDir(port, pin, IOPINDIR_INPUT);
		RP_REG(IO_BANK0_BASE, IO_BANK0_GPIO0_CTRL_OFFSET + 8U * pin) = 31U;
		RP_REG(PADS_BANK0_BASE, PADS_BANK0_GPIO0_OFFSET + 4U * pin) = PADS_BANK0_GPIO0_OD_BITS;
	}
}

#include "coredev/interrupt.h"
#include "hardware/regs/intctrl.h"

typedef struct {
	IOPinEvtHandler_t Handler;
	void *Ctx;
	unsigned Pin;
} RpPinIrq_t;

static RpPinIrq_t s_PinIrq[4];

static uint32_t RpEdge(IOPINSENSE sense)
{
	switch (sense)
	{
		case IOPINSENSE_LOW_TRANSITION: return 4U;
		case IOPINSENSE_HIGH_TRANSITION: return 8U;
		case IOPINSENSE_TOGGLE: return 12U;
		default: return 0;
	}
}

bool IOPinEnableInterrupt(int index, int prio, uint32_t port, uint32_t pin, IOPINSENSE sense,
	IOPinEvtHandler_t cb, void *ctx)
{
	unsigned shift = (pin % 8U) * 4U;
	unsigned offset = (pin / 8U) * 4U;
	uint32_t edge = RpEdge(sense);
	uint32_t state;
	if ((unsigned)index >= 4U || !RpPinValid(port, pin) || !cb || !edge) return false;
	state = DisableInterrupt();
	for (unsigned i = 0; i < 4; i++)
	{
		if (s_PinIrq[i].Handler && (i == (unsigned)index || s_PinIrq[i].Pin == pin))
		{
			EnableInterrupt(state);
			return false;
		}
	}
	s_PinIrq[index] = (RpPinIrq_t){cb, ctx, pin};
	RP_REG(IO_BANK0_BASE, IO_BANK0_INTR0_OFFSET + offset) = edge << shift;
	RP_SET(IO_BANK0_BASE, IO_BANK0_PROC0_INTE0_OFFSET + offset) = edge << shift;
	RpEnableIrq(IO_IRQ_BANK0, prio);
	EnableInterrupt(state);
	return true;
}
int IOPinAllocateInterrupt(int prio, int port, int pin, IOPINSENSE sense, IOPinEvtHandler_t cb, void *ctx)
{
	for (int i = 0; i < 4; i++)
	{
		if (IOPinEnableInterrupt(i, prio, port, pin, sense, cb, ctx)) return i;
	}
	return -1;
}
void IOPinDisableInterrupt(int index)
{
	uint32_t state = DisableInterrupt();
	if ((unsigned)index < 4 && s_PinIrq[index].Handler)
	{
		unsigned pin = s_PinIrq[index].Pin;
		RP_CLR(IO_BANK0_BASE, IO_BANK0_PROC0_INTE0_OFFSET + 4U * (pin / 8)) = 15U << (4U * (pin % 8));
		s_PinIrq[index].Handler = NULL;
	}
	EnableInterrupt(state);
}
void RpPinIrq(void)
{
	for (int i = 0; i < 4; i++)
	{
		unsigned pin = s_PinIrq[i].Pin;
		unsigned offset = 4U * (pin / 8U);
		uint32_t mask = (12U << (4U * (pin % 8U))) & RP_REG(IO_BANK0_BASE, IO_BANK0_PROC0_INTS0_OFFSET + offset);
		if (s_PinIrq[i].Handler && mask)
		{
			RP_REG(IO_BANK0_BASE, IO_BANK0_INTR0_OFFSET + offset) = mask;
			s_PinIrq[i].Handler(i, s_PinIrq[i].Ctx);
		}
	}
}
