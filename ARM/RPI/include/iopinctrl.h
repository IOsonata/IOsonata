// Pico and Pico 2 expose GPIO 0..29 on SIO bank 0.
#ifndef RP2_IOPINCTRL_H
#define RP2_IOPINCTRL_H
#include "rp2.h"
#include "hardware/regs/sio.h"
#include "coredev/iopincfg.h"

static inline bool RpPinValid(int port, int pin) { return port == 0 && (unsigned)pin < 30U; }
static inline void IOPinSetDir(int port, int pin, IOPINDIR dir)
{
	if (RpPinValid(port, pin))
	{
		RP_REG(SIO_BASE, dir == IOPINDIR_OUTPUT ? SIO_GPIO_OE_SET_OFFSET : SIO_GPIO_OE_CLR_OFFSET) = 1UL << pin;
	}
}
static inline int IOPinRead(int port, int pin)
{
	return RpPinValid(port, pin) && (RP_REG(SIO_BASE, SIO_GPIO_IN_OFFSET) & (1UL << pin));
}
static inline void IOPinSet(int port, int pin)
{
	if (RpPinValid(port, pin)) RP_REG(SIO_BASE, SIO_GPIO_OUT_SET_OFFSET) = 1UL << pin;
}
static inline void IOPinClear(int port, int pin)
{
	if (RpPinValid(port, pin)) RP_REG(SIO_BASE, SIO_GPIO_OUT_CLR_OFFSET) = 1UL << pin;
}
static inline void IOPinToggle(int port, int pin)
{
	if (RpPinValid(port, pin)) RP_REG(SIO_BASE, SIO_GPIO_OUT_XOR_OFFSET) = 1UL << pin;
}
static inline uint32_t IOPinReadPort(int port)
{
	return port == 0 ? RP_REG(SIO_BASE, SIO_GPIO_IN_OFFSET) : 0U;
}
static inline void IOPinWritePort(int port, uint32_t data)
{
	if (port == 0) RP_REG(SIO_BASE, SIO_GPIO_OUT_OFFSET) = data;
}
#endif
