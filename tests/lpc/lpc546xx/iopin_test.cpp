// Host regression of the LPC546xx IOCON setup in IOPinConfig. Plain memory
// stands for the IOCON, GPIO, SYSCON, PINT and INPUTMUX registers.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"

static uint32_t Iocon(int Port, int Pin)
{
	return IOCON->PIO[Port][Pin];
}

static uint32_t Mode(int Port, int Pin)
{
	return (Iocon(Port, Pin) & IOCON_PIO_MODE_MASK) >> IOCON_PIO_MODE_SHIFT;
}

int main()
{
	// Type I pins: no MODE bits whatever the resistor, I2CSLEW 0 (I2C mode),
	// I2CFILTER set by open drain (filter off), I2CDRIVE kept
	IOCON->PIO[3][24] = IOCON_PIO_I2CDRIVE_MASK | IOCON_PIO_I2CSLEW_MASK;
	IOPinConfig(3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);
	IOPinConfig(3, 24, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_OPENDRAIN);
	assert(Iocon(3, 23) == (IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_FILTEROFF_MASK |
							IOCON_PIO_I2CFILTER_MASK));
	assert(Iocon(3, 24) == (IOCON_PIO_FUNC(1) | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_FILTEROFF_MASK |
							IOCON_PIO_I2CFILTER_MASK | IOCON_PIO_I2CDRIVE_MASK));
	IOPinConfig(3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_FOLLOW, IOPINTYPE_NORMAL);
	assert(Mode(3, 23) == 0 && (Iocon(3, 23) & IOCON_PIO_I2CFILTER_MASK) == 0);

	// Type D pins keep the requested resistor and open drain
	IOPinConfig(1, 17, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);
	assert(Iocon(1, 17) == (IOCON_PIO_FUNC(2) | IOCON_PIO_MODE(2) | IOCON_PIO_DIGIMODE_MASK |
							IOCON_PIO_FILTEROFF_MASK | IOCON_PIO_OD_MASK));
	IOPinConfig(3, 22, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	assert(Mode(3, 22) == 1 && (Iocon(3, 22) & IOCON_PIO_OD_MASK) == 0);
	IOPinConfig(3, 25, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_FOLLOW, IOPINTYPE_NORMAL);
	assert(Mode(3, 25) == 3);
	IOPinConfig(2, 23, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	assert(Mode(2, 23) == 2);

	printf("lpc546xx IOCON: type I pins without resistor, type D resistor and open drain: PASS\n");

	return 0;
}
