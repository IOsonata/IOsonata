#pragma once
#include <stdint.h>
#define IOSONATA_STM32_HAS_RNG 1
#define RCC_AHB2ENR_RNGEN 1U
#define RCC_AHB2RSTR_RNGRST 1U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define RNG_SR_CEIS 32U
#define RNG_SR_SEIS 64U
struct RngReg {
	unsigned id;
	uint32_t value;
	operator uint32_t() const;
	RngReg &operator=(uint32_t v) { value = v; return *this; }
	RngReg &operator|=(uint32_t v) { return *this = uint32_t(*this) | v; }
	RngReg &operator&=(uint32_t v) { return *this = uint32_t(*this) & v; }
};
struct RngRegisters { RngReg CR{0,0}, SR{1,0}, DR{2,0}; };
struct RccRegisters { uint32_t AHB2ENR, AHB2RSTR; };
extern RngRegisters g_Rng;
extern RccRegisters g_Rcc;
#define RNG (&g_Rng)
#define RCC (&g_Rcc)
