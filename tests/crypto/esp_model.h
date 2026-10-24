#pragma once
#include <stdint.h>
#ifdef __cplusplus
struct EspReg {
	uintptr_t address;
	operator uint32_t() const;
	EspReg &operator=(uint32_t value);
	EspReg &operator|=(uint32_t value) { return *this = uint32_t(*this) | value; }
	EspReg &operator&=(uint32_t value) { return *this = uint32_t(*this) & value; }
};
EspReg &EspRegister(uintptr_t address);
#define ESP32_CRYPTO_REG(address) EspRegister(address)
#endif
