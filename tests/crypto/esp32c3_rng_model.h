#ifndef TEST_ESP32C3_RNG_MODEL_H
#define TEST_ESP32C3_RNG_MODEL_H

#include <stdint.h>
#ifdef __cplusplus
class EspRngModelReg {
public:
	uintptr_t Addr;
	operator uint32_t() const;
	EspRngModelReg &operator=(uint32_t Value);
	EspRngModelReg &operator|=(uint32_t Value)
	{
		return *this = (uint32_t)*this | Value;
	}
	EspRngModelReg &operator&=(uint32_t Value)
	{
		return *this = (uint32_t)*this & Value;
	}
};
EspRngModelReg &EspRngModelRegister(uintptr_t Addr);
#define ESP32_RNG_REG(addr) EspRngModelRegister(addr)
#endif
#endif
