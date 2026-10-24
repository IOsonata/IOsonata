#pragma once
#include <stdint.h>
struct EcbReg {
	unsigned id;
	uintptr_t value;
	EcbReg &operator=(uintptr_t v);
	operator uintptr_t() const { return value; }
};
struct NRF_ECB_Type {
	EcbReg EVENTS_ENDECB{0,0}, EVENTS_ERRORECB{0,0}, ECBDATAPTR{0,0};
	EcbReg TASKS_STARTECB{1,0}, TASKS_STOPECB{2,0};
};
extern NRF_ECB_Type g_Ecb;
#define NRF_ECB (&g_Ecb)
#define NRF_ECB_NS (&g_Ecb)
inline void __DMB() {}
#define NRF_SUCCESS 0U
struct nrf_ecb_hal_data_t { uint8_t key[16], cleartext[16], ciphertext[16]; };
uint32_t sd_softdevice_is_enabled(uint8_t *enabled);
uint32_t sd_ecb_block_encrypt(nrf_ecb_hal_data_t *data);
