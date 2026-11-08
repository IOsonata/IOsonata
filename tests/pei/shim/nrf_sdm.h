#ifndef NRF_SDM_H__
#define NRF_SDM_H__
#include <stdint.h>
#include "nrf_soc.h"
// SoftDevice info structure of the test, the magic number at offset 4
#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t g_TestSdInfo[2];
uint32_t sd_softdevice_is_enabled(uint8_t *p_softdevice_enabled);
#ifdef __cplusplus
}
#endif
#define SOFTDEVICE_INFO_STRUCT_ADDRESS	((uintptr_t)g_TestSdInfo)
#endif
