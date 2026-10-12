/* Host substitutes for IOsonata's CMSIS-backed interrupt helpers. */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t DisableInterrupt(void);
void EnableInterrupt(uint32_t State);
#ifdef __cplusplus
}
#endif
