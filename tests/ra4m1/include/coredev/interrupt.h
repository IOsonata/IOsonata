/* TEST SHIM: same PRIMASK save/restore as IOsonata's ARM implementation. */
#ifndef TEST_INTERRUPT_H
#define TEST_INTERRUPT_H
#include <stdint.h>
static inline uint32_t DisableInterrupt(void) {uint32_t v=__get_PRIMASK();__disable_irq();return v;}
static inline void EnableInterrupt(uint32_t v) {__set_PRIMASK(v);}
#endif
