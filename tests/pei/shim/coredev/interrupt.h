#ifndef TEST_INTERRUPTS_H
#define TEST_INTERRUPTS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern int g_critical_depth;
#ifdef __cplusplus
}
#endif
static inline uint32_t DisableInterrupt(void) { ++g_critical_depth; return 0; }
static inline void EnableInterrupt(uint32_t x) { (void)x; --g_critical_depth; }
#endif
