#ifndef TEST_INTERRUPT_H
#define TEST_INTERRUPT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t DisableInterrupt(void);
void EnableInterrupt(uint32_t State);
#ifdef __cplusplus
}
#endif
#endif
