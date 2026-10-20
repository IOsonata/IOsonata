// Host stand in for the SCB registers SystemInit writes, included before the
// device header by tests/stm32g4/run.py
#ifndef __TEST_SCB_H__
#define __TEST_SCB_H__

#include <stdint.h>

typedef struct {
	uint32_t VTOR;
	uint32_t CPACR;
} TestScb_t;

#ifdef __cplusplus
extern "C" TestScb_t g_TestScb;
#else
extern TestScb_t g_TestScb;
#endif

#define SCB		(&g_TestScb)

#endif
