#pragma once
#include <stdint.h>
#define __IO volatile
#define __I volatile const
#define __O volatile
#define __FPU_USED 0
#define __DSB() ((void)0)
#define __ISB() ((void)0)
typedef struct { volatile uint32_t VTOR; } TestSCB;
extern TestSCB test_scb;
#define SCB (&test_scb)
