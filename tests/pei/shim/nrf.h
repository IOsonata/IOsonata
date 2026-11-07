#ifndef NRF_H_
#define NRF_H_
#include <stdint.h>
// Deliberately do not supply PPI_CH_NUM or PPI_GROUP_NUM: Nordic MDK
// defines the register arrays, not these legacy driver convenience macros.
#ifndef TEST_PPI_CHANNEL_COUNT
#define TEST_PPI_CHANNEL_COUNT 20
#endif
typedef struct {
	volatile uint32_t CHEN;
	volatile uint32_t CHENSET;
	volatile uint32_t CHENCLR;
	struct { volatile uint32_t EEP; volatile uint32_t TEP; } CH[TEST_PPI_CHANNEL_COUNT];
	struct { volatile uint32_t TEP; } FORK[20];
	volatile uint32_t CHG[6];
} NRF_PPI_Type;
#ifdef __cplusplus
extern "C" {
#endif
extern NRF_PPI_Type g_TestPpi;
#ifdef __cplusplus
}
#endif
#define NRF_PPI (&g_TestPpi)
#endif
