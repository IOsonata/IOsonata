#ifndef NRF_H_
#define NRF_H_
#include <stdint.h>
#ifndef PPI_CH_NUM
#define PPI_CH_NUM 20
#endif
#ifndef PPI_GROUP_NUM
#define PPI_GROUP_NUM 6
#endif
typedef struct {
	volatile uint32_t CHEN;
	volatile uint32_t CHENSET;
	volatile uint32_t CHENCLR;
	struct { volatile uint32_t EEP; volatile uint32_t TEP; } CH[20];
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
