#ifndef TEST_NRF53_H
#define TEST_NRF53_H
#include <stdint.h>
#define NRF_DPPI_CHANNELS_USED 0x00000003UL
typedef struct {
	volatile uint32_t CHEN;
	volatile uint32_t CHENSET;
	volatile uint32_t CHENCLR;
	volatile uint32_t CHG[6];
} NRF_DPPIC_Type;
extern NRF_DPPIC_Type g_TestDppic;
#define NRF_DPPIC_S (&g_TestDppic)
#define NRF_DPPIC_NS (&g_TestDppic)
#endif
