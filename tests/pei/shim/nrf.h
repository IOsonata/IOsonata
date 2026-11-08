#ifndef NRF_H_
#define NRF_H_
#include <stdint.h>
// Deliberately do not supply PPI_CH_NUM or PPI_GROUP_NUM: Nordic MDK
// defines the register arrays, not these legacy driver convenience macros.
#ifndef TEST_PPI_CHANNEL_COUNT
#define TEST_PPI_CHANNEL_COUNT 20
#endif
typedef struct {
	// Nordic PPI group hardware tasks, six groups on the host model.
	struct { volatile uint32_t EN; volatile uint32_t DIS; } TASKS_CHG[6];
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

// EGU at the nRF52 addresses. A test that uses software events maps this
// range in host memory first (TestEguMap in the tests).
#ifndef TEST_EGU_COUNT
#define TEST_EGU_COUNT 6
#endif
typedef struct {
	volatile uint32_t TASKS_TRIGGER[16];
	volatile uint32_t RESERVED[48];
	volatile uint32_t EVENTS_TRIGGERED[16];
	volatile uint32_t RESERVED1[112];
	volatile uint32_t INTEN;
	volatile uint32_t INTENSET;
	volatile uint32_t INTENCLR;
} NRF_EGU_Type;
#define TEST_EGU_BASE 0x40014000UL
#define NRF_EGU0 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x0000UL))
#define NRF_EGU1 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x1000UL))
#if TEST_EGU_COUNT > 2
#define NRF_EGU2 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x2000UL))
#define NRF_EGU3 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x3000UL))
#define NRF_EGU4 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x4000UL))
#define NRF_EGU5 ((NRF_EGU_Type *)(TEST_EGU_BASE + 0x5000UL))
#endif
#endif
