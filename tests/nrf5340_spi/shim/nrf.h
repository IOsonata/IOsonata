// Host register model for the production nRF5340 SPI driver.
#pragma once
#include <stdint.h>
#include "nrf_peripherals.h"
struct TestTask {
	void operator=(uint32_t);
};
struct NRF_SPIM_Type {
	TestTask TASKS_START, TASKS_STOP;
	uint32_t TASKS_ACQUIRE, TASKS_RELEASE;
	uint32_t EVENTS_END, EVENTS_STOPPED, EVENTS_ENDRX, EVENTS_ENDTX, EVENTS_ACQUIRED;
	uint32_t ENABLE, INTENCLR, INTENSET, SHORTS, CONFIG, FREQUENCY, ORC, STATUS, DEF;
	struct { uint32_t SCK, MISO, MOSI, CSN; } PSEL;
	struct { uint32_t PTR, MAXCNT, AMOUNT, LIST; } RXD, TXD;
};
using NRF_SPIS_Type = NRF_SPIM_Type;
struct NRF_QSPI_Type { uint32_t unused; };
struct NRF_GPIO_Type { uint32_t PIN_CNF[32]; };
extern NRF_SPIM_Type g_Regs[5];
extern NRF_QSPI_Type g_Qspi;
extern NRF_GPIO_Type g_Gpio;
#define NRF_P0_S (&g_Gpio)
#define NRF_SPIM0_S_BASE (&g_Regs[0])
#define NRF_SPIM1_S_BASE (&g_Regs[1])
#define NRF_SPIM2_S_BASE (&g_Regs[2])
#define NRF_SPIM3_S_BASE (&g_Regs[3])
#define NRF_SPIM4_S_BASE (&g_Regs[4])
#define NRF_QSPI_S_BASE (&g_Qspi)
extern uint32_t SystemCoreClock;
extern bool g_IrqEnabled[6];
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
inline void NVIC_ClearPendingIRQ(int) {}
inline void NVIC_SetPriority(int, int) {}
inline void NVIC_EnableIRQ(int n) { g_IrqEnabled[n] = true; }
enum { SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn,
	SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn,
	SPIM4_IRQn, QSPI_IRQn };
#define GPIO_PIN_CNF_DRIVE_Msk (7U << 8)
#define GPIO_PIN_CNF_DRIVE_H0H1 3U
#define GPIO_PIN_CNF_DRIVE_Pos 8
#define GPIO_PIN_CNF_MCUSEL_Msk (7U << 28)
#define GPIO_PIN_CNF_MCUSEL_Peripheral 3U
#define GPIO_PIN_CNF_MCUSEL_Pos 28
#define SPIM_ENABLE_ENABLE_Enabled 7
#define SPIM_ENABLE_ENABLE_Disabled 0
#define SPIM_ENABLE_ENABLE_Pos 0
#define SPIS_ENABLE_ENABLE_Enabled 2
#define SPIS_ENABLE_ENABLE_Disabled 0
#define SPIS_ENABLE_ENABLE_Pos 0
#define SPIM_INTENSET_END_Msk (1U << 6)
#define SPIM_INTENSET_STOPPED_Msk (1U << 1)
#define SPIM_INTENCLR_STOPPED_Msk (1U << 1)
#define SPIM_CONFIG_ORDER_MsbFirst 0
#define SPIM_CONFIG_ORDER_LsbFirst 1
#define SPIM_CONFIG_CPHA_Pos 1
#define SPIM_CONFIG_CPHA_Leading 0
#define SPIM_CONFIG_CPHA_Trailing 1
#define SPIM_CONFIG_CPOL_Pos 2
#define SPIM_CONFIG_CPOL_ActiveHigh 0
#define SPIM_CONFIG_CPOL_ActiveLow 1
#define SPIS_SHORTS_END_ACQUIRE_Enabled 1
#define SPIS_SHORTS_END_ACQUIRE_Pos 2
#define SPIS_INTENSET_ACQUIRED_Enabled 1
#define SPIS_INTENSET_ACQUIRED_Pos 10
#define SPIS_INTENSET_END_Enabled 1
#define SPIS_INTENSET_END_Pos 6
#define SPIS_INTENSET_ENDRX_Msk (1U << 4)
#define SPIM_FREQUENCY_FREQUENCY_K125 0x02000000U
#define SPIM_FREQUENCY_FREQUENCY_K250 0x04000000U
#define SPIM_FREQUENCY_FREQUENCY_K500 0x08000000U
#define SPIM_FREQUENCY_FREQUENCY_M1 0x10000000U
#define SPIM_FREQUENCY_FREQUENCY_M2 0x20000000U
#define SPIM_FREQUENCY_FREQUENCY_M4 0x40000000U
#define SPIM_FREQUENCY_FREQUENCY_M8 0x80000000U
#define SPIM_FREQUENCY_FREQUENCY_M16 0x0A000000U
#define SPIM_FREQUENCY_FREQUENCY_M32 0x14000000U
