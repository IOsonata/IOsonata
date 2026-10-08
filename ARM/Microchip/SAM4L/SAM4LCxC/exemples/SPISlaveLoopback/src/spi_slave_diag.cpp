// SAM4L-only failure diagnostics for SPISlaveLoopback.
// Copyright (c) 2026 I-SYST inc. MIT License.
#include <stdio.h>
#include "sam4lxxx.h"
#include "board.h"

void SpiSlaveLoopbackDiagnostics(void)
{
	// Capture once, after timeout. Reading SR clears NSSR/error flags.
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	const uint32_t enabled = NVIC_GetEnableIRQ(SPI_IRQn);
	const uint32_t pending = NVIC_GetPendingIRQ(SPI_IRQn);
	const uint32_t imr = SAM4L_SPI->SPI_IMR;
	const uint32_t sr = SAM4L_SPI->SPI_SR;
	const uint32_t mr = SAM4L_SPI->SPI_MR;
	const uint32_t csr = SAM4L_SPI->SPI_CSR[0];
	const GpioPort *master = &SAM4L_GPIO->GPIO_PORT[SPI_MASTER_CS_PORT];
	const GpioPort *slave = &SAM4L_GPIO->GPIO_PORT[SPI_SLAVE_CS_PORT];
	const uint32_t masterMask = 1UL << SPI_MASTER_CS_PIN;
	const uint32_t slaveMask = 1UL << SPI_SLAVE_CS_PIN;
	const uint32_t masterGper = master->GPIO_GPER & masterMask;
	const uint32_t masterOder = master->GPIO_ODER & masterMask;
	const uint32_t masterOvr = master->GPIO_OVR & masterMask;
	const uint32_t masterPvr = master->GPIO_PVR & masterMask;
	const uint32_t slaveGper = slave->GPIO_GPER & slaveMask;
	const uint32_t slavePvr = slave->GPIO_PVR & slaveMask;
	const uint32_t mux = ((slave->GPIO_PMR0 & slaveMask) ? 1U : 0U) |
		((slave->GPIO_PMR1 & slaveMask) ? 2U : 0U) |
		((slave->GPIO_PMR2 & slaveMask) ? 4U : 0U);
	__set_PRIMASK(state);
	printf("SPI SR=%08lx IMR=%08lx MR=%08lx CSR0=%08lx IRQ enabled=%lu pending=%lu PRIMASK=%lu\r\n",
		(unsigned long)sr, (unsigned long)imr, (unsigned long)mr, (unsigned long)csr,
		(unsigned long)enabled, (unsigned long)pending, (unsigned long)state);
	printf("Master CS GPIO=%d output=%d latch=%d pad=%d; slave CS GPIO=%d mux=%lu pad=%d\r\n",
		masterGper != 0U, masterOder != 0U, masterOvr != 0U, masterPvr != 0U,
		slaveGper != 0U, (unsigned long)mux, slavePvr != 0U);
}
