/**-------------------------------------------------------------------------
@file	spi_diagnostics.cpp

@brief ESP32C6 SPI master/slave failure diagnostics.

Copyright (c) 2026 I-SYST inc. All rights reserved.
SPDX-License-Identifier: MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>

#include "board.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "esp32xx_gpio.h"
#include "esp32xx_spi.h"
#include "esp32xx_irq.h"

void SpiMasterSlaveDiagnostics(void)
{
	uint32_t values[25];
	uint32_t pads[8];
	uint32_t outputs[8];
	uint32_t status;
	uint32_t enable;
	uint32_t pending;
	uint32_t vector;
	unsigned int i;

	// Capture before UART output. These reads do not change the armed slave.
	values[0] = ESP32_GPIO_OUT_REG32;
	values[1] = ESP32_GPIO_ENABLE_REG32;
	values[2] = ESP32_GPIO_IN_REG32;
	values[3] = ESP32_LP_AON_GPIO_MUX_REG;
	values[4] = ESP32_SPI_CMD_REG(ESP32_SPI2_BASE);
	values[5] = ESP32_SPI_USER_REG(ESP32_SPI2_BASE);
	values[6] = ESP32_SPI_MISC_REG(ESP32_SPI2_BASE);
	values[7] = ESP32_SPI_SLAVE_REG(ESP32_SPI2_BASE);
	values[8] = ESP32_SPI_SLAVE1_REG(ESP32_SPI2_BASE);
	values[9] = ESP32_SPI_DMA_CONF_REG(ESP32_SPI2_BASE);
	values[10] = ESP32_SPI_DMA_INT_ENA_REG(ESP32_SPI2_BASE);
	values[11] = ESP32_SPI_DMA_INT_RAW_REG(ESP32_SPI2_BASE);
	values[12] = ESP32_SPI_DMA_INT_ST_REG(ESP32_SPI2_BASE);
	values[13] = ESP32_SPI_CLK_GATE_REG(ESP32_SPI2_BASE);
	values[14] = ESP32_GPIO_FUNC_IN_REG32(ESP32_SPI_CS_SIGNAL);
	values[15] = ESP32_GPIO_FUNC_IN_REG32(ESP32_SPI_SCK_SIGNAL);
	values[16] = ESP32_GPIO_FUNC_IN_REG32(ESP32_SPI_MOSI_SIGNAL);
	values[17] = ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CONF_OFFSET);
	values[18] = ESP32_REG32(ESP32_PCR_BASE + ESP32_SPI_PCR_CLKM_CONF_OFFSET);
	values[19] = ESP32_INTMTX_SOURCE_MAP_REG(ESP32_SPI_INTR_SOURCE_SPI2);
	values[20] = ESP32_CPU_INT_ENABLE_REG;
	values[21] = ESP32_CPU_INT_TYPE_REG;
	values[22] = ESP32_CPU_INT_PRI_REG(12);
	values[23] = ESP32_CPU_INT_THRESH_REG;
	values[24] = ESP32_CPU_INT_EIP_STATUS_REG;
	__asm volatile("csrr %0, mstatus" : "=r"(status));
	__asm volatile("csrr %0, mie" : "=r"(enable));
	__asm volatile("csrr %0, mip" : "=r"(pending));
	__asm volatile("csrr %0, mtvec" : "=r"(vector));
	for (i = 0; i < 8; i++)
	{
		pads[i] = ESP32_IOMUX_PAD_REG(i);
		outputs[i] = ESP32_GPIO_FUNC_OUT_REG32(i);
	}

	printf("C6 SPI failure snapshot (after CS release)\r\n");
	printf("  GPIO_OUT       %08lx\r\n", (unsigned long)values[0]);
	printf("  GPIO_ENABLE    %08lx\r\n", (unsigned long)values[1]);
	printf("  GPIO_IN        %08lx\r\n", (unsigned long)values[2]);
	printf("  LP_MUX         %08lx\r\n", (unsigned long)values[3]);
	printf("  SPI_CMD        %08lx\r\n", (unsigned long)values[4]);
	printf("  SPI_USER       %08lx\r\n", (unsigned long)values[5]);
	printf("  SPI_MISC       %08lx\r\n", (unsigned long)values[6]);
	printf("  SPI_SLAVE      %08lx\r\n", (unsigned long)values[7]);
	printf("  SPI_SLAVE1     %08lx\r\n", (unsigned long)values[8]);
	printf("  SPI_DMA_CONF   %08lx\r\n", (unsigned long)values[9]);
	printf("  SPI_DMA_INT_ENA %08lx\r\n", (unsigned long)values[10]);
	printf("  SPI_DMA_INT_RAW %08lx\r\n", (unsigned long)values[11]);
	printf("  SPI_DMA_INT_ST %08lx\r\n", (unsigned long)values[12]);
	printf("  SPI_CLK_GATE   %08lx\r\n", (unsigned long)values[13]);
	printf("  MATRIX_CS      %08lx\r\n", (unsigned long)values[14]);
	printf("  MATRIX_SCK     %08lx\r\n", (unsigned long)values[15]);
	printf("  MATRIX_MOSI    %08lx\r\n", (unsigned long)values[16]);
	printf("  PCR_SPI        %08lx\r\n", (unsigned long)values[17]);
	printf("  PCR_CLOCK      %08lx\r\n", (unsigned long)values[18]);
	printf("  IRQ_MAP        %08lx\r\n", (unsigned long)values[19]);
	printf("  IRQ_ENABLE     %08lx\r\n", (unsigned long)values[20]);
	printf("  IRQ_TYPE       %08lx\r\n", (unsigned long)values[21]);
	printf("  IRQ_PRIORITY   %08lx\r\n", (unsigned long)values[22]);
	printf("  IRQ_THRESHOLD  %08lx\r\n", (unsigned long)values[23]);
	printf("  PLIC_PENDING   %08lx\r\n", (unsigned long)values[24]);
	printf("  MSTATUS %08lx MIE %08lx\r\n", (unsigned long)status, (unsigned long)enable);
	printf("  MIP %08lx MTVEC %08lx\r\n", (unsigned long)pending, (unsigned long)vector);
	printf("  Pin IO_MUX   OUT_MATRIX\r\n");
	for (i = 0; i < 8; i++)
	{
		printf("  %u   %08lx %08lx\r\n", i, (unsigned long)pads[i], (unsigned long)outputs[i]);
	}
	printf("C6 SPI snapshot complete\r\n");
	fflush(stdout);
}

// Run before either SPI interface owns the pins. SPI Init restores its pin map.
bool SpiMasterSlaveCheckPins(void)
{
	const int outputs[] = {SPI_MASTER_CS_PIN, SPI_MASTER_SCK_PIN,
		SPI_MASTER_MOSI_PIN, SPI_SLAVE_MISO_PIN};
	const int inputs[] = {SPI_SLAVE_CS_PIN, SPI_SLAVE_SCK_PIN,
		SPI_SLAVE_MOSI_PIN, SPI_MASTER_MISO_PIN};
	int low;
	int high;
	int lowAgain;
	unsigned int i;
	bool pass = true;

	printf("C6 diagnostics: %s UART, GPIO jumper check\r\n",
		SPI_TEST_UART_INT_ENABLE ? "interrupt" : "polling");
	for (i = 0; i < 4; i++)
	{
		IOPinConfig(0, outputs[i], IOPINOP_GPIO, IOPINDIR_INPUT,
			IOPINRES_NONE, IOPINTYPE_NORMAL);
		IOPinConfig(0, inputs[i], IOPINOP_GPIO, IOPINDIR_INPUT,
			IOPINRES_NONE, IOPINTYPE_NORMAL);
	}
	for (i = 0; i < 4; i++)
	{
		IOPinClear(0, outputs[i]);
		IOPinConfig(0, outputs[i], IOPINOP_GPIO, IOPINDIR_OUTPUT,
			IOPINRES_NONE, IOPINTYPE_NORMAL);
		usDelay(10);
		low = IOPinRead(0, inputs[i]);
		IOPinSet(0, outputs[i]);
		usDelay(10);
		high = IOPinRead(0, inputs[i]);
		IOPinClear(0, outputs[i]);
		usDelay(10);
		lowAgain = IOPinRead(0, inputs[i]);
		IOPinConfig(0, outputs[i], IOPINOP_GPIO, IOPINDIR_INPUT,
			IOPINRES_NONE, IOPINTYPE_NORMAL);
		printf("  GPIO%d -> GPIO%d: low=%d high=%d low=%d %s\r\n",
			outputs[i], inputs[i], low, high, lowAgain,
			low == 0 && high != 0 && lowAgain == 0 ? "PASS" : "FAIL");
		if (low != 0 || high == 0 || lowAgain != 0)
		{
			pass = false;
		}
	}
	return pass;
}
