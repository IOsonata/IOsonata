/**-------------------------------------------------------------------------
@file	i2c_diagnostics.cpp

@brief ESP32C6 I2C slave callback register snapshots.

Copyright (c) 2026 I-SYST inc. All rights reserved.
SPDX-License-Identifier: MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include "board.h"

static uint32_t s_Registers[I2C_TEST_TRACE_SIZE][6];

void I2cMasterSlaveCapture(int Index)
{
	if (Index < 0 || Index >= I2C_TEST_TRACE_SIZE)
	{
		return;
	}
	// Read-only status and configuration. Never read DATA: it pops RX FIFO.
	s_Registers[Index][0] = ESP32_I2C_SR_REG(ESP32_I2C0_BASE);
	s_Registers[Index][1] = ESP32_I2C_FIFO_ST_REG(ESP32_I2C0_BASE);
	s_Registers[Index][2] = ESP32_I2C_CTR_REG(ESP32_I2C0_BASE);
	s_Registers[Index][3] = ESP32_I2C_FIFO_CONF_REG(ESP32_I2C0_BASE);
	s_Registers[Index][4] = ESP32_I2C_INT_RAW_REG(ESP32_I2C0_BASE);
	s_Registers[Index][5] = ESP32_I2C_SCL_STRETCH_CONF_REG(ESP32_I2C0_BASE);
}

void I2cMasterSlavePrint(int Index)
{
	const uint32_t *values;

	if (Index < 0 || Index >= I2C_TEST_TRACE_SIZE)
	{
		return;
	}
	values = s_Registers[Index];
	printf("    HP SR=%08lx FIFO_ST=%08lx CTR=%08lx\r\n",
		(unsigned long)values[0], (unsigned long)values[1], (unsigned long)values[2]);
	printf("    HP FIFO_CONF=%08lx RAW=%08lx STRETCH=%08lx\r\n",
		(unsigned long)values[3], (unsigned long)values[4], (unsigned long)values[5]);
}
