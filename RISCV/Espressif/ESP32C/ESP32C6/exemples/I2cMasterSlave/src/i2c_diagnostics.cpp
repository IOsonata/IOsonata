/**-------------------------------------------------------------------------
@file	i2c_diagnostics.cpp

@brief ESP32C6 I2C slave callback register snapshots.

Copyright (c) 2026 I-SYST inc. All rights reserved.
SPDX-License-Identifier: MIT
----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "idelay.h"

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
	printf("T%d HP SR=%08lx FIFO_ST=%08lx CTR=%08lx\r\n", Index,
		(unsigned long)values[0], (unsigned long)values[1], (unsigned long)values[2]);
	fflush(stdout);
	usDelay(I2C_TEST_TRACE_DELAY_US);
	printf("T%d HP FIFO_CONF=%08lx RAW=%08lx STRETCH=%08lx\r\n", Index,
		(unsigned long)values[3], (unsigned long)values[4], (unsigned long)values[5]);
	fflush(stdout);
	usDelay(I2C_TEST_TRACE_DELAY_US);
}

typedef struct {
	uint32_t Expected;
	uint32_t Raw;
	uint32_t State;
	uint32_t Control;
	uint32_t Fifo;
	uint32_t TimeoutReg;
	uint32_t Command[4];
	bool TimedOut;
} I2cMasterFailure_t;

static I2cMasterFailure_t s_MasterFailures[24];
static unsigned s_MasterFailureCount;
static bool s_MasterFailureOverflow;

void Esp32I2cTraceFailure(uintptr_t Base, uint32_t Done,
	uint32_t Status, bool Timeout)
{
	I2cMasterFailure_t *entry;
	unsigned i;

	if (Base != ESP32_LP_I2C_BASE)
	{
		return;
	}
	if (s_MasterFailureCount >= 24)
	{
		s_MasterFailureOverflow = true;
		return;
	}
	entry = &s_MasterFailures[s_MasterFailureCount];
	entry->Expected = Done;
	entry->Raw = Status;
	entry->State = ESP32_I2C_SR_REG(Base);
	entry->Control = ESP32_I2C_CTR_REG(Base);
	entry->Fifo = ESP32_I2C_FIFO_ST_REG(Base);
	entry->TimeoutReg = ESP32_I2C_TO_REG(Base);
	entry->TimedOut = Timeout;
	for (i = 0; i < 4; i++)
	{
		entry->Command[i] = ESP32_I2C_COMD_REG(Base, i);
	}
	s_MasterFailureCount++;
}

typedef struct {
	unsigned Phase;
	int Queued;
	int Count;
	uint32_t State;
	uint32_t Fifo;
	uint32_t Raw;
	uint32_t Pending;
} I2cSlaveTxTrace_t;

static I2cSlaveTxTrace_t s_SlaveTxTrace[128];
static unsigned s_SlaveTxTraceCount;
static bool s_SlaveTxTraceOverflow;

bool Esp32I2cTraceSlaveBytes()
{
	return true;
}

void Esp32I2cTraceSlaveTx(uintptr_t Base, unsigned Phase, int Queued, int Count)
{
	I2cSlaveTxTrace_t *entry;

	if (Base != ESP32_I2C0_BASE)
	{
		return;
	}
	if (s_SlaveTxTraceCount >= 128)
	{
		s_SlaveTxTraceOverflow = true;
		return;
	}
	entry = &s_SlaveTxTrace[s_SlaveTxTraceCount];
	entry->Phase = Phase;
	entry->Queued = Queued;
	entry->Count = Count;
	entry->State = ESP32_I2C_SR_REG(Base);
	entry->Fifo = ESP32_I2C_FIFO_ST_REG(Base);
	entry->Raw = ESP32_I2C_INT_RAW_REG(Base);
	entry->Pending = ESP32_I2C_INT_STATUS_REG(Base);
	s_SlaveTxTraceCount++;
}

static void I2cMasterSlavePrintTx()
{
	const I2cSlaveTxTrace_t *entry;
	unsigned i;

	printf("HP slave TX trace: %u entries%s\r\n", s_SlaveTxTraceCount,
		s_SlaveTxTraceOverflow ? " (capacity exceeded)" : "");
	fflush(stdout);
	usDelay(I2C_TEST_TRACE_DELAY_US);
	for (i = 0; i < s_SlaveTxTraceCount; i++)
	{
		entry = &s_SlaveTxTrace[i];
		printf("S%u %s queued=%d count=%d SR=%08lx\r\n", i,
			entry->Phase == 0 ? "BEFORE" : entry->Phase == 1 ? "FILLED" :
			entry->Phase == 2 ? "DONE" : "IRQ",
			entry->Queued, entry->Count, (unsigned long)entry->State);
		fflush(stdout);
		usDelay(I2C_TEST_TRACE_DELAY_US);
		printf("S%u FIFO_ST=%08lx RAW=%08lx INT=%08lx\r\n", i,
			(unsigned long)entry->Fifo, (unsigned long)entry->Raw,
			(unsigned long)entry->Pending);
		fflush(stdout);
		usDelay(I2C_TEST_TRACE_DELAY_US);
	}
}

void I2cMasterSlavePrintFailures()
{
	const I2cMasterFailure_t *entry;
	unsigned i;

	I2cMasterSlavePrintTx();
	printf("LP master failure trace: %u entries%s\r\n", s_MasterFailureCount,
		s_MasterFailureOverflow ? " (capacity exceeded)" : "");
	fflush(stdout);
	usDelay(I2C_TEST_TRACE_DELAY_US);
	for (i = 0; i < s_MasterFailureCount; i++)
	{
		entry = &s_MasterFailures[i];
		printf("L%u %s expected=%08lx RAW=%08lx SR=%08lx\r\n", i,
			entry->TimedOut ? "TIMEOUT" : "ERROR",
			(unsigned long)entry->Expected, (unsigned long)entry->Raw,
			(unsigned long)entry->State);
		fflush(stdout);
		usDelay(I2C_TEST_TRACE_DELAY_US);
		printf("L%u CTR=%08lx FIFO_ST=%08lx TO=%08lx\r\n", i,
			(unsigned long)entry->Control, (unsigned long)entry->Fifo,
			(unsigned long)entry->TimeoutReg);
		fflush(stdout);
		usDelay(I2C_TEST_TRACE_DELAY_US);
		printf("L%u CMD0=%08lx CMD1=%08lx CMD2=%08lx CMD3=%08lx\r\n", i,
			(unsigned long)entry->Command[0], (unsigned long)entry->Command[1],
			(unsigned long)entry->Command[2], (unsigned long)entry->Command[3]);
		fflush(stdout);
		usDelay(I2C_TEST_TRACE_DELAY_US);
	}
}
