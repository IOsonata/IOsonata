/**-------------------------------------------------------------------------
@file	spi_master_slave_diag.cpp

@brief	R9A02G021 SPI master/slave timeout diagnostics

Called by the shared example only after a completion timeout. First read
the GPIO, SAU and interrupt state without consuming received data. Then
stop both SPI interfaces and check the connections as GPIO, followed by
the chip-select rising-edge interrupt. The failed SPI test is not resumed.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdio.h>

#include "coredev/spi.h"
#include "iopinctrl.h"
#include "interrupt_r9a02.h"
#include "sau_r9a02.h"
#include "system_r9a02.h"
#include "clic.h"
#include "board.h"

typedef struct __Spi_Diag_Connection {
	const char *pName;
	int OutPort;
	int OutPin;
	int InPort;
	int InPin;
} SpiDiagConnection_t;

static const SpiDiagConnection_t s_Connections[] = {
	{"SCK", SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN, SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN},
	{"MOSI", SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN, SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN},
	{"MISO", SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN, SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN},
	{"CS", SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN, SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN},
};

static volatile unsigned s_CsIrqCount;

extern SPI g_SpiMaster;
extern SPI g_SpiSlave;

static void SpiCsEvent(int IntNo, void *pCtx)
{
	(void)IntNo;
	(void)pCtx;
	s_CsIrqCount = s_CsIrqCount + 1U;
}

static void SpiGpioDiagnostics(void)
{
	const SpiDiagConnection_t *connection;
	unsigned i;
	unsigned before;
	unsigned count;
	int outLow;
	int inLow;
	int outHigh;
	int inHigh;
	int line;
	bool pass;

	// Release both channels and the SPI slave's CS interrupt before changing
	// pin functions. Every connection retains its normal signal direction.
	g_SpiMaster.PowerOff();
	g_SpiSlave.PowerOff();
	printf("SPI stopped; GPIO connection and CS interrupt checks\r\n");
	for (i = 0; i < sizeof(s_Connections) / sizeof(s_Connections[0]); i++)
	{
		connection = &s_Connections[i];
		IOPinConfig(connection->InPort, connection->InPin, IOPINOP_GPIO,
			IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
		IOPinSet(connection->OutPort, connection->OutPin);
		IOPinConfig(connection->OutPort, connection->OutPin, IOPINOP_GPIO,
			IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		IOPinClear(connection->OutPort, connection->OutPin);
		R9a02DelayUs(1000);
		outLow = IOPinRead(connection->OutPort, connection->OutPin);
		inLow = IOPinRead(connection->InPort, connection->InPin);
		IOPinSet(connection->OutPort, connection->OutPin);
		R9a02DelayUs(1000);
		outHigh = IOPinRead(connection->OutPort, connection->OutPin);
		inHigh = IOPinRead(connection->InPort, connection->InPin);
		pass = outLow == 0 && inLow == 0 && outHigh == 1 && inHigh == 1;
		printf("GPIO %s P%d%02d->P%d%02d low=%d/%d high=%d/%d %s\r\n",
			connection->pName, connection->OutPort, connection->OutPin,
			connection->InPort, connection->InPin, outLow, inLow, outHigh, inHigh,
			pass ? "PASS" : "FAIL");
	}

	s_CsIrqCount = 0;
	line = IOPinAllocateInterrupt(SPI_SLAVE_INT_PRIO, SPI_SLAVE_CS_PORT,
		SPI_SLAVE_CS_PIN, IOPINSENSE_HIGH_TRANSITION, SpiCsEvent, NULL);
	if (line >= 0)
	{
		IOPinClear(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
		R9a02DelayUs(1000);
		outLow = IOPinRead(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
		inLow = IOPinRead(SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN);
		before = s_CsIrqCount;
		IOPinSet(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
		R9a02DelayUs(1000);
		outHigh = IOPinRead(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
		inHigh = IOPinRead(SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN);
		count = s_CsIrqCount - before;
		IOPinDisableInterrupt(line);
		pass = outLow == 0 && inLow == 0 && outHigh == 1 && inHigh == 1 && count == 1;
		printf("GPIO CS IRQ%d low=%d/%d high=%d/%d rising callbacks=%u %s\r\n",
			line, outLow, inLow, outHigh, inHigh, count, pass ? "PASS" : "FAIL");
	}
	else
	{
		printf("GPIO CS interrupt allocation FAIL\r\n");
	}
	for (i = 0; i < sizeof(s_Connections) / sizeof(s_Connections[0]); i++)
	{
		connection = &s_Connections[i];
		IOPinDisable(connection->OutPort, connection->OutPin);
		IOPinDisable(connection->InPort, connection->InPin);
	}
}

static void SpiCsDiagnostics(const char *pName, int Port, int Pin)
{
	R9A02_PORT_Type *port = R9A02_PORT(Port);
	uint32_t pfs = R9A02_PFS(Port, Pin);
	uint32_t control = port->PCNTR1;
	uint32_t input = port->PCNTR2;
	uint16_t pidr = port->PIDR;

	printf("%s CS P%d%02d PFS=%08lx pullup=%d IRQ=%d output=%d peripheral=%d "
		"PCNTR1=%08lx PCNTR2=%08lx PIDR=%04x\r\n",
		pName, Port, Pin, (unsigned long)pfs,
		(pfs & R9A02_PFS_PCR) != 0U, (pfs & R9A02_PFS_ISEL) != 0U,
		(pfs & R9A02_PFS_PDR) != 0U, (pfs & R9A02_PFS_PMR) != 0U,
		(unsigned long)control, (unsigned long)input, (unsigned)pidr);
}

void SpiMasterSlaveDiagnostics(void)
{
	R9A02_SAU_Type *sau = R9a02SauReg(1);
	uintptr_t status;
	uint32_t route;
	uint32_t clic;
	unsigned slot;
	uint8_t event;
	bool pinIrq;

	__asm volatile ("csrr %0, mstatus" : "=r" (status));
	// PRWCNTR is write-only; a read does not report the wait setting.
	printf("SPI timeout MSTATUS=%08lx PWPR=%02x\r\n",
		(unsigned long)status, (unsigned)R9A02_PFS_PWPR);
	SpiCsDiagnostics("Master", SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
	SpiCsDiagnostics("Slave", SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN);
	printf("SPI master pins PFS SCK=%08lx MISO=%08lx MOSI=%08lx\r\n",
		(unsigned long)R9A02_PFS(SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN),
		(unsigned long)R9A02_PFS(SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN),
		(unsigned long)R9A02_PFS(SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN));
	printf("SPI slave pins PFS SCK=%08lx MISO=%08lx MOSI=%08lx\r\n",
		(unsigned long)R9A02_PFS(SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN),
		(unsigned long)R9A02_PFS(SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN),
		(unsigned long)R9A02_PFS(SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN));
	printf("SAU1 SE=%04x SO=%04x SOE=%04x SPS=%04x "
		"SPI20 SMR=%04x SCR=%04x SSR=%04x SPI21 SMR=%04x SCR=%04x SSR=%04x\r\n",
		(unsigned)sau->SE, (unsigned)sau->SO, (unsigned)sau->SOE, (unsigned)sau->SPS,
		(unsigned)sau->SMR[0], (unsigned)sau->SCR[0], (unsigned)sau->SSR[0],
		(unsigned)sau->SMR[1], (unsigned)sau->SCR[1], (unsigned)sau->SSR[1]);
	for (slot = 0; slot < R9A02_ICU_IELSR_CNT; slot++)
	{
		event = R9a02IntEvent((int)slot);
		pinIrq = event >= R9A02_EVT_PORT_IRQ0 && event <= R9A02_EVT_PORT_IRQ7;
		if (!pinIrq && event != R9A02_EVT_SAU1_ENDI0 &&
			event != R9A02_EVT_SAU1_ENDI1)
		{
			continue;
		}
		route = R9A02_ICU->IELSR[slot];
		clic = R9A02_CLIC_ID_IELSR0 + slot;
		printf("SPI event=%02x IELSR%u=%08lx CLIC%lu IE=%02x IP=%02x CTL=%02x\r\n",
			(unsigned)event, slot, (unsigned long)route, (unsigned long)clic,
			(unsigned)CLIC_INT_IE_REG(R9A02_CLIC_CTRL_BASE, clic),
			(unsigned)CLIC_INT_IP_REG(R9A02_CLIC_CTRL_BASE, clic),
			(unsigned)CLIC_INT_CTL_REG(R9A02_CLIC_CTRL_BASE, clic));
		if (pinIrq)
		{
			printf("SPI pin IRQ%u IRQCR=%02x\r\n",
				(unsigned)(event - R9A02_EVT_PORT_IRQ0),
				(unsigned)R9A02_ICU->IRQCR[event - R9A02_EVT_PORT_IRQ0]);
		}
	}
	SpiGpioDiagnostics();
}
