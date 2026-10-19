/**
@example data_nvm_demo.cpp
@brief Internal data EEPROM/data flash test using the common Nvm interface.

Destructive test of the last three units of the data memory only. Two units
are scratch; the third keeps a boot stamp for a power-cycle check. On true
EEPROM a unit is 256 bytes. Run once per boot, not continuously.
@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>
#include "coredev/uart.h"
#include "storage/nvm_intrf.h"
#include "idelay.h"
#include "board.h"

static const IOPinCfg_t s_Pins[] = UART_PINS;
alignas(4) static uint8_t s_TxFifo[CFIFO_MEMSIZE(512)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = sizeof(s_TxFifo),
	.pTxMem = s_TxFifo,
	.bDMAMode = false,
};
static UART s_Uart;
static NvmIntrf s_Intrf;
static Nvm s_Mem;
static unsigned s_Fail;

static void Check(bool Ok, const char *pText)
{
	s_Uart.printf("%s: %s\r\n", Ok ? "PASS" : "FAIL", pText);
	if (!Ok) s_Fail++;
}

static void Run(void)
{
	NvmCfg_t cfg = {};
	NvmMcuCfg(cfg);
	uint32_t unit = cfg.EraseSize ? cfg.EraseSize : 256;
	uint64_t size = (uint64_t)unit * 3;
	if (cfg.TotalSize < size || !s_Intrf.Init() ||
		!s_Mem.Init(cfg, &s_Intrf, cfg.TotalSize - size, size))
	{
		s_Uart.printf("FAIL: memory init\r\n");
		return;
	}
	s_Uart.printf("Data NVM: %lu bytes, erase %lu, write %lu\r\n",
		(unsigned long)cfg.TotalSize, (unsigned long)cfg.EraseSize,
		(unsigned long)cfg.WriteGran);
	s_Uart.printf("Test region: 0x%08lX, %lu bytes\r\n",
		(unsigned long)(cfg.BaseAddr + cfg.TotalSize - size), (unsigned long)size);

	const uint32_t magic = 0x4E564D32UL;
	uint32_t stamp[2] = {};
	if (s_Mem.Read(2 * unit, stamp, sizeof(stamp)) != sizeof(stamp))
	{
		Check(false, "read previous stamp");
		return;
	}
	uint32_t boots = stamp[0] == magic ? stamp[1] : 0;
	s_Uart.printf("Previous boot stamp: %lu\r\n", (unsigned long)boots);
	stamp[0] = magic;
	stamp[1] = boots + 1;
	if (cfg.EraseSize && s_Mem.Erase(2 * unit, unit) != 0)
	{
		Check(false, "erase stamp block");
		return;
	}
	Check(s_Mem.Write(2 * unit, stamp, sizeof(stamp)) == sizeof(stamp), "write boot stamp");
	uint32_t back[2] = {};
	Check(s_Mem.Read(2 * unit, back, sizeof(back)) == sizeof(back) &&
		memcmp(stamp, back, sizeof(back)) == 0, "read boot stamp");

	if (cfg.EraseSize && s_Mem.Erase(0, 2 * unit) != 0)
	{
		Check(false, "erase scratch blocks");
		return;
	}
	// Odd address and 131 bytes cross transfer and erase-unit boundaries.
	uint8_t tx[131], rx[131];
	for (unsigned i = 0; i < sizeof(tx); i++) tx[i] = (uint8_t)(i * 37);
	uint32_t off = unit - 63;
	Check(s_Mem.Write(off, tx, sizeof(tx)) == sizeof(tx), "unaligned cross-boundary write");
	Check(s_Mem.Read(off, rx, sizeof(rx)) == sizeof(rx) &&
		memcmp(tx, rx, sizeof(tx)) == 0, "cross-boundary readback");
	memset(tx, 0xFF, sizeof(tx));
	if (cfg.EraseSize)
	{
		Check(s_Mem.Write(off, tx, sizeof(tx)) < 0, "rewrite without erase rejected");
		Check(s_Mem.Erase(1, unit) < 0, "unaligned erase rejected");
		if (s_Mem.Erase(0, 2 * unit) != 0)
		{
			Check(false, "erase before rewrite");
			return;
		}
	}
	Check(s_Mem.Write(off, tx, sizeof(tx)) == sizeof(tx), "write all ones");
	Check(s_Mem.Read(off, rx, sizeof(rx)) == sizeof(rx) &&
		memcmp(tx, rx, sizeof(tx)) == 0, "all ones readback");
	if (!cfg.EraseSize)
	{
		memset(tx, 0, sizeof(tx));
		Check(s_Mem.Write(off, tx, sizeof(tx)) == sizeof(tx), "EEPROM overwrite with zeros");
		Check(s_Mem.Read(off, rx, sizeof(rx)) == sizeof(rx) &&
			memcmp(tx, rx, sizeof(tx)) == 0, "zero readback");
	}
	Check(s_Mem.Read(size, rx, 1) < 0 && s_Mem.Write(size, tx, 1) < 0,
		"out-of-region access rejected");
	s_Uart.printf("%s; power cycle to check boot stamp persistence.\r\n",
		s_Fail ? "FAILURES" : "ALL PASS");
}

int main(void)
{
	if (s_Uart.Init(s_UartCfg)) Run();
	for (;;) msDelay(1000);
}
