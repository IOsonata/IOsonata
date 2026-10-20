/**-------------------------------------------------------------------------
@file	wba_eeprom_test.cpp

@brief	Host test of the EEPROM demo with the WBA65 board configuration.

Run the shared demo and production Nvm against an M95P32 command model.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#include <cassert>
#include <cstdio>
#define main EepromDemoMain
#include "../../exemples/storage/eeprom_demo.cpp"
#undef main

static uint8_t memory[EEPROM_SIZE];
static bool selected, oledHigh, memoryHigh, wel;
static unsigned command, address, payload, writes, enables, busy, busyReads;

extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int) { assert(false); }
extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int) { assert(false); }
extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY p) { assert(false); return p; }
extern "C" void QuadSPISetMemSize(SPIDev_t *, uint32_t) { assert(false); }
extern "C" bool QuadSPISendCmd(SPIDev_t *, uint8_t, uint32_t, uint8_t, uint32_t, uint8_t)
{ assert(false); return false; }
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) { assert(false); }
extern "C" void IOPinClear(int, int) { assert(false); }
extern "C" void IOPinSet(int port, int pin)
{
	if (port == IOPORTE && pin == 1) { oledHigh = true; }
	else { assert(port == IOPORTH && pin == 3); memoryHigh = true; }
}
static bool Start(DevIntrf_t *, uint32_t cs)
{
	assert(cs == 0 && !selected && oledHigh && memoryHigh);
	selected = true;
	command = address = payload = 0;
	return true;
}
static bool Restart(DevIntrf_t *, uint32_t cs) { assert(selected && cs == 0); return true; }
static void Stop(DevIntrf_t *)
{
	assert(selected);
	selected = false;
	if (command == 6) { wel = true; enables++; }
	if (command == 2) { assert(payload > 0); wel = false; busy = 2; writes++; }
}
static int Tx(DevIntrf_t *, const uint8_t *data, int count)
{
	assert(selected && count > 0);
	if (command == 0)
	{
		command = data[0];
		assert(command == 2 || command == 3 || command == 5 || command == 6 || command == 0x9f);
		if (command == 2 || command == 3)
		{
			assert(count == 4);
			address = (unsigned(data[1]) << 16) | (unsigned(data[2]) << 8) | data[3];
		}
		else { assert(count == 1); }
	}
	else
	{
		assert(command == 2 && wel && busy == 0);
		assert(address + count <= sizeof(memory));
		assert(address / 512 == (address + count - 1) / 512);
		memcpy(memory + address, data, count); // PAGE WRITE permits zero-to-one changes.
		payload += count;
	}
	return count;
}
static int Rx(DevIntrf_t *, uint8_t *data, int count)
{
	assert(selected);
	if (command == 0x9f)
	{
		assert(count == 3);
		data[0] = 0x20; data[1] = 0; data[2] = 0x16;
	}
	else if (command == 5)
	{
		assert(count == 1);
		data[0] = (wel ? 2 : 0) | (busy ? 1 : 0);
		if (busy) { busy--; busyReads++; }
	}
	else
	{
		assert(command == 3 && address + count <= sizeof(memory) && busy == 0);
		memcpy(data, memory + address, count);
	}
	return count;
}
extern "C" bool SPIInit(SPIDev_t *dev, const SPICfg_t *cfg)
{
	assert(cfg->DevNo == 2 && cfg->Phy == SPIPHY_NORMAL);
	assert(cfg->DataSize == 8 && cfg->ClkPol == SPICLKPOL_HIGH);
	assert(cfg->DataPhase == SPIDATAPHASE_FIRST_CLK && cfg->BitOrder == SPIDATABIT_MSB);
	assert(!cfg->bIntEn && !cfg->bDmaEn && cfg->NbIOPins == 5);
	assert(cfg->pIOPinMap[3].PortNo == IOPORTH && cfg->pIOPinMap[3].PinNo == 3);
	assert(cfg->pIOPinMap[4].PortNo == IOPORTE && cfg->pIOPinMap[4].PinNo == 1);
	dev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	dev->DevIntrf.StartTx = Start;
	dev->DevIntrf.StartRx = Restart;
	dev->DevIntrf.StopTx = dev->DevIntrf.StopRx = Stop;
	dev->DevIntrf.TxData = Tx;
	dev->DevIntrf.RxData = Rx;
	atomic_flag_clear(&dev->DevIntrf.bBusy);
	return true;
}
int main()
{
	memset(memory, 0xa5, sizeof(memory));
	assert(EepromInit() && EepromTest());
	assert(writes == 2 && enables == 2 && busyReads == 4 && !selected);
	// A second write must restore ones as well as clear bits, without erase.
	uint8_t replacement[512], readback[512];
	for (unsigned i = 0; i < sizeof(replacement); i++) { replacement[i] = ~memory[256 + i]; }
	assert(EepromWrite(256, replacement, sizeof(replacement)) == 512);
	assert(EepromRead(256, readback, sizeof(readback)) == 512);
	assert(memcmp(replacement, readback, sizeof(readback)) == 0);
	for (unsigned i = 0; i < sizeof(memory); i++)
	{
		if (i < 256 || i >= 768) { assert(memory[i] == 0xa5); }
	}
	assert(writes == 4 && enables == 4 && busyReads == 8 && !selected);
	puts("WBA EEPROM: shared demo, page split, WREN/WIP, overwrite and guards PASS");
}
