/**-------------------------------------------------------------------------
@example	sdcard_test.cpp

@brief	SD card read and write test

Sets up the SD card (SDCard) on the interface of the board, prints the card,
then tests reads and writes on a group of sectors whose content it saves
first and writes back at the end: single sector writes and reads, multiple
sector writes and reads, the multiple sector data read back one sector at a
time, buffers not aligned on 4 bytes, the last sector of the card, a read
past its end refused. Prints the result of each check and ends with
"ALL PASS".

The card is on the SD/MMC host of the MCU when board.h gives SDMMC_PINS, else
on an SPI interface in SPI mode. board.h gives:

	SD/MMC host:
	SDMMC_PINS			pin map: CLK, CMD, D0, D1 to D3, then the pins of the
						MCU host (card detect, supply switch, ...)
	SDMMC_DEVNO			optional, host instance, 0
	SDMMC_BUS_WIDTH		optional, data lines wired, 4
	SDMMC_RATE			optional, highest card clock, 25000000
	SDMMC_CARD_DETECT	optional, card detect wired, false
	SDMMC_POWER_CTRL	optional, card supply switched by the host, false

	SPI:
	SDCARD_SPI_PINS		pin map: SCK, MISO, MOSI, CS
	SDCARD_SPI_DEVNO	optional, SPI instance, 0
	SDCARD_SPI_RATE		optional, highest clock, 20000000

	SDCARD_TEST_SECT	optional, first sector of the test, the middle of
						the card without it

The content of the test sectors is put back unless the test stops on a
failure of the card. Use a card without data to keep.

With UART_RETARGET and the UART pins, the Release build prints on the UART
at 1000000 baud 8N1, the Debug build uses semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "coredev/uart.h"
#include "storage/sdcard.h"
#include "stddev.h"

#include "board.h"

#ifdef SDMMC_PINS
#include "coredev/sdmmc.h"
#else
#include "coredev/spi.h"
#endif

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifdef UART_RETARGET
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
#endif

#ifdef SDMMC_PINS
// The SD/MMC host of the MCU
#ifndef SDMMC_DEVNO
#define SDMMC_DEVNO			0
#endif
#ifndef SDMMC_BUS_WIDTH
#define SDMMC_BUS_WIDTH		4
#endif
#ifndef SDMMC_RATE
#define SDMMC_RATE			25000000
#endif
#ifndef SDMMC_CARD_DETECT
#define SDMMC_CARD_DETECT	false
#endif
#ifndef SDMMC_POWER_CTRL
#define SDMMC_POWER_CTRL	false
#endif

static const IOPinCfg_t s_SdPins[] = SDMMC_PINS;

static const SdmmcCfg_t s_SdIntrfCfg = {
	.DevNo = SDMMC_DEVNO,
	.pIOPinMap = s_SdPins,
	.NbIOPins = sizeof(s_SdPins) / sizeof(IOPinCfg_t),
	.Rate = SDMMC_RATE,
	.BusWidth = SDMMC_BUS_WIDTH,
	.bCardDetect = SDMMC_CARD_DETECT,
	.bPowerCtrl = SDMMC_POWER_CTRL,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCB = NULL,
};

static Sdmmc s_SdIntrf;
#else
// SPI mode: 0xFF sent on reads, the chip select low for a whole command,
// transfers ended in the call
#ifndef SDCARD_SPI_DEVNO
#define SDCARD_SPI_DEVNO	0
#endif
#ifndef SDCARD_SPI_RATE
#define SDCARD_SPI_RATE		20000000
#endif

static const IOPinCfg_t s_SdPins[] = SDCARD_SPI_PINS;

static const SPICfg_t s_SdIntrfCfg = {
	.DevNo = SDCARD_SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SdPins,
	.NbIOPins = sizeof(s_SdPins) / sizeof(IOPinCfg_t),
	.Rate = SDCARD_SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.DummyByte = 0xFF,
	.EvtCB = NULL,
};

static SPI s_SdIntrf;
#endif

#define SECT_SIZE			SDCARD_SECT_SIZE
#define NB_TEST				16		// Sectors of the test

alignas(4) static uint8_t s_Save[NB_TEST * SECT_SIZE];
alignas(4) static uint8_t s_Out[NB_TEST * SECT_SIZE + 4];
alignas(4) static uint8_t s_In[NB_TEST * SECT_SIZE + 4];
static SDCard s_SdCard;
static int s_Fail = 0;

static void Check(bool Ok, const char *pName)
{
	printf("%s : %s\r\n", Ok ? "PASS" : "FAIL", pName);
	if (Ok == false)
	{
		s_Fail++;
	}
}

// Bytes that differ from one sector and one pass to the next
static void Fill(uint8_t *p, uint32_t Len, uint32_t Seed)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		Seed = Seed * 1103515245UL + 12345UL;
		p[i] = (uint8_t)(Seed >> 16);
	}
}

static void PrintCard()
{
	static const char * const s_Type[] = { "none", "MMC", "SDSC version 1", "SDSC", "SDHC/SDXC" };
	const uint8_t *cid = s_SdCard.Cid();

	printf("%s, %lu sectors, %lu MB, clock %lu Hz\r\n", s_Type[s_SdCard.Type()],
		   (unsigned long)s_SdCard.GetNbSect(), (unsigned long)(s_SdCard.GetSize() / 1024UL),
		   (unsigned long)s_SdCard.Rate());
	printf("CID");
	for (int i = 0; i < 16; i++)
	{
		printf(" %02X", cid[i]);
	}
	printf("\r\n");
}

int main()
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
#endif

	printf("\r\nSD card read write test\r\n");

	if (s_SdIntrf.Init(s_SdIntrfCfg) == false)
	{
		printf("Interface init failed\r\n");

		return 1;
	}
	if (s_SdCard.Init(&s_SdIntrf) == false)
	{
		printf("No card or card init failed\r\n");

		return 1;
	}
	PrintCard();

	uint32_t nb = s_SdCard.GetNbSect();

	if (nb < 2 * NB_TEST)
	{
		printf("Card size not supported\r\n");

		return 1;
	}

#ifdef SDCARD_TEST_SECT
	uint32_t first = SDCARD_TEST_SECT;
#else
	uint32_t first = (nb / 2) & ~(uint32_t)(NB_TEST - 1);
#endif
	printf("Test sectors %lu to %lu\r\n", (unsigned long)first, (unsigned long)(first + NB_TEST - 1));

	// Content of the test sectors, put back at the end
	bool ok = s_SdCard.ReadSect(first, s_Save, NB_TEST) == NB_TEST;
	Check(ok, "save the test sectors");
	if (ok == false)
	{
		printf("\r\nFAILURES, nothing written\r\n");

		return 1;
	}

	// One sector
	Fill(s_Out, SECT_SIZE, first);
	ok = s_SdCard.SectWrite(first, s_Out);
	memset(s_In, 0, SECT_SIZE);
	ok = ok && s_SdCard.SectRead(first, s_In);
	Check(ok && memcmp(s_In, s_Out, SECT_SIZE) == 0, "single sector write, read");

	// All the test sectors in one transfer, read back in one transfer, then
	// one sector at a time
	Fill(s_Out, sizeof(s_Save), first + 1);
	ok = s_SdCard.WriteSect(first, s_Out, NB_TEST) == NB_TEST;
	memset(s_In, 0, sizeof(s_Save));
	ok = ok && s_SdCard.ReadSect(first, s_In, NB_TEST) == NB_TEST;
	Check(ok && memcmp(s_In, s_Out, sizeof(s_Save)) == 0, "multiple sector write, read");
	ok = true;
	for (uint32_t i = 0; i < NB_TEST && ok; i++)
	{
		ok = s_SdCard.SectRead(first + i, s_In) && memcmp(s_In, s_Out + i * SECT_SIZE, SECT_SIZE) == 0;
	}
	Check(ok, "multiple sector write, single sector reads");

	// Buffers on an odd address
	Fill(s_Out + 1, 3 * SECT_SIZE, first + 2);
	ok = s_SdCard.WriteSect(first + 5, s_Out + 1, 3) == 3;
	memset(s_In, 0, sizeof(s_In));
	ok = ok && s_SdCard.ReadSect(first + 5, s_In + 3, 3) == 3;
	Check(ok && memcmp(s_In + 3, s_Out + 1, 3 * SECT_SIZE) == 0, "buffers not aligned");

	// The end of the card
	Check(s_SdCard.SectRead(nb - 1, s_In), "last sector read");
	Check(s_SdCard.ReadSect(nb - 2, s_In, 2) == 2, "last two sectors, one transfer");
	Check(s_SdCard.SectRead(nb, s_In) == false, "read past the end refused");

	// Content put back
	ok = s_SdCard.WriteSect(first, s_Save, NB_TEST) == NB_TEST;
	ok = ok && s_SdCard.ReadSect(first, s_In, NB_TEST) == NB_TEST;
	Check(ok && memcmp(s_In, s_Save, sizeof(s_Save)) == 0, "test sectors put back");

	printf("\r\n%s\r\n", s_Fail == 0 ? "ALL PASS" : "FAILURES");

	return s_Fail == 0 ? 0 : 1;
}
