/**-------------------------------------------------------------------------
@example	nfc_tag_demo.cpp

@brief	Dynamic NFC tag on an I2C bus

Reads the NDEF message the tag holds, writes a text and a URI record, reads
them back, then checks the tag every 2 seconds. It prints the message when a
reader changed it and a line when a reader holds the tag. A phone held over
the antenna reads the records. board.h selects the tag by its name:

	M24SR		ST M24SR, NFC Forum Type 4

and gives the I2C bus with I2C_DEVNO and I2C_PINS. When the RF disable pin of
the tag is on the MCU, NFC_RFDIS_PORT, NFC_RFDIS_PIN and NFC_RFDIS_PINOP give
it and the demo holds it low, RF on. With UART_RETARGET and the UART pins, the
Release build prints on the UART at 1000000 baud 8N1, the Debug build uses
semihosting.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#include <string.h>

#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "coredev/i2c.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "stddev.h"
#include "rftag/m24sr.h"
#include "rftag/rftag_ndef.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// Period of the check for a message changed by a reader
#define NFC_DEMO_CHECK_MS		2000

// Largest message handled by the demo
#define NFC_DEMO_NDEF_MAX		512

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

static const IOPinCfg_t s_I2cPins[] = I2C_PINS;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCB = NULL,
};

static I2C s_I2c;

// The chip gives the memory size, the id and the NDEF format
static const RFTagCfg_t s_TagCfg = {
	.NfcId = { 0 },
	.IdLen = 0,
	.bReadOnly = false,
	.pMem = nullptr,
	.MemSize = 0,
	.NdefAddr = 0,
	.NdefMaxLen = 0,
	.NdefFmt = RFTAG_NDEF_FMT_NLEN16,
	.WrProtCB = nullptr,
	.pWrProtCtx = nullptr,
};

#if defined(M24SR)
static TagM24sr s_Tag;
static const char s_TagName[] = "M24SR";
#else
#error "board.h selects no NFC tag"
#endif

static uint8_t s_Msg[NFC_DEMO_NDEF_MAX];
static uint8_t s_Ndef[NFC_DEMO_NDEF_MAX];
static int s_NdefLen = 0;
static bool s_bHeld = false;

// Prints one record, the text of the Text and URI records
static bool PrintRecord(void *pCtx, uint8_t Tnf, const uint8_t *pType, size_t TypeLen,
						const uint8_t *pPayload, size_t PayloadLen)
{
	(void)pCtx;

	printf("  record TNF %u, type %.*s, %u bytes", (unsigned)Tnf, (int)TypeLen, (const char *)pType,
		   (unsigned)PayloadLen);

	if (Tnf == RFNDEF_TNF_WELL_KNOWN && TypeLen == 1 && PayloadLen > 0)
	{
		if (pType[0] == 'T')
		{
			// Status byte with the length of the language code, then the text
			size_t ll = pPayload[0] & 0x3F;

			if (1 + ll <= PayloadLen)
			{
				printf(": %.*s", (int)(PayloadLen - 1 - ll), (const char *)&pPayload[1 + ll]);
			}
		}
		else if (pType[0] == 'U')
		{
			// Prefix code, then the URI
			printf(": prefix %u, %.*s", (unsigned)pPayload[0], (int)(PayloadLen - 1), (const char *)&pPayload[1]);
		}
	}
	printf("\r\n");

	return true;
}

static void PrintNdef(const uint8_t *pNdef, int Len)
{
	printf("NDEF message: %d bytes\r\n", Len);
	if (Len > 0 && RFNdefParse(pNdef, Len, PrintRecord, nullptr) == false)
	{
		printf("  not a valid NDEF message\r\n");
	}
}

// Prints the message when a reader changed it, and when a reader holds the tag
static void CheckTag()
{
	// The tag refuses the I2C session while a reader holds it
	uint8_t hdr[2];

	if (s_Tag.MemRead(s_Tag.NdefAddr(), hdr, sizeof(hdr)) != (int)sizeof(hdr))
	{
		if (s_bHeld == false)
		{
			printf("Held by a reader\r\n");
			s_bHeld = true;
		}

		return;
	}
	s_bHeld = false;

	int nlen = (hdr[0] << 8) | hdr[1];
	int len = s_Tag.GetNdef(s_Msg, sizeof(s_Msg));

	if (nlen > 0 && len == 0)
	{
		// A reader took the tag between the two reads
		return;
	}

	if (len != s_NdefLen || memcmp(s_Msg, s_Ndef, len) != 0)
	{
		printf("Changed by a reader, ");
		PrintNdef(s_Msg, len);
		memcpy(s_Ndef, s_Msg, len);
		s_NdefLen = len;
	}
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

	printf("\r\nNFC tag, I2C %d\r\n", I2C_DEVNO);

#ifdef NFC_RFDIS_PORT
	// RF on
	IOPinConfig(NFC_RFDIS_PORT, NFC_RFDIS_PIN, NFC_RFDIS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinClear(NFC_RFDIS_PORT, NFC_RFDIS_PIN);
#endif

	if (s_I2c.Init(s_I2cCfg) == false)
	{
		printf("I2C init failed\r\n");

		return 1;
	}

	if (s_Tag.Init(s_TagCfg, &s_I2c) == false)
	{
		printf("%s: not found or held by a reader\r\n", s_TagName);

		return 1;
	}

	printf("%s at 0x%02lX: found, UID", s_TagName, (unsigned long)M24SR_I2C_DEV_ADDR);
	for (int i = 0; i < s_Tag.IdLen(); i++)
	{
		printf(" %02X", s_Tag.NfcId()[i]);
	}
	printf(", product code 0x%02X, NDEF file %lu bytes\r\n", s_Tag.ProductCode(), (unsigned long)s_Tag.MemSize());

	// What the tag holds now
	s_NdefLen = s_Tag.GetNdef(s_Ndef, sizeof(s_Ndef));
	PrintNdef(s_Ndef, s_NdefLen);

	RFNdefMsg_t msg;

	RFNdefInit(&msg, s_Msg, sizeof(s_Msg));
	RFNdefAddText(&msg, "en", "IOsonata NFC tag demo");
	RFNdefAddUri(&msg, "https://github.com/IOsonata/IOsonata");

	if (s_Tag.SetNdef(msg.pBuf, (uint16_t)msg.Len) == false)
	{
		printf("NDEF write failed, status 0x%04X\r\n", s_Tag.LastStatus());

		return 1;
	}

	s_NdefLen = s_Tag.GetNdef(s_Ndef, sizeof(s_Ndef));
	printf("NDEF written, %u bytes, read back %s\r\n", (unsigned)msg.Len,
		   s_NdefLen == (int)msg.Len && memcmp(s_Ndef, msg.pBuf, msg.Len) == 0 ? "the same" : "DIFFERENT");
	PrintNdef(s_Ndef, s_NdefLen);
	printf("Hold a phone over the antenna to read the tag\r\n");

	while (1)
	{
		msDelay(NFC_DEMO_CHECK_MS);
		CheckTag();
	}

	return 0;
}
