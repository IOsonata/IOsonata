/**-------------------------------------------------------------------------
@example	pdm_mic_demo.cpp

@brief	PDM microphone demo.

Captures PDM microphone samples into a local buffer and prints the first few
of each block to the UART. The PDM driver owns the sample buffers through its
CFifo, the event handler only drains them.

@author	Hoang Nguyen Hoan
@date	Dev. 9, 2020

@license

Copyright (c) 2020, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "coredev/pdm.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"

// This include contain i/o definition the board in use
#include "board.h"

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void PdmHandler(PdmDev_t *pDev, DEVINTRF_EVT Evt);

#define FIFOSIZE			CFIFO_MEMSIZE(10*1024)

uint8_t g_TxBuff[FIFOSIZE];

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// Interrupt priority
	nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

// Microphone pins. The driver expects clock first, data second, indexed by
// PDM_CLKPIN_IDX and PDM_DINPIN_IDX.
static const IOPinCfg_t s_PdmPins[] = {
	{PDM_CLK_PORT, PDM_CLK_PIN, PDM_CLK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{PDM_DIN_PORT, PDM_DIN_PIN, PDM_DIN_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

// Block size in octets. The driver hands one block back per RX_DATA event.
#define PDM_FIFO_BLKSIZE		128
#define PDM_FIFO_MEMSIZE		CFIFO_TOTAL_MEMSIZE(4, PDM_FIFO_BLKSIZE)

alignas(4) static uint8_t s_PdmFifoMem[PDM_FIFO_MEMSIZE];

static const PdmCfg_t s_PdmCfg = {
	.pPins = s_PdmPins,
	.NbPins = sizeof(s_PdmPins) / sizeof(IOPinCfg_t),
	.Freq = 1032000,
	.SmplMode = PDM_SMPLMODE_FALLING,
	.OpMode = PDM_OPMODE_MONO_LEFT,
	.GainLeft = 0,
	.GainRight = 0,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtHandler = PdmHandler,
	.pFifoMem = s_PdmFifoMem,
	.FifoMemSize = PDM_FIFO_MEMSIZE,
	.FifoBlkSize = PDM_FIFO_BLKSIZE,
};

PdmDev_t g_PdmDev;

// Capture buffer. Filled a block at a time from the event handler, printed
// from the main loop once full.
#define CAPTURE_SIZE			(10 * 1024)

alignas(4) uint8_t g_CaptureBuff[CAPTURE_SIZE];
volatile int g_CaptureIdx = 0;
volatile bool g_bCaptureFull = false;

void PdmHandler(PdmDev_t *pDev, DEVINTRF_EVT Evt)
{
	if (Evt != DEVINTRF_EVT_RX_DATA)
	{
		return;
	}

	uint16_t *p = PdmGetSamples(pDev);

	if (p == NULL || g_bCaptureFull)
	{
		return;
	}

	int len = PDM_FIFO_BLKSIZE;

	if (g_CaptureIdx + len > CAPTURE_SIZE)
	{
		len = CAPTURE_SIZE - g_CaptureIdx;
	}

	memcpy(&g_CaptureBuff[g_CaptureIdx], p, len);
	g_CaptureIdx += len;

	if (g_CaptureIdx >= CAPTURE_SIZE)
	{
		g_CaptureIdx = 0;
		g_bCaptureFull = true;
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);

	// Retarget printf to the UART, the capture dump below uses it
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	PdmInit(&g_PdmDev, &s_PdmCfg);
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	HardwareInit();

	PdmStart(&g_PdmDev);

	while (1)
	{
		__WFE();

		if (g_bCaptureFull)
		{
			uint16_t *p = (uint16_t *)g_CaptureBuff;

			for (int i = 0; i < 16; i++)
			{
				printf("%04x ", p[i]);
			}
			printf("\n");

			g_bCaptureFull = false;
		}
	}

	return 0;
}
