/**-------------------------------------------------------------------------
@example	spi_loopback.cpp

@brief	External SPI master MOSI-to-MISO polling loopback.

Connect MOSI to MISO as documented in the target board.h. RX clocks the
configured DummyByte onto MOSI; the jumper must return it unchanged on MISO.

@author	Hoang Nguyen Hoan
@date	July 21, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include "coredev/uart.h"
#include "coredev/spi.h"
#include "stddev.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef SPI_MASTER_DMA_ENABLE
#define SPI_MASTER_DMA_ENABLE false
#endif
#ifndef SPI_MASTER_INT_ENABLE
#define SPI_MASTER_INT_ENABLE false
#endif

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_UarTxBuff[FIFOSIZE];

// Assign UART pins
static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	UART_DEVNO,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	NULL,//nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_UarTxBuff,
};

UART g_Uart;

//********** SPI Master **********
static const IOPinCfg_t s_SpiMasterPins[] = {
	{SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN, SPI_MASTER_SCK_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// SCK
	{SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN, SPI_MASTER_MISO_PINOP,
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// MISO
	{SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN, SPI_MASTER_MOSI_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
	{SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN, SPI_MASTER_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// CS
};

static const SPICfg_t s_SpiMasterCfg = {
	SPI_MASTER_DEVNO,
	SPIPHY_NORMAL,
	SPIMODE_MASTER,
	s_SpiMasterPins,
	sizeof(s_SpiMasterPins) / sizeof(IOPinCfg_t),
	1000000,   // Speed in Hz
	8,      // Data Size
	5,      // Max retries
	SPIDATABIT_MSB,
	SPIDATAPHASE_SECOND_CLK, // Data phase
	SPICLKPOL_LOW,         // clock polarity
	SPICSEL_AUTO,
	SPI_MASTER_DMA_ENABLE,	// DMA
	SPI_MASTER_INT_ENABLE,
	6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
	0xff,
	NULL
};

SPI g_SpiMaster;


void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	printf("Init SPI master loopback demo\r\n");
}

int main()
{
	HardwareInit();
	printf("SPI master DMA=%d INT=%d\r\n",
		s_SpiMasterCfg.bDmaEn, s_SpiMasterCfg.bIntEn);
#ifdef SPI_LOOPBACK_WIRING
	printf("%s\r\n", SPI_LOOPBACK_WIRING);
#endif
	// This first loopback test exercises the synchronous polling path.
	if (s_SpiMasterCfg.bDmaEn || s_SpiMasterCfg.bIntEn)
	{
		printf("SPI loopback requires DMA=0 INT=0\r\n");
		while (1)
			__WFE();
	}

	const uint8_t patterns[] = { 0x00, 0xFF, 0x55, 0xAA, 0xA5, 0x3C };
	uint8_t rx[16];
	bool pass = true;
	for (int mode = 0; mode < 4; ++mode)
	{
		for (unsigned p = 0; p < sizeof(patterns); ++p)
		{
			SPICfg_t cfg = s_SpiMasterCfg;
			cfg.ClkPol = (mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
			cfg.DataPhase = (mode & 1) ?
				SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
			cfg.DummyByte = patterns[p];
			if (!g_SpiMaster.Init(cfg))
			{
				printf("SPI init FAIL mode=%d\r\n", mode);
				pass = false;
				break;
			}
			if (p == 0)
				printf("SPI mode=%d bits=8 rate=%lu Hz\r\n",
					mode, (unsigned long)g_SpiMaster.Rate());

			// Fill with the inverse so a missing receive write cannot pass.
			memset(rx, (uint8_t)~patterns[p], sizeof(rx));
			const int count = g_SpiMaster.Rx(0, rx, sizeof(rx));
			bool match = count == (int)sizeof(rx);
			for (unsigned i = 0; i < sizeof(rx); ++i)
				if (rx[i] != patterns[p])
					match = false;

			printf("RX %d/%d expected=%02x: ", count, (int)sizeof(rx), patterns[p]);
			for (unsigned i = 0; i < sizeof(rx); ++i)
				printf("%02x ", rx[i]);
			printf("%s\r\n", match ? "PASS" : "FAIL");
			pass = pass && match;
		}
	}
	printf("SPI master loopback %s\r\n", pass ? "PASS" : "FAIL");
	while (1)
		__WFE();
	return 0;
}
