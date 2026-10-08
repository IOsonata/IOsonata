/**-------------------------------------------------------------------------
@example	spi_slave_loopback.cpp

@brief	GPIO master to hardware SPI slave loopback.

Wire the GPIO master to the hardware slave as specified by board.h.
GPIO clocking intentionally leaves interrupt service time between characters.
This validates slave framing and data, not maximum external clock rate.

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
#include "iopinctrl.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef SPI_SLAVE_DMA_ENABLE
#define SPI_SLAVE_DMA_ENABLE false
#endif
#ifndef SPI_SLAVE_INT_ENABLE
#define SPI_SLAVE_INT_ENABLE true
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



static const IOPinCfg_t s_MasterPins[] = {
	{SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN, IOPINOP_GPIO,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN, IOPINOP_GPIO,
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN, IOPINOP_GPIO,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN, IOPINOP_GPIO,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
static const IOPinCfg_t s_SlavePins[] = {
	{SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN, SPI_SLAVE_SCK_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN, SPI_SLAVE_MISO_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN, SPI_SLAVE_MOSI_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN, SPI_SLAVE_CS_PINOP,
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};
static uint8_t s_SlaveTx[129];
static uint8_t s_SlaveRx[129];
static int s_FrameLength;
static volatile bool s_Done;
static volatile int s_Count;
static volatile unsigned s_Completions;
static volatile unsigned s_Arms;
SPI g_SpiSlave;

// Optional target-test diagnostic implementation, outside board configuration.
void SpiSlaveLoopbackDiagnostics(void) __attribute__((weak));

static int SlaveEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
					 uint8_t *pBuffer, int Length)
{
	(void)pDev;
	(void)pBuffer;
	if (Event == DEVINTRF_EVT_STATECHG)
	{
		++s_Arms;
		g_SpiSlave.SetSlaveRxBuffer(0, s_SlaveRx, s_FrameLength);
		g_SpiSlave.SetSlaveTxData(0, s_SlaveTx, s_FrameLength);
	}
	else if (Event == DEVINTRF_EVT_COMPLETED)
	{
		s_Count = Length;
		++s_Completions;
		s_Done = true;
	}
	return 0;
}

static const SPICfg_t s_SlaveCfg = {
	SPI_SLAVE_DEVNO, SPIPHY_NORMAL, SPIMODE_SLAVE,
	s_SlavePins, sizeof(s_SlavePins) / sizeof(IOPinCfg_t),
	100000, 8, 0, SPIDATABIT_MSB,
	SPIDATAPHASE_FIRST_CLK, SPICLKPOL_HIGH, SPICSEL_AUTO,
	SPI_SLAVE_DMA_ENABLE, SPI_SLAVE_INT_ENABLE, 6, 0xFF, SlaveEvent
};

static void Delay()
{
	for (volatile unsigned i = 0; i < 100; ++i)
		__NOP();
}

static void Clock(bool High)
{
	if (High)
		IOPinSet(SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN);
	else
		IOPinClear(SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN);
}

static void Mosi(bool High)
{
	if (High)
		IOPinSet(SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN);
	else
		IOPinClear(SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN);
}

static uint8_t TransferByte(int Mode, uint8_t Tx)
{
	const bool idle = (Mode & 2) != 0;
	const bool phase = (Mode & 1) != 0;
	uint8_t rx = 0;
	for (int bit = 7; bit >= 0; --bit)
	{
		if (phase)
			Clock(!idle);
		Mosi((Tx & (1U << bit)) != 0U);
		Delay();
		// Sample adjacent to the edge, before the slave ISR can preload the
		// next character. A hardware master samples at the edge itself.
		const uint32_t state = DisableInterrupt();
		Clock(phase ? idle : !idle);
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		rx = (uint8_t)((rx << 1U) |
			(IOPinRead(SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN) != 0));
		EnableInterrupt(state);
		Delay();
		if (!phase)
			Clock(idle);
		Delay();
	}
	return rx;
}

static bool Frame(int Mode, int Length, uint8_t Seed)
{
	uint8_t masterRx[129];
	s_Done = false;
	const unsigned before = s_Completions;
	const unsigned armsBefore = s_Arms;
	IOPinClear(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
	Delay();
	for (int i = 0; i < Length; ++i)
		masterRx[i] = TransferByte(Mode, (uint8_t)(Seed + i));
	Delay();
	IOPinSet(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
	uint32_t timeout = 1000000U;
	while (!s_Done && --timeout != 0U)
		__NOP();
	bool pass = s_Done && s_Completions == before + 1 && s_Count == Length;
	int mismatch = -1;
	for (int i = 0; i < Length; ++i)
		if (masterRx[i] != s_SlaveTx[i] || s_SlaveRx[i] != (uint8_t)(Seed + i))
		{
			pass = false;
			if (mismatch < 0)
				mismatch = i;
		}
	printf("Full duplex %d bytes slave RX=%d %s\r\n",
		Length, s_Done ? s_Count : 0, pass ? "PASS" : "FAIL");
	if (!s_Done)
	{
		printf("Slave completion TIMEOUT\r\n");
		printf("CS master=%d slave=%d callbacks=%u->%u arms=%u->%u last RX=%d\r\n",
			IOPinRead(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN),
			IOPinRead(SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN),
			before, s_Completions, armsBefore, s_Arms, s_Count);
		if (SpiSlaveLoopbackDiagnostics != nullptr)
			SpiSlaveLoopbackDiagnostics();
	}
	if (mismatch >= 0)
		printf("Mismatch at %d: master RX=%02x expected=%02x slave RX=%02x expected=%02x\r\n",
			mismatch, masterRx[mismatch], s_SlaveTx[mismatch],
			s_SlaveRx[mismatch], (uint8_t)(Seed + mismatch));
	return pass;
}

int main()
{
	g_Uart.Init(s_UartCfg);
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	printf("Init SPI slave loopback demo\r\n");
	printf("SPI master GPIO DMA=0 INT=0 (software clock)\r\n");
	printf("SPI slave DMA=%d INT=%d\r\n",
		s_SlaveCfg.bDmaEn, s_SlaveCfg.bIntEn);
	printf("%s\r\n", SPI_LOOPBACK_WIRING);
	IOPinSet(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN);
	Clock(false);
	Mosi(false);
	IOPinCfg(s_MasterPins, sizeof(s_MasterPins) / sizeof(IOPinCfg_t));
	bool pass = true;
	const int lengths[] = {1, 16, 129};
	for (int mode = 0; mode < 4; ++mode)
	{
		Clock((mode & 2) != 0);
		SPICfg_t cfg = s_SlaveCfg;
		cfg.ClkPol = (mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
		cfg.DataPhase = (mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
		s_FrameLength = sizeof(s_SlaveRx);
		memset(s_SlaveTx, 0, sizeof(s_SlaveTx));
		if (!g_SpiSlave.Init(cfg))
		{
			printf("SPI slave init FAIL\r\n");
			pass = false;
			break;
		}
		printf("SPI mode=%d bits=8 software-paced clock\r\n", mode);
		for (unsigned n = 0; n < sizeof(lengths) / sizeof(lengths[0]); ++n)
		{
			s_FrameLength = lengths[n];
			for (int i = 0; i < s_FrameLength; ++i)
				s_SlaveTx[i] = (uint8_t)(0xA0 ^ i);
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			// Reload the preloaded first byte after changing foreground data.
			g_SpiSlave.Reset();
			if (!Frame(mode, s_FrameLength, 0x30))
			{
				pass = false;
				break;
			}
			// No reset: NSS rising must rearm byte zero for the next frame.
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			if (!Frame(mode, s_FrameLength, 0x50))
			{
				pass = false;
				break;
			}
			if (s_FrameLength > 1)
			{
				// A short CS frame must discard the unused TX prefetch.
				if (!Frame(mode, 1, 0x70) || !Frame(mode, s_FrameLength, 0x90))
				{
					pass = false;
					break;
				}
			}
		}
		if (!pass)
			break;
	}
	printf("SPI slave loopback %s\r\n", pass ? "PASS" : "FAIL");
	while (1)
		__WFE();
}

