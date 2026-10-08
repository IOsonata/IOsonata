/**-------------------------------------------------------------------------
@example	spi_master_slave.cpp

@brief	SPI master/slave validation with hardware or software master.

Select SPISoft with SPI_MASTER_SOFTWARE in board.h when only one hardware
SPI controller is available. Hardware masters exercise TX and RX frames;
SPISoft also verifies both directions simultaneously.

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

#ifndef SPI_MASTER_SOFTWARE
#define SPI_MASTER_SOFTWARE false
#endif
#if SPI_MASTER_SOFTWARE
#include "coredev/spi_soft.h"
#endif
#ifndef SPI_MASTER_DMA_ENABLE
#define SPI_MASTER_DMA_ENABLE (!SPI_MASTER_SOFTWARE)
#endif
#ifndef SPI_MASTER_INT_ENABLE
#define SPI_MASTER_INT_ENABLE (!SPI_MASTER_SOFTWARE)
#endif
#ifndef SPI_MASTER_RATE
#define SPI_MASTER_RATE 1000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef SPI_SLAVE_DMA_ENABLE
#define SPI_SLAVE_DMA_ENABLE true
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
	{SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN, SPI_MASTER_SCK_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN, SPI_MASTER_MISO_PINOP,
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN, SPI_MASTER_MOSI_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN, SPI_MASTER_CS_PINOP,
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
void SpiMasterSlaveDiagnostics(void) __attribute__((weak));

static int SlaveEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
					 uint8_t *pBuffer, int Length)
{
	(void)pDev;
	(void)pBuffer;
	if (Event == DEVINTRF_EVT_STATECHG)
	{
		++s_Arms;
		g_SpiSlave.SetSlaveRxBuffer(0, s_SlaveRx,
			SPI_MASTER_SOFTWARE ? s_FrameLength : sizeof(s_SlaveRx));
		g_SpiSlave.SetSlaveTxData(0, s_SlaveTx,
			SPI_MASTER_SOFTWARE ? s_FrameLength : sizeof(s_SlaveTx));
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

static volatile bool s_MasterDone;
static volatile int s_MasterCount;
static int MasterEvent(DevIntrf_t * const, DEVINTRF_EVT Event, uint8_t *, int Length)
{
	if (Event == DEVINTRF_EVT_COMPLETED)
	{
		s_MasterCount = Length;
		s_MasterDone = true;
	}
	return 0;
}

static const SPICfg_t s_MasterCfg = {
	SPI_MASTER_DEVNO, SPIPHY_NORMAL, SPIMODE_MASTER,
	s_MasterPins, sizeof(s_MasterPins) / sizeof(IOPinCfg_t),
	SPI_MASTER_RATE, 8, 0, SPIDATABIT_MSB,
	SPIDATAPHASE_FIRST_CLK, SPICLKPOL_HIGH, SPICSEL_AUTO,
	SPI_MASTER_DMA_ENABLE, SPI_MASTER_INT_ENABLE, 6, 0xFF, MasterEvent
};
#if SPI_MASTER_SOFTWARE
SPISoft g_SpiMaster;
#else
SPI g_SpiMaster;
#endif

static bool Frame(int Length, uint8_t Seed, bool Receive = false)
{
	uint8_t masterRx[129] = {};
	s_Done = false;
	const unsigned before = s_Completions;
	const unsigned armsBefore = s_Arms;
	uint8_t masterTx[129];
	for (int i = 0; i < Length; ++i)
		masterTx[i] = static_cast<uint8_t>(Seed + i);
	s_MasterDone = false;
#if SPI_MASTER_SOFTWARE
	(void)Receive;
	int count = g_SpiMaster.Transfer(0, masterTx, masterRx, Length);
	const bool checkRx = true;
	const uint8_t slaveSeed = Seed;
	const char *label = "Full duplex";
#else
	int count = Receive ? g_SpiMaster.Rx(0, masterRx, Length) :
		g_SpiMaster.Tx(0, masterTx, Length);
	if (count < 0)
	{
		uint32_t timeout = 1000000U;
		while (!s_MasterDone && --timeout != 0U)
			__NOP();
		count = s_MasterDone ? s_MasterCount : 0;
	}
	const bool checkRx = Receive;
	const uint8_t slaveSeed = Receive ? s_MasterCfg.DummyByte : Seed;
	const char *label = Receive ? "Master RX" : "Master TX";
#endif
	uint32_t timeout = 1000000U;
	while (!s_Done && --timeout != 0U)
		__NOP();
	bool pass = count == Length && s_Done && s_Completions == before + 1 && s_Count == Length;
	int mismatch = -1;
	for (int i = 0; i < Length; ++i)
		if ((checkRx && masterRx[i] != s_SlaveTx[i]) ||
			s_SlaveRx[i] != (uint8_t)(slaveSeed + (SPI_MASTER_SOFTWARE || !Receive ? i : 0)))
		{
			pass = false;
			if (mismatch < 0)
				mismatch = i;
		}
	printf("%s %d bytes slave RX=%d %s\r\n",
		label, Length, s_Done ? s_Count : 0, pass ? "PASS" : "FAIL");
	if (!s_Done)
	{
		printf("Slave completion TIMEOUT\r\n");
		printf("CS master=%d slave=%d callbacks=%u->%u arms=%u->%u last RX=%d\r\n",
			IOPinRead(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN),
			IOPinRead(SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN),
			before, s_Completions, armsBefore, s_Arms, s_Count);
		if (SpiMasterSlaveDiagnostics != nullptr)
			SpiMasterSlaveDiagnostics();
	}
	if (mismatch >= 0)
		printf("Mismatch at %d: master RX=%02x expected=%02x slave RX=%02x expected=%02x\r\n",
			mismatch, masterRx[mismatch], s_SlaveTx[mismatch],
			s_SlaveRx[mismatch], (uint8_t)(slaveSeed + (SPI_MASTER_SOFTWARE || !Receive ? mismatch : 0)));
	return pass;
}

int main()
{
	g_Uart.Init(s_UartCfg);
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	printf("Init SPI Master/Slave demo\r\n");
	printf("SPI master %s DMA=%d INT=%d\r\n",
		SPI_MASTER_SOFTWARE ? "SPISoft" : "hardware",
		s_MasterCfg.bDmaEn, s_MasterCfg.bIntEn);
	printf("SPI slave requested DMA=%d INT=%d\r\n",
		s_SlaveCfg.bDmaEn, s_SlaveCfg.bIntEn);
#ifdef SPI_LOOPBACK_WIRING
	printf("%s\r\n", SPI_LOOPBACK_WIRING);
#endif
	bool pass = true;
	const int lengths[] = {1, 16, 129};
	for (int mode = 0; mode < 4; ++mode)
	{

		SPICfg_t cfg = s_SlaveCfg;
		cfg.ClkPol = (mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
		cfg.DataPhase = (mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
		SPICfg_t masterCfg = s_MasterCfg;
		masterCfg.ClkPol = cfg.ClkPol;
		masterCfg.DataPhase = cfg.DataPhase;
		if (!g_SpiMaster.Init(masterCfg))
		{
			printf("SPI master init FAIL\r\n");
			pass = false;
			break;
		}
		s_FrameLength = sizeof(s_SlaveRx);
		for (unsigned i = 0; i < sizeof(s_SlaveTx); ++i)
			s_SlaveTx[i] = static_cast<uint8_t>(0xA0 ^ i);
		if (!g_SpiSlave.Init(cfg))
		{
			printf("SPI slave init FAIL\r\n");
			pass = false;
			break;
		}
		if (mode == 0)
		{
			const DevIntrf_t *intrf = static_cast<DevIntrf_t *>(g_SpiSlave);
			printf("SPI slave effective DMA=%d INT=%d\r\n", intrf->bDma, intrf->bIntEn);
		}
		printf("SPI mode=%d bits=8 rate=%lu Hz%s\r\n", mode,
			(unsigned long)g_SpiMaster.Rate(), SPI_MASTER_SOFTWARE ? " (nominal software rate)" : "");
		for (unsigned n = 0; n < sizeof(lengths) / sizeof(lengths[0]); ++n)
		{
			s_FrameLength = lengths[n];
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
#if SPI_MASTER_SOFTWARE
			// Apply this frame capacity while CS is inactive. Hardware-master
			// tests keep a fixed capacity for ports whose Reset is a no-op.
			g_SpiSlave.Reset();
#endif
			if (!Frame(s_FrameLength, 0x30))
			{
				pass = false;
				break;
			}
			// No reset: NSS rising must rearm byte zero for the next frame.
			memset(s_SlaveRx, 0, sizeof(s_SlaveRx));
			if (!Frame(s_FrameLength, 0x50))
			{
				pass = false;
				break;
			}
#if !SPI_MASTER_SOFTWARE
			if (!Frame(s_FrameLength, 0, true))
			{
				pass = false;
				break;
			}
#endif
			if (s_FrameLength > 1)
			{
				// A short CS frame must discard the unused TX prefetch.
				if (!Frame(1, 0x70) || !Frame(s_FrameLength, 0x90))
				{
					pass = false;
					break;
				}
			}
		}
		if (!pass)
			break;
	}
	printf("SPI master/slave loopback %s\r\n", pass ? "PASS" : "FAIL");
	while (1)
		__WFE();
}

