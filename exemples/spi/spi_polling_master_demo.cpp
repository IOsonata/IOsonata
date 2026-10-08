// SPDX-License-Identifier: MIT
// Polling transfers through the generic SPI and DeviceIntrf APIs.
// The peer must accept a one-byte command before returning 16 bytes.

#include "coredev/spi.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "board.h"

#ifndef SPI_MASTER_SOFTWARE
#define SPI_MASTER_SOFTWARE false
#endif
#if SPI_MASTER_SOFTWARE
#include "coredev/spi_soft.h"
#endif
#ifndef SPI_MASTER_RATE
#define SPI_MASTER_RATE 1000000
#endif

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static const IOPinCfg_t s_SpiPins[] = {
	{SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN,
	 SPI_MASTER_SOFTWARE ? IOPINOP_GPIO : SPI_MASTER_SCK_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN,
	 SPI_MASTER_SOFTWARE ? IOPINOP_GPIO : SPI_MASTER_MISO_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN,
	 SPI_MASTER_SOFTWARE ? IOPINOP_GPIO : SPI_MASTER_MOSI_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN,
	 SPI_MASTER_SOFTWARE ? IOPINOP_GPIO : SPI_MASTER_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_MASTER_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = SPI_MASTER_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH, // CPOL=0, idle low; mode 0.
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.DummyByte = 0xff,
	.EvtCB = NULL,
};

static UART s_Uart;
#if SPI_MASTER_SOFTWARE
static SPISoft s_Spi;
#else
static SPI s_Spi;
#endif

int main()
{
	if (!s_Uart.Init(s_UartCfg))
	{
		while (1) __WFE();
	}
	if (!s_Spi.Init(s_SpiCfg))
	{
		s_Uart.printf("SPI initialization failed\r\n");
		while (1) __WFE();
	}

	s_Uart.printf("SPI master %s DMA=0 INT=0%s\r\n",
		SPI_MASTER_SOFTWARE ? "SPISoft" : "hardware",
		SPI_MASTER_SOFTWARE ? " (nominal software rate)" : "");

	uint8_t command = 0;
	uint8_t data[16] = {0,};
	// Automatic CS remains low across the command and all receive clocks.
	int count = s_Spi.Read(0, &command, sizeof(command), data, sizeof(data));
	s_Uart.printf("SPI %lu Hz: read %d of %u bytes\r\n",
		(unsigned long)s_Spi.Rate(), count, (unsigned)sizeof(data));
	if (count == (int)sizeof(data))
	{
		for (unsigned i = 0; i < sizeof(data); i++)
			s_Uart.printf("%02x ", data[i]);
		s_Uart.printf("\r\n");
	}
	else
	{
		s_Uart.printf("Incomplete read\r\n");
	}

	for (unsigned i = 0; i < sizeof(data); i++) data[i] = i;
	count = s_Spi.Tx(0, data, sizeof(data));
	s_Uart.printf("SPI: transmitted %d of %u bytes\r\n", count, (unsigned)sizeof(data));
	while (1) __WFE();
}
