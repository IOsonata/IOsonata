// SPDX-License-Identifier: MIT
// Polling register read through the generic I2C and DeviceIntrf APIs.
// The peer at 0x22 must accept a one-byte register offset before a read.

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "board.h"

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

static const IOPinCfg_t s_I2cPins[] = {
	{I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, I2C_MASTER_SDA_PINOP,
	 IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	{I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, I2C_MASTER_SCL_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_MASTER_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCB = NULL,
};

static UART s_Uart;
static I2C s_I2c;

int main()
{
	if (!s_Uart.Init(s_UartCfg))
	{
		while (1) __WFE();
	}
	if (!s_I2c.Init(s_I2cCfg))
	{
		s_Uart.printf("I2C initialization failed\r\n");
		while (1) __WFE();
	}

	// Read() sends the offset, repeated START, data, then STOP.
	const int peer = 0x22;
	uint8_t offset = 3;
	uint8_t data[5] = {0,};
	int count = s_I2c.Read(peer, &offset, sizeof(offset), data, sizeof(data));
	s_Uart.printf("I2C %lu Hz: read %d of %u bytes\r\n",
		(unsigned long)s_I2c.Rate(), count, (unsigned)sizeof(data));
	if (count == (int)sizeof(data))
	{
		for (unsigned i = 0; i < sizeof(data); i++)
			s_Uart.printf("%02x ", data[i]);
		s_Uart.printf("\r\n");
	}
	else
	{
		s_Uart.printf("Incomplete read: check the peer, wiring and pull-ups\r\n");
	}

	while (1) __WFE();
}
