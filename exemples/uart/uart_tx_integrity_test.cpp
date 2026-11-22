/**-------------------------------------------------------------------------
@example uart_tx_integrity_test.cpp
@brief Deterministic UART TX integrity test using the production UART driver.

Sends exactly UART_TX_TEST_FRAMES independent 64-byte binary frames with a
sequence number, deterministic payload and CRC-16/CCITT-FALSE. No SysLog or
printf is used; the host checker owns all statistics.

MIT License. Copyright (c) 2026, I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "board.h"

#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif

#ifndef UART_TX_TEST_FRAMES
#define UART_TX_TEST_FRAMES 2048U
#endif

#define UART_TX_FRAME_SIZE 64U
#define UART_TX_CRC_OFFSET 62U

static const IOPinCfg_t s_UartPins[] = {
	{ UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL },
	{ UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCB = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static UART s_Uart;
static uint8_t s_Frame[UART_TX_FRAME_SIZE];

static uint16_t UartTxTestCrc(const uint8_t *pData, uint32_t Len)
{
	uint16_t crc;
	uint32_t i;
	uint32_t bit;

	crc = 0xffffU;
	for (i = 0U; i < Len; i++)
	{
		crc ^= (uint16_t)pData[i] << 8;
		for (bit = 0U; bit < 8U; bit++)
		{
			crc = (crc & 0x8000U) != 0U ?
				(uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static uint32_t UartTxTestNext(uint32_t Value)
{
	Value ^= Value << 13;
	Value ^= Value >> 17;
	Value ^= Value << 5;
	return Value;
}

static void UartTxTestFrameBuild(uint32_t Sequence)
{
	uint16_t crc;
	uint32_t state;
	uint32_t i;

	s_Frame[0] = 'I';
	s_Frame[1] = 'O';
	s_Frame[2] = 'T';
	s_Frame[3] = 'X';
	s_Frame[4] = (uint8_t)Sequence;
	s_Frame[5] = (uint8_t)(Sequence >> 8);
	s_Frame[6] = (uint8_t)(Sequence >> 16);
	s_Frame[7] = (uint8_t)(Sequence >> 24);
	state = Sequence ^ 0xA5C3D29FUL;
	for (i = 8U; i < UART_TX_CRC_OFFSET; i++)
	{
		state = UartTxTestNext(state);
		s_Frame[i] = (uint8_t)state;
	}
	crc = UartTxTestCrc(s_Frame, UART_TX_CRC_OFFSET);
	s_Frame[62] = (uint8_t)crc;
	s_Frame[63] = (uint8_t)(crc >> 8);
}

int main(void)
{
	uint32_t seq;
	uint32_t offset;
	int count;

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}

	// Open the host serial port before resetting the board.
	// Deliberately send nothing except framed test bytes.
	msDelay(1500);
	for (seq = 0U; seq < UART_TX_TEST_FRAMES; seq++)
	{
		UartTxTestFrameBuild(seq);
		offset = 0U;
		while (offset < UART_TX_FRAME_SIZE)
		{
			count = s_Uart.Tx(s_Frame + offset, UART_TX_FRAME_SIZE - offset);
			if (count > 0)
			{
				offset += (uint32_t)count;
			}
			else
			{
				// Cooperative retry: no private UART FIFO access or forced
				// DMA completion. Interrupt mode remains enabled.
				msDelay(1);
			}
		}
	}

	// Leave the hardware running so the last queued DMA block can finish.
	while (1)
	{
		__WFI();
	}
}
