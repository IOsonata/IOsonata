/**-------------------------------------------------------------------------
@example	i2c_master_demo.cpp

@brief	I2C master write/read verification with the generic I2C slave demo.

Connect SDA, SCL and ground to a second MCU running i2c_slave_demo.cpp.
Use external pull-ups on SDA and SCL. The slave uses address 0x22 and a
one-byte memory offset. This example writes and reads 16 bytes at offset 2.

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
#include <atomic>
#include <stdio.h>
#include <string.h>

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef I2C_MASTER_DMA_ENABLE
#define I2C_MASTER_DMA_ENABLE true
#endif

#ifndef I2C_MASTER_INT_ENABLE
#define I2C_MASTER_INT_ENABLE true
#endif

#ifndef I2C_MASTER_SCL_DIR
#define I2C_MASTER_SCL_DIR IOPINDIR_OUTPUT
#endif

#define FIFOSIZE		CFIFO_MEMSIZE(512)

uint8_t g_TxBuff[FIFOSIZE];

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	UART_DEVNO,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	UART_BAUDRATE,		// Rate
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
	g_TxBuff,
};

UART g_Uart;

//********** I2C Master **********
#define I2C_SCL_RATE	100000 // Rate in Hz, supported 100k, 250k, and 400k

int I2CMasterIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cMasterPins[] = {
	{I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, I2C_MASTER_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SDA
	{I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, I2C_MASTER_SCL_PINOP, I2C_MASTER_SCL_DIR, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
};

static const I2CCfg_t s_I2cCfgMaster = {
	.DevNo = I2C_MASTER_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cMasterPins,
	.NbIOPins = sizeof(s_I2cMasterPins) / sizeof(IOPinCfg_t),
	.Rate = I2C_SCL_RATE,		// Rate in Hz
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},		// Slave addresses
	.bDmaEn = I2C_MASTER_DMA_ENABLE,
	.bIntEn = I2C_MASTER_INT_ENABLE,
	.IntPrio = 7,			// Interrupt prio
	.EvtCB = I2CMasterIntrfHandler		// Event callback
};

I2C g_I2CMaster;

#define I2C_SLAVE_ADDR 0x22

static std::atomic<bool> s_Completed(false);
static std::atomic<int> s_Count(0);

int I2CMasterIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	if (EvtId == DEVINTRF_EVT_COMPLETED)
	{
		s_Count = Len;
		s_Completed = true;
	}
	return 0;
}

static int WaitTransfer(int Count)
{
	uint32_t timeout = 10000000U;

	if (Count >= 0)
	{
		return Count;
	}
	while (!s_Completed && --timeout > 0)
	{
		__NOP();
	}
	if (!s_Completed)
	{
		g_I2CMaster.Reset();
		return 0;
	}
	return s_Count;
}

int main()
{
	uint8_t offset = 2;
	uint8_t tx[16];
	uint8_t rx[16];
	int written;
	int received;
	int i;
	bool pass;

	if (!g_Uart.Init(s_UartCfg))
	{
		return 1;
	}
#if defined(NDEBUG) || defined(UART_RETARGET)
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	if (!g_I2CMaster.Init(s_I2cCfgMaster))
	{
		printf("I2C master init FAIL\r\n");
		return 1;
	}
	for (i = 0; i < (int)sizeof(tx); i++)
	{
		tx[i] = 0xa0 + i;
	}
	memset(rx, 0, sizeof(rx));
	s_Completed = false;
	written = WaitTransfer(g_I2CMaster.Write(I2C_SLAVE_ADDR, &offset, 1, tx, sizeof(tx)));
	s_Completed = false;
	received = WaitTransfer(g_I2CMaster.Read(I2C_SLAVE_ADDR, &offset, 1, rx, sizeof(rx)));
	pass = written == (int)sizeof(tx) && received == (int)sizeof(rx) && memcmp(tx, rx, sizeof(tx)) == 0;
	printf("I2C master write %d read %d: %s\r\n", written, received, pass ? "PASS" : "FAIL");
	while (1)
	{
		__WFE();
	}
}
