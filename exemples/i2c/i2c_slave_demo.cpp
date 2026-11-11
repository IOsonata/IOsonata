/**-------------------------------------------------------------------------
@example	i2c_slave_demo.cpp

@brief	I2C slave memory with a one-byte address offset.

Connect an external master to the SDA, SCL and ground pins.
Provide pull-ups to the I/O supply on SDA and SCL.

This example demonstrate the read/write to the slave device memory. The
read/write command starts with a 1 byte offset location of the device memory.

For example :

- Reading 10 bytes at offset 3 is done by issuing a write of 1 byte value 3 then
  follow by a read of 10 byte.

    uint8_t offset = 3;
    uint8_t buff[10];
    g_I2C.Read(SLAVE_I2C_DEV_ADDR, &offset, 1, buff, 10);


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

#include "idelay.h"
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

#ifndef I2C_SLAVE_DMA_ENABLE
#define I2C_SLAVE_DMA_ENABLE true
#endif

#ifndef I2C_SLAVE_SCL_DIR
#define I2C_SLAVE_SCL_DIR IOPINDIR_OUTPUT
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


//********** I2C Slave **********

#define I2C_SLAVE_ADDR			0x22

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cSlavePins[] = {
	{I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},		// SDA
	{I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN, I2C_SLAVE_SCL_PINOP, I2C_SLAVE_SCL_DIR, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},	// SCL
};

static const I2CCfg_t s_I2cCfgSlave = {
	.DevNo = I2C_SLAVE_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,			// I2C type standard or SMBus
	.Mode = I2CMODE_SLAVE,				// Master/Slave mode
	.pIOPinMap = s_I2cSlavePins,		// I/O pins used by I2C
	.NbIOPins = sizeof(s_I2cSlavePins) / sizeof(IOPinCfg_t), // Number of IO pins mapped
	.Rate = I2C_SCL_RATE,				// Rate in Hz
	.MaxRetry = 5,						// Max number of retry
	.AddrType = I2CADDR_TYPE_NORMAL,	// I2C address type normal 7bits or extended 10bits
	.NbSlaveAddr = 1,					// Number of slave addresses
	.SlaveAddr = {I2C_SLAVE_ADDR,},// + 1,I2C_SLAVE_ADDR},		// Slave addresses
	.bDmaEn = I2C_SLAVE_DMA_ENABLE,						// DMA mode enable
	.bIntEn = true,						// Interrupt enable
	.IntPrio = 7,						// Interrupt priority
	.EvtCB = I2CSlaveIntrfHandler		// Event callback
};

I2C g_I2CSlave;

#define I2C_BUFF_SIZE 20
static uint8_t s_ReadRqstData[I2C_BUFF_SIZE];
static uint8_t s_WriteRqstData[I2C_BUFF_SIZE + 1];
static bool s_bWriteRqst;
static unsigned s_Offset;
static volatile unsigned s_Completions;
static volatile unsigned s_Errors;

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	int count;

	(void)pDev;
	(void)pBuffer;
	switch (EvtId)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bWriteRqst && Len > 0)
			{
				s_Offset = s_WriteRqstData[0] % I2C_BUFF_SIZE;
			}
			s_bWriteRqst = false;
			g_I2CSlave.SetReadRqstData(0, &s_ReadRqstData[s_Offset], I2C_BUFF_SIZE - s_Offset);
			break;

		case DEVINTRF_EVT_WRITE_RQST:
			s_bWriteRqst = true;
			g_I2CSlave.SetWriteRqstBuffer(0, s_WriteRqstData, sizeof(s_WriteRqstData));
			break;

		case DEVINTRF_EVT_COMPLETED:
			if (Len < 0 || (s_bWriteRqst && Len > (int)sizeof(s_WriteRqstData)))
			{
				++s_Errors;
			}
			else if (s_bWriteRqst && Len > 0)
			{
				s_Offset = s_WriteRqstData[0] % I2C_BUFF_SIZE;
				count = Len - 1;
				if (count > (int)(I2C_BUFF_SIZE - s_Offset))
				{
					count = I2C_BUFF_SIZE - s_Offset;
				}
				memcpy(&s_ReadRqstData[s_Offset], &s_WriteRqstData[1], count);
			}
			else if (!s_bWriteRqst)
			{
				s_Offset = (s_Offset + Len) % I2C_BUFF_SIZE;
			}
			s_bWriteRqst = false;
			++s_Completions;
			break;

		default:
			break;
	}
	return 0;
}

int main()
{
	unsigned previous;
	int i;

	g_Uart.Init(s_UartCfg);
#if defined(NDEBUG) || defined(UART_RETARGET)
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
	for (i = 0; i < I2C_BUFF_SIZE; i++)
	{
		s_ReadRqstData[i] = i;
	}
	if (!g_I2CSlave.Init(s_I2cCfgSlave))
	{
		printf("I2C slave init failed\r\n");
		while (1)
		{
			__WFE();
		}
	}
	printf("I2C slave ready: address 0x22, SDA GPIO%d, SCL GPIO%d\r\n",
		I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN);
	previous = 0;
	while (1)
	{
		if (previous != s_Completions)
		{
			previous = s_Completions;
			printf("I2C slave: completed %u, errors %u\r\n", previous, s_Errors);
		}
		__WFE();
	}
}
