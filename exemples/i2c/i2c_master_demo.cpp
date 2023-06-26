/**-------------------------------------------------------------------------
@example	i2c_master_demo.cpp

@brief	This example demonstrate the use of I2C in both master and slave mode

Two I2C devices are created, one in master mode and the other in slave mode.
User is required to connect the wire to the appropriate pins.

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
#include <atomic>

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

//int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

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
	115200,			// Rate
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
	{I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, I2C_MASTER_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
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
	.bDmaEn = true,
	.bIntEn = true,
	.IntPrio = 7,			// Interrupt prio
	.EvtCB = I2CMasterIntrfHandler		// Event callback
};

I2C g_I2CMaster;

//********** I2C Slave **********

#define I2C_SLAVE_ADDR			0x22

#define I2C_BUFF_SIZE	20
#define I2C_BUFF_SIZE	20
uint8_t s_ReadRqstData[I2C_BUFF_SIZE];
uint8_t s_WriteRqstData[I2C_BUFF_SIZE];
bool s_bWriteRqst = false;
int s_Offset = 0;
std::atomic<bool> g_bCompleted(false);
std::atomic<int> g_RxCnt(0);

int I2CMasterIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_TX_READY:
			//g_bCompleted = true;
			break;
		case DEVINTRF_EVT_COMPLETED:
			g_RxCnt = Len;
			printf("DEVINTRF_EVT_COMPLETED %d\r\n", (int)g_RxCnt);
			g_bCompleted = true;
			break;
	}

	return 0;
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif

	g_Uart.printf("Init I2C Master demo\r\n");
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
	uint8_t offset;
	uint8_t buff[I2C_BUFF_SIZE];

	HardwareInit();

	g_I2CMaster.Init(s_I2cCfgMaster);


	// Fill dummy data for debugging and validation
	for (int i = 0; i < I2C_BUFF_SIZE; i++)
	{
		s_ReadRqstData[i] = i;
	}

	memset(s_WriteRqstData, 0, I2C_BUFF_SIZE);
	memset(buff, 0xFF, I2C_BUFF_SIZE);

	// Fill buff with data
	buff[0] = 0xa0;
	buff[1] = 0xa1;
	buff[2] = 0xa2;
	buff[3] = 0xa3;
	buff[4] = 0xa4;
	buff[5] = 0xa5;
	buff[6] = 0xa6;
	buff[7] = 0xa7;

	buff[8] = 0xb0;
	buff[9] = 0xb1;
	buff[10] = 0xb2;
	buff[11] = 0xb3;
	buff[12] = 0xb4;
	buff[13] = 0xb5;
	buff[14] = 0xb6;
	buff[15] = 0xb7;


	uint16_t i2cDevAddr = (buff[1] << 8) | buff[0];
	uint8_t x = i2cDevAddr;

	offset = 0; // want to read/write from offset position
	uint8_t nBytes = 11;
	int c = 0;

#if 1
//	int c = g_I2CMaster.Write(I2C_SLAVE_ADDR, &offset, 1, buff, nBytes);
	//printf("Write %d bytes at offset %d\r\n", c, offset);
	c = g_I2CMaster.Tx(I2C_SLAVE_ADDR, buff, nBytes);

	while (s_I2cCfgMaster.bIntEn)
	{
		if (g_bCompleted == true)
			break;
	}

	printf("Write %d bytes\r\n", c);
	printf("s_WriteRqstData: ");
	for (int i = offset; i < offset + nBytes; i++)
	{
		printf("%x ", s_WriteRqstData[i]);
	}
	printf("\r\n");
#endif

	g_bCompleted = false;

	// Master send read command to read 10 bytes from offset defined in data[0]
	//offset = 3;
	nBytes = 9;
	memset(buff, 0xFF, I2C_BUFF_SIZE);
#if 1
//	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, &offset, 1, buff, nBytes);
	//printf("Read %d bytes from offset %d: ", c, offset);
	printf("Rx %d bytes\r\n", nBytes);
	c = g_I2CMaster.Rx(I2C_SLAVE_ADDR, buff, nBytes);
	while (s_I2cCfgMaster.bIntEn)
	{
		__WFE();
		if (g_bCompleted == true)
		{
			c = g_RxCnt;
			break;
		}
	}
	printf("Read %d bytes: ", c);
	for (int i = 0; i < c; i++)
	{
		printf("%x ", buff[i]);
	}
	printf("\r\n");
	g_bCompleted = false;
#endif

#if 1
	// Master send read command without setting anything
	nBytes = 5;
	uint8_t ad[2] = {3, 13};
	//c = g_I2CMaster.Write(I2C_SLAVE_ADDR, &ad, 1, 0, 0);
#if 1
	g_Uart.printf("Read 1, %d\r\n", nBytes);
	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, ad, 1, buff, nBytes);
	while (s_I2cCfgMaster.bIntEn)
	{
		__WFE();
		if (g_bCompleted == true)
		{
			c = g_RxCnt;
			break;
		}
	}
	printf("Master send read command without setting anything %d bytes: \r\n", c);
	for (int i = 0; i < c; i++)
	{
		printf("%x ", buff[i]);
	}
	printf("\r\n");
//#else
	g_bCompleted = false;

	g_Uart.printf("Read %d\r\n", nBytes);
	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, NULL, 0, buff, nBytes);
	while (s_I2cCfgMaster.bIntEn)
	{
		__WFE();
		if (g_bCompleted == true)
		{
			c = g_RxCnt;
			break;
		}
	}
	printf("Master send read command without setting anything %d bytes: \r\n", c);
	for (int i = 0; i < c; i++)
	{
		printf("%x ", buff[i]);
	}
	printf("\r\n");
#endif
#endif

	while(1) __WFE();

	return 0;
}
