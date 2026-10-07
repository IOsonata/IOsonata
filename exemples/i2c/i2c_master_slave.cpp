/**-------------------------------------------------------------------------
@example	i2c_master_slave.cpp

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

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#ifndef I2C_MASTER_DMA_ENABLE
#define I2C_MASTER_DMA_ENABLE true
#endif

#ifndef I2C_MASTER_INT_ENABLE
#define I2C_MASTER_INT_ENABLE false
#endif

#ifndef I2C_SLAVE_DMA_ENABLE
#define I2C_SLAVE_DMA_ENABLE true
#endif

#ifndef I2C_SLAVE_INT_ENABLE
#define I2C_SLAVE_INT_ENABLE true
#endif

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
	g_TxBuff,
};

UART g_Uart;

//********** I2C Master **********
#define I2C_SCL_RATE	100000 // Rate in Hz, supported 100k, 250k, and 400k

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
	.bDmaEn = I2C_MASTER_DMA_ENABLE,
	.bIntEn = I2C_MASTER_INT_ENABLE,
	.IntPrio = 7,			// Interrupt prio
	.EvtCB = I2CMasterIntrfHandler		// Event callback
};

I2C g_I2CMaster;

static std::atomic<bool> s_MasterCompleted(false);
static std::atomic<int> s_MasterCount(0);

static int I2CMasterIntrfHandler(DevIntrf_t * const pDev,
	DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	if (EvtId == DEVINTRF_EVT_COMPLETED)
	{
		s_MasterCount = Len;
		s_MasterCompleted = true;
	}
	return 0;
}

static int WaitMasterComplete(int Timeout)
{
	while (!s_MasterCompleted && --Timeout > 0)
	{
		// Interrupt completion updates the atomic state.
	}
	return s_MasterCompleted ? (int)s_MasterCount : -1;
}

//********** I2C Slave **********

#define I2C_SLAVE_ADDR			0x22

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cSlavePins[] = {
	{I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},		// SDA
	{I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN, I2C_SLAVE_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},	// SCL
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
	.bDmaEn = I2C_SLAVE_DMA_ENABLE,			// DMA mode enable
	.bIntEn = I2C_SLAVE_INT_ENABLE,						// Interrupt enable
	.IntPrio = 7,						// Interrupt priority
	.EvtCB = I2CSlaveIntrfHandler		// Event callback
};

I2C g_I2CSlave;

#define I2C_BUFF_SIZE	20
static uint8_t s_ReadRqstData[I2C_BUFF_SIZE];
static uint8_t s_WriteRqstData[I2C_BUFF_SIZE];
static bool s_bWriteRqst = false;
static int s_Offset = 0;

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bWriteRqst)
			{
				// There was a write command previously
				// get the offset.  parameter Len should indicates number of byte
				// was written prior to change state to read request
				if (Len > 0)
				{
					s_Offset = s_WriteRqstData[0];
				}
				s_bWriteRqst = false;
			}
			g_I2CSlave.SetReadRqstData(0, &s_ReadRqstData[s_Offset], I2C_BUFF_SIZE);

			break;

		case DEVINTRF_EVT_WRITE_RQST:
			s_bWriteRqst = true;
			g_I2CSlave.SetWriteRqstBuffer(0, s_WriteRqstData, I2C_BUFF_SIZE);
			break;

		case DEVINTRF_EVT_COMPLETED:

			if (s_bWriteRqst == false)
			{
				// No previous write command. This means a continuous read
				// move to last read
				s_Offset += Len;
				if (s_Offset >= I2C_BUFF_SIZE)
				{
					// Reset offset if we moved beyond max buffer
					s_Offset = 0;
				}
			}
			else
			{
				// Write data completed. Byte 0 is the slave-memory offset.
				if (Len > 1 && s_WriteRqstData[0] < I2C_BUFF_SIZE)
				{
					int n = Len - 1;
					if (n > I2C_BUFF_SIZE - s_WriteRqstData[0])
						n = I2C_BUFF_SIZE - s_WriteRqstData[0];
					memcpy(&s_ReadRqstData[s_WriteRqstData[0]], &s_WriteRqstData[1], n);
				}
			}
			s_bWriteRqst = false;
			break;
	}

	return I2C_BUFF_SIZE;
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
#ifdef NDEBUG
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif

	printf("Init I2C Master/Slave demo\r\n");
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
	uint8_t buff[I2C_BUFF_SIZE];
	uint8_t wr[8] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7 };
	uint8_t offset = 4;

	HardwareInit();

	bool masterOk = g_I2CMaster.Init(s_I2cCfgMaster);
	bool slaveOk = g_I2CSlave.Init(s_I2cCfgSlave);
	printf("I2C init master=%d slave=%d\r\n", masterOk, slaveOk);
	if (!masterOk || !slaveOk)
	{
		while (1)
			__WFE();
	}

	for (int i = 0; i < I2C_BUFF_SIZE; ++i)
		s_ReadRqstData[i] = (uint8_t)i;
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	memset(buff, 0xFF, sizeof(buff));

	s_MasterCompleted = false;
	s_MasterCount = 0;
	int c = g_I2CMaster.Write(I2C_SLAVE_ADDR, &offset, 1, wr, sizeof(wr));
	if (s_I2cCfgMaster.bIntEn)
	{
		const int total = WaitMasterComplete(10000000);
		c = total >= 1 ? total - 1 : 0;
	}
	printf("Write %d/%d bytes at offset %d\r\n", c, (int)sizeof(wr), offset);

	memset(buff, 0xFF, sizeof(buff));
	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, &offset, 1, buff, sizeof(wr));
	printf("Read %d/%d bytes at offset %d:", c, (int)sizeof(wr), offset);
	for (int i = 0; i < c; ++i)
		printf(" %02x", buff[i]);
	printf("\r\n");

	bool pass = c == (int)sizeof(wr) &&
		memcmp(buff, wr, sizeof(wr)) == 0;

	if (pass && s_I2cCfgMaster.bIntEn)
	{
		// The register-style read above validates the SAM4L CMDR/NCMDR
		// repeated-start path. Exercise the ordinary interrupt RX path too.
		uint8_t rx[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
		s_MasterCompleted = false;
		s_MasterCount = 0;
		int rc = g_I2CMaster.Rx(I2C_SLAVE_ADDR, rx, sizeof(rx));
		if (rc < 0)
			rc = WaitMasterComplete(10000000);
		printf("Interrupt RX %d/%d bytes:", rc, (int)sizeof(rx));
		for (int i = 0; i < rc; ++i)
			printf(" %02x", rx[i]);
		printf("\r\n");
		pass = rc == (int)sizeof(rx);
	}

	printf("I2C master/slave loopback %s\r\n", pass ? "PASS" : "FAIL");

	while (1)
		__WFE();

	return 0;
}
