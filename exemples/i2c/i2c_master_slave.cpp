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

#include "coredev/system_core_clock.h"
#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

// Optional target-test diagnostic implementation, outside board configuration.
// Called after a failed run.
void I2cMasterSlaveDiagnostics(const char *pWhen) __attribute__((weak));

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// One run tests every master and slave transfer mode combination.
// Master mode bits: 0 polling, 1 interrupt, 2 DMA, 3 DMA with interrupt.
// Slave mode bits: 0 interrupt, 1 DMA.
// A board restricts the set with these masks. A board that still defines the
// single mode I2C_MASTER_DMA_ENABLE / I2C_MASTER_INT_ENABLE and
// I2C_SLAVE_DMA_ENABLE flags keeps testing only that combination.
#ifndef I2C_TEST_MASTER_MASK
#if defined(I2C_MASTER_DMA_ENABLE) || defined(I2C_MASTER_INT_ENABLE)
#ifndef I2C_MASTER_DMA_ENABLE
#define I2C_MASTER_DMA_ENABLE true
#endif
#ifndef I2C_MASTER_INT_ENABLE
#define I2C_MASTER_INT_ENABLE false
#endif
#define I2C_TEST_MASTER_MASK \
	(1 << ((I2C_MASTER_DMA_ENABLE ? 2 : 0) + (I2C_MASTER_INT_ENABLE ? 1 : 0)))
#else
#define I2C_TEST_MASTER_MASK 0x0F
#endif
#endif

#ifndef I2C_TEST_SLAVE_MASK
#if defined(I2C_SLAVE_DMA_ENABLE)
#define I2C_TEST_SLAVE_MASK (I2C_SLAVE_DMA_ENABLE ? 2 : 1)
#else
#define I2C_TEST_SLAVE_MASK 0x03
#endif
#endif

#ifndef I2C_MASTER_INT_PRIO
#define I2C_MASTER_INT_PRIO 7
#endif

#ifndef I2C_SLAVE_INT_PRIO
#define I2C_SLAVE_INT_PRIO 7
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

static int I2CMasterIntrfHandler(DevIntrf_t * const pDev,
	DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

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
	.bDmaEn = false,			// set for each run
	.bIntEn = false,
	.IntPrio = I2C_MASTER_INT_PRIO,	// Interrupt prio
	.EvtCB = I2CMasterIntrfHandler		// Event callback
};

I2C g_I2CMaster;

static std::atomic<bool> s_MasterCompleted(false);
static std::atomic<int> s_MasterCount(0);

//********** I2C Slave **********

#define I2C_SLAVE_ADDR			0x22

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cSlavePins[] = {
	{I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},		// SDA
	{I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN, I2C_SLAVE_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
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
	.bDmaEn = false,					// set for each run
	.bIntEn = true,
	.IntPrio = I2C_SLAVE_INT_PRIO,		// Interrupt priority
	.EvtCB = I2CSlaveIntrfHandler		// Event callback
};

I2C g_I2CSlave;

#define I2C_BUFF_SIZE	20
static uint8_t s_ReadRqstData[I2C_BUFF_SIZE];
static uint8_t s_WriteRqstData[I2C_BUFF_SIZE];
static bool s_bWriteRqst = false;
static int s_Offset = 0;

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

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	int n;

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
					n = Len - 1;
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

static const char *ModeName(const I2CCfg_t &Cfg)
{
	if (Cfg.bDmaEn)
		return Cfg.bIntEn ? "DMA+interrupt" : "DMA";
	return Cfg.bIntEn ? "interrupt" : "polling";
}

// Run one master and slave combination.
// Return 1 for pass, 0 for fail, -1 when the combination is not supported.
static int RunCombo(const I2CCfg_t &MasterCfg, const I2CCfg_t &SlaveCfg)
{
	const uint8_t expectedRx[4] = { 0x0C, 0x0D, 0x0E, 0x0F };
	uint8_t rx[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
	bool masterOk;
	bool slaveOk;
	int c;
	int total;
	bool writeOk;
	bool pass;
	int rc;
	int i;
	uint8_t buff[I2C_BUFF_SIZE];
	uint8_t wr[8] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7 };
	uint8_t offset = 4;

	masterOk = g_I2CMaster.Init(MasterCfg);
#ifdef I2C_SLAVE_SKIP_INIT
	// Board test: leave the slave unit stopped to exercise the master alone.
	// The master then sees no ACK, which is the expected result.
	slaveOk = true;
	printf("I2C slave not started\r\n");
#else
	slaveOk = g_I2CSlave.Init(SlaveCfg);
#endif
	if (!masterOk || !slaveOk)
	{
		printf("Init master=%d slave=%d, combination not supported\r\n", masterOk, slaveOk);
		return -1;
	}
#ifdef I2C_PINS_STRONG_DRIVE
	// Optional board test: raise the output drive of the bus pins after Init
	if (I2C_PINS_STRONG_DRIVE)
	{
		IOPinSetStrength(I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, IOPINSTRENGTH_STRONG);
		IOPinSetStrength(I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, IOPINSTRENGTH_STRONG);
		IOPinSetStrength(I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN, IOPINSTRENGTH_STRONG);
		IOPinSetStrength(I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN, IOPINSTRENGTH_STRONG);
		printf("I2C pins strong drive\r\n");
	}
#endif

	for (i = 0; i < I2C_BUFF_SIZE; ++i)
		s_ReadRqstData[i] = (uint8_t)i;
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	memset(buff, 0xFF, sizeof(buff));
	s_Offset = 0;
	s_bWriteRqst = false;

	s_MasterCompleted = false;
	s_MasterCount = 0;
	c = g_I2CMaster.Write(I2C_SLAVE_ADDR, &offset, 1, wr, sizeof(wr));
	if (MasterCfg.bIntEn)
	{
		// DeviceIntrfWrite() converts the target TxData() async -1 return
		// into 0 after subtracting the address-command length. In interrupt
		// mode completion must therefore be taken from the callback.
		total = WaitMasterComplete(10000000);
		c = total >= 1 ? total - 1 : 0;
	}
	printf("Write %d/%d bytes at offset %d\r\n", c, (int)sizeof(wr), offset);
	writeOk = c == (int)sizeof(wr);

	memset(buff, 0xFF, sizeof(buff));
	s_MasterCompleted = false;
	s_MasterCount = 0;
	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, &offset, 1, buff, sizeof(wr));
	if (c < 0)
	{
		// Reception completed in the background, the count comes with the
		// callback as for Rx below
		c = WaitMasterComplete(10000000);
	}
	printf("Read %d/%d bytes at offset %d:", c, (int)sizeof(wr), offset);
	for (i = 0; i < c; ++i)
		printf(" %02x", buff[i]);
	printf("\r\n");

	pass = writeOk && c == (int)sizeof(wr) &&
		memcmp(buff, wr, sizeof(wr)) == 0;

	if (pass)
	{
		// The register-style read above validates the SAM4L CMDR/NCMDR
		// repeated-start path. Exercise the ordinary RX path for the selected
		// transfer engine too.
		s_MasterCompleted = false;
		s_MasterCount = 0;
		rc = g_I2CMaster.Rx(I2C_SLAVE_ADDR, rx, sizeof(rx));
		if (rc < 0)
			rc = WaitMasterComplete(10000000);
		printf("%s RX %d/%d bytes:", ModeName(MasterCfg), rc, (int)sizeof(rx));
		for (i = 0; i < rc && i < (int)sizeof(rx); ++i)
			printf(" %02x", rx[i]);
		printf("\r\n");
		pass = rc == (int)sizeof(rx) &&
			memcmp(rx, expectedRx, sizeof(rx)) == 0;
		if (!pass)
			printf("Expected RX 4/4 bytes: 0c 0d 0e 0f\r\n");
	}

	if (!pass && I2cMasterSlaveDiagnostics != nullptr)
		I2cMasterSlaveDiagnostics("after failure");
	return pass ? 1 : 0;
}

int main()
{
	I2CCfg_t masterCfg;
	I2CCfg_t slaveCfg;
	int mm;
	int sm;
	int rc;
	int nPass = 0;
	int nFail = 0;
	int nSkip = 0;

	HardwareInit();

	for (sm = 0; sm < 2; ++sm)
	{
		if ((I2C_TEST_SLAVE_MASK & (1 << sm)) == 0)
			continue;

		for (mm = 0; mm < 4; ++mm)
		{
			if ((I2C_TEST_MASTER_MASK & (1 << mm)) == 0)
				continue;

			masterCfg = s_I2cCfgMaster;
			masterCfg.bDmaEn = mm >= 2;
			masterCfg.bIntEn = mm == 1 || mm == 3;
			slaveCfg = s_I2cCfgSlave;
			slaveCfg.bDmaEn = sm == 1;
			slaveCfg.bIntEn = true;

			printf("\r\n== I2C master %s, slave %s ==\r\n",
				ModeName(masterCfg), sm == 1 ? "DMA" : "interrupt");
			rc = RunCombo(masterCfg, slaveCfg);
			if (rc > 0)
			{
				++nPass;
				printf("-> PASS\r\n");
			}
			else if (rc == 0)
			{
				++nFail;
				printf("-> FAIL\r\n");
			}
			else
			{
				++nSkip;
				printf("-> SKIPPED\r\n");
			}
		}
	}

	printf("\r\nI2C master/slave loopback %s (%d passed, %d failed, %d skipped)\r\n",
		(nFail == 0 && nPass > 0) ? "PASS" : "FAIL", nPass, nFail, nSkip);

	while (1)
		__WFE();

	return 0;
}
