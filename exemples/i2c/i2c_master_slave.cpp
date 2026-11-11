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
#include "idelay.h"
#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

// One run tests every master and slave transfer mode combination.
// Master mode bits: 0 polling, 1 interrupt, 2 DMA, 3 DMA with interrupt.
// Slave mode bits: 0 interrupt, 1 DMA.
// A board restricts the set with these masks. A board that still defines the
// single mode I2C_MASTER_DMA_ENABLE / I2C_MASTER_INT_ENABLE and
// I2C_SLAVE_DMA_ENABLE flags keeps testing only that combination.
// Hardware capability of the selected instances, not driver maturity.
// A DMA request on capable hardware must initialize successfully.
#ifndef I2C_MASTER_HAS_DMA
#define I2C_MASTER_HAS_DMA 1
#endif
#ifndef I2C_SLAVE_HAS_DMA
#define I2C_SLAVE_HAS_DMA 1
#endif
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

#ifndef I2C_MASTER_SCL_DIR
#define I2C_MASTER_SCL_DIR IOPINDIR_OUTPUT
#endif
#ifndef I2C_SLAVE_SCL_DIR
#define I2C_SLAVE_SCL_DIR IOPINDIR_OUTPUT
#endif

//int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#ifndef I2C_TEST_UART_INT_ENABLE
#define I2C_TEST_UART_INT_ENABLE true
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
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	I2C_TEST_UART_INT_ENABLE,
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
#define I2C_SCL_RATE	100000
#ifndef I2C_TEST_RATES
#define I2C_TEST_RATES { I2C_SCL_RATE }
#endif

static int I2CMasterIntrfHandler(DevIntrf_t * const pDev,
	DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

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

// No device answers this address, for the address NACK test
#ifndef I2C_NACK_ADDR
#define I2C_NACK_ADDR			(I2C_SLAVE_ADDR + 1)
#endif

int I2CSlaveIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cSlavePins[] = {
	{I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},		// SDA
	{I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN, I2C_SLAVE_SCL_PINOP, I2C_SLAVE_SCL_DIR, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
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

#define I2C_BUFF_SIZE	128

// Most bytes the slave takes in one transfer, the command byte included.
// The longest write test sends this many bytes. A board.h overrides it with
// what its slave takes.
#ifndef I2C_SLAVE_MAX_XFER
#define I2C_SLAVE_MAX_XFER		255
#endif

// Data bytes of the longest write test, after the command byte
#define I2C_LONG_WRITE_LEN		(I2C_SLAVE_MAX_XFER - 1)

// Command byte of the write length tests: outside the slave memory, so the
// slave only keeps the data in its write buffer
#define I2C_WRITE_TEST_CMD		0xFF

static uint8_t s_ReadRqstData[I2C_BUFF_SIZE];
static uint8_t s_WriteRqstData[I2C_SLAVE_MAX_XFER];
static std::atomic<int> s_SlaveWriteLen(0);	// Bytes of the last completed write
static uint8_t s_WriteTestData[I2C_LONG_WRITE_LEN];
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
			if (s_Offset < 0 || s_Offset >= I2C_BUFF_SIZE)
			{
				s_Offset = 0;
			}
			g_I2CSlave.SetReadRqstData(0, &s_ReadRqstData[s_Offset], I2C_BUFF_SIZE - s_Offset);

			break;

		case DEVINTRF_EVT_WRITE_RQST:
			s_bWriteRqst = true;
			g_I2CSlave.SetWriteRqstBuffer(0, s_WriteRqstData, sizeof(s_WriteRqstData));
			break;

		case DEVINTRF_EVT_COMPLETED:

			if (Len < 0)
			{
				s_SlaveWriteLen = Len;
				s_Offset = 0;
				s_bWriteRqst = false;
				break;
			}

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
				s_SlaveWriteLen = Len;
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
	static char outputBuffer[256];

	setvbuf(stdout, outputBuffer, _IOLBF, sizeof(outputBuffer));
	g_Uart.Init(s_UartCfg);
#if defined(NDEBUG) || defined(UART_RETARGET)
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif

	printf("Init I2C Master/Slave demo\r\n");
#ifdef I2C_LOOPBACK_WIRING
	printf("%s\r\n", I2C_LOOPBACK_WIRING);
#endif
}

static const char *ModeName(const I2CCfg_t &Cfg)
{
	if (Cfg.bDmaEn)
		return Cfg.bIntEn ? "DMA+interrupt" : "DMA";
	return Cfg.bIntEn ? "interrupt" : "polling";
}

// Master write of the command byte then Len data bytes. Returns the data
// bytes written. In interrupt mode the count comes with the completion.
static int MasterWrite(const I2CCfg_t &Cfg, uint8_t Addr, uint8_t Cmd, const uint8_t *pData, int Len)
{
	int c;

	s_MasterCompleted = false;
	s_MasterCount = 0;
	c = g_I2CMaster.Write(Addr, &Cmd, 1, pData, Len);
	if (Cfg.bIntEn)
	{
		// DeviceIntrfWrite() converts the -1 of an interrupt transfer into 0
		// after subtracting the command length
		int total = WaitMasterComplete(10000000);
		c = total >= 1 ? total - 1 : 0;
	}

	return c;
}

// Register read of Len bytes at Offset
static int MasterRead(uint8_t Offset, uint8_t *pBuff, int Len)
{
	int c;

	s_MasterCompleted = false;
	s_MasterCount = 0;
	c = g_I2CMaster.Read(I2C_SLAVE_ADDR, &Offset, 1, pBuff, Len);
	if (c < 0)
	{
		c = WaitMasterComplete(10000000);
	}

	return c;
}

// Receive of Len bytes without a command, the slave continues from its last
// offset
static int MasterRx(uint8_t *pBuff, int Len)
{
	int c;

	s_MasterCompleted = false;
	s_MasterCount = 0;
	c = g_I2CMaster.Rx(I2C_SLAVE_ADDR, pBuff, Len);
	if (c < 0)
	{
		c = WaitMasterComplete(10000000);
	}

	return c;
}

static void PrintBytes(const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len && i < 16; ++i)
		printf(" %02x", pData[i]);
	if (Len > 16)
		printf(" ...");
	printf("\r\n");
}


// Check Len received bytes against the expected ones
static bool CheckRx(const char *pLabel, int Count, int Len, const uint8_t *pRx, const uint8_t *pExpected)
{
	bool pass = Count == Len && memcmp(pRx, pExpected, Len) == 0;

	printf("%s %d/%d bytes:", pLabel, Count, Len);
	PrintBytes(pRx, Count > 0 ? Count : 0);
	if (!pass)
	{
		printf("Expected:");
		PrintBytes(pExpected, Len);
	}

	return pass;
}

// Write Len data bytes after the test command. The slave must receive every
// byte, in order.
static bool WriteTest(const I2CCfg_t &Cfg, int Len)
{
	uint32_t timeout;
	int c;
	int got;
	bool pass;

	for (int i = 0; i < Len; ++i)
		s_WriteTestData[i] = (uint8_t)(i * 7 + 3);
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	s_SlaveWriteLen = 0;

	c = MasterWrite(Cfg, I2C_SLAVE_ADDR, I2C_WRITE_TEST_CMD, Len > 0 ? s_WriteTestData : NULL, Len);

	// The slave reports the write when it sees the STOP
	timeout = 1000000;
	while (s_SlaveWriteLen == 0 && --timeout != 0)
	{
	}
	got = s_SlaveWriteLen - 1;
	pass = c == Len && got == Len && s_WriteRqstData[0] == I2C_WRITE_TEST_CMD &&
		memcmp(&s_WriteRqstData[1], s_WriteTestData, Len) == 0;
	printf("Write %d bytes: master %d, slave %d %s\r\n", Len, c, got < 0 ? 0 : got,
		pass ? "OK" : "FAIL");
	if (!pass)
	{
		for (int i = 0; i < Len; ++i)
		{
			if (s_WriteRqstData[i + 1] != s_WriteTestData[i])
			{
				printf("Write mismatch at %d: %02x expected %02x\r\n",
					i, s_WriteRqstData[i + 1], s_WriteTestData[i]);
				break;
			}
		}
	}

	return pass;
}

#ifndef I2C_TEST_STRESS_ROUNDS
#define I2C_TEST_STRESS_ROUNDS 0
#endif

#if I2C_TEST_STRESS_ROUNDS > 0
static bool StressTest(const I2CCfg_t &Cfg)
{
	static const int lengths[] = { 1, 2, 8, 15, 16, 17, 31, 32, 33, 63, 64, 65 };
	uint8_t expected[66];
	uint8_t received[66];
	uint8_t offset;
	uint32_t timeout;
	unsigned round;
	int len;
	int count;
	int i;

	static_assert(I2C_SLAVE_MAX_XFER >= 67, "Stress test needs a 67-byte slave write buffer");
	printf("Stress: %u rounds\r\n", (unsigned)I2C_TEST_STRESS_ROUNDS);
	for (round = 0; round < I2C_TEST_STRESS_ROUNDS; round++)
	{
		len = lengths[round % (sizeof(lengths) / sizeof(lengths[0]))];
		offset = (uint8_t)(round % 32U);
		for (i = 0; i <= len; i++)
		{
			expected[i] = (uint8_t)(round * 37U + i * 13U + (round >> 8));
		}
		s_SlaveWriteLen = 0;
		count = MasterWrite(Cfg, I2C_SLAVE_ADDR, offset, expected, len + 1);
		timeout = 1000000;
		while (s_SlaveWriteLen == 0 && --timeout != 0)
		{
		}
		if (count != len + 1 || s_SlaveWriteLen != len + 2)
		{
			printf("Stress round %u WRITE FAIL: master=%d slave=%d expected=%d+command\r\n",
				round + 1, count, (int)s_SlaveWriteLen, len + 1);
			return false;
		}
		// Every hundredth read also checks recovery from an address NACK.
		if ((round + 1) % 100U == 0)
		{
			count = MasterWrite(Cfg, I2C_NACK_ADDR, offset, expected, len + 1);
			if (count != 0)
			{
				printf("Stress round %u NACK FAIL: returned %d\r\n", round + 1, count);
				return false;
			}
		}
		memset(received, 0, sizeof(received));
		count = MasterRead(offset, received, len);
		if (count != len || memcmp(received, expected, len) != 0)
		{
			printf("Stress round %u READ FAIL, offset=%u\r\n", round + 1, (unsigned)offset);
			CheckRx("Stress read", count, len, received, expected);
			return false;
		}
		// The extra byte written above independently checks the completion
		// length used to advance the slave's offset.
		count = MasterRx(received, 1);
		if (count != 1 || received[0] != expected[len])
		{
			printf("Stress round %u CONTINUE FAIL after %d bytes\r\n", round + 1, len);
			CheckRx("Stress continuation", count, 1, received, &expected[len]);
			return false;
		}
		if ((round + 1) % 100U == 0 || round + 1 == I2C_TEST_STRESS_ROUNDS)
		{
			printf("Stress %u/%u PASS\r\n", round + 1, (unsigned)I2C_TEST_STRESS_ROUNDS);
		}
	}
	return true;
}
#endif

// Run one master and slave combination.
// Return 1 for pass, 0 for fail, -1 when the combination is not supported.
static int RunCombo(const I2CCfg_t &MasterCfg, const I2CCfg_t &SlaveCfg)
{
	static const int writeLen[] = { 0, 1, 2, 8, I2C_LONG_WRITE_LEN };
	static const int readLen[] = { 1, 2, 8 };
	static const int boundaryLen[] = { 15, 16, 17, 31, 32, 33, 63, 64, 65 };
	const uint8_t wr[8] = { 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7 };
	const uint8_t offset = 4;
	uint8_t buff[I2C_BUFF_SIZE];
	uint8_t expected[I2C_BUFF_SIZE];
	char label[32];
	bool pass = true;
	int c;
	int i;

	if ((MasterCfg.bDmaEn && !I2C_MASTER_HAS_DMA) ||
		(SlaveCfg.bDmaEn && !I2C_SLAVE_HAS_DMA))
	{
		printf("Selected instance has no DMA hardware\r\n");
		return -1;
	}

	if (!g_I2CMaster.Init(MasterCfg))
	{
		printf("I2C master initialization failed\r\n");
		return 0;
	}
	if (!g_I2CSlave.Init(SlaveCfg))
	{
		printf("I2C slave initialization failed\r\n");
		g_I2CMaster.PowerOff();
		return 0;
	}

	printf("I2C configured rate: master %lu Hz, slave %lu Hz\r\n",
		(unsigned long)g_I2CMaster.Rate(), (unsigned long)g_I2CSlave.Rate());

	for (i = 0; i < I2C_BUFF_SIZE; ++i)
		s_ReadRqstData[i] = (uint8_t)i;
	memset(s_WriteRqstData, 0, sizeof(s_WriteRqstData));
	s_Offset = 0;
	s_bWriteRqst = false;


	// Register write of 8 bytes at offset 4
	c = MasterWrite(MasterCfg, I2C_SLAVE_ADDR, offset, wr, sizeof(wr));
	printf("Write %d/%d bytes at offset %d\r\n", c, (int)sizeof(wr), offset);
	pass = c == (int)sizeof(wr);

	// Register reads of 1, 2 and 8 bytes at offset 4
	for (i = 0; pass && i < (int)(sizeof(readLen) / sizeof(readLen[0])); ++i)
	{
		memset(buff, 0xFF, sizeof(buff));
		c = MasterRead(offset, buff, readLen[i]);
		snprintf(label, sizeof(label), "Read at offset %d", offset);
		pass = CheckRx(label, c, readLen[i], buff, wr);
	}

	// Receives without a command continue after the last read: 4 bytes at
	// offset 12, then 1 byte at offset 16
	if (pass)
	{
		memset(buff, 0xFF, sizeof(buff));
		c = MasterRx(buff, 4);
		memcpy(expected, &s_ReadRqstData[offset + sizeof(wr)], 4);
		snprintf(label, sizeof(label), "%s RX", ModeName(MasterCfg));
		pass = CheckRx(label, c, 4, buff, expected);
	}
	for (i = 0; pass && i < 2; i++)
	{
		memset(buff, 0xFF, sizeof(buff));
		c = MasterRx(buff, 1);
		expected[0] = s_ReadRqstData[offset + sizeof(wr) + 4 + i];
		pass = CheckRx(label, c, 1, buff, expected);
	}

	// Reads around common 16- and 32-byte FIFO boundaries. Check a following
	// command-free read too: correct data alone does not prove the slave
	// reported the correct completion length.
	for (i = 0; pass && i < (int)(sizeof(boundaryLen) / sizeof(boundaryLen[0])); i++)
	{
		memset(buff, 0xff, sizeof(buff));
		c = MasterRead(32, buff, boundaryLen[i]);
		pass = CheckRx("Boundary read at offset 32", c, boundaryLen[i],
			buff, &s_ReadRqstData[32]);
		if (pass)
		{
			c = MasterRx(buff, 1);
			expected[0] = s_ReadRqstData[32 + boundaryLen[i]];
			pass = CheckRx("Read after boundary", c, 1, buff, expected);
		}
	}

	// Writes of 0 (command only), 1, 2, 8 and I2C_LONG_WRITE_LEN data bytes
	for (i = 0; pass && i < (int)(sizeof(writeLen) / sizeof(writeLen[0])); ++i)
	{
		pass = WriteTest(MasterCfg, writeLen[i]);
	}

	// Run recovery even if an earlier data check failed.
	{
		// Address NACK: no device answers I2C_NACK_ADDR. No data byte may be
		// reported as written, and a register read must work after the error.
		printf("Starting NACK test (MaxRetry=%d)\r\n", MasterCfg.MaxRetry);
		fflush(stdout);
		c = MasterWrite(MasterCfg, I2C_NACK_ADDR, offset, wr, sizeof(wr));
		printf("NACK write %d/%d bytes at address 0x%02x\r\n", c, (int)sizeof(wr), I2C_NACK_ADDR);
		pass = pass && c == 0;

		memset(buff, 0xFF, sizeof(buff));
		c = MasterRead(offset, buff, sizeof(wr));
		if (!CheckRx("Read after NACK", c, sizeof(wr), buff, wr))
		{
			pass = false;
		}
	}

#if I2C_TEST_STRESS_ROUNDS > 0
	if (pass)
	{
		pass = StressTest(MasterCfg);
	}
#endif

	g_I2CMaster.PowerOff();
	g_I2CSlave.PowerOff();
	return pass ? 1 : 0;
}

int main()
{
	static const uint32_t rates[] = I2C_TEST_RATES;
	unsigned rateIndex;
	I2CCfg_t masterCfg;
	I2CCfg_t slaveCfg;
	int mm;
	int sm;
	int rc;
	int nPass = 0;
	int nFail = 0;
	int nSkip = 0;

	HardwareInit();

	for (rateIndex = 0; rateIndex < sizeof(rates) / sizeof(rates[0]); rateIndex++)
	{
		printf("\r\nI2C requested rate: %lu Hz\r\n", (unsigned long)rates[rateIndex]);
		for (sm = 0; sm < 2; ++sm)
		{
			if ((I2C_TEST_SLAVE_MASK & (1 << sm)) == 0)
				continue;

			for (mm = 0; mm < 4; ++mm)
			{
				if ((I2C_TEST_MASTER_MASK & (1 << mm)) == 0)
					continue;

				masterCfg = s_I2cCfgMaster;
				masterCfg.Rate = rates[rateIndex];
				masterCfg.bDmaEn = mm >= 2;
				masterCfg.bIntEn = mm == 1 || mm == 3;
				slaveCfg = s_I2cCfgSlave;
				slaveCfg.Rate = rates[rateIndex];
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
	}

	printf("\r\nI2C master/slave loopback %s (%d passed, %d failed, %d skipped)\r\n",
		(nFail == 0 && nPass > 0) ? "PASS" : "FAIL", nPass, nFail, nSkip);

	fflush(stdout);
	while (1)
		__WFE();

	return 0;
}
