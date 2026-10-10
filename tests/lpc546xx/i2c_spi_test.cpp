// Host regression of the LPC546xx Flexcomm I2C and SPI drivers with register
// models. The I2C model is the master state machine of the Flexcomm I2C with
// one simulated slave. The SPI model shifts each frame written to FIFOWR
// through a loopback, MOSI wired to MISO.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>

#include "LPC546xx.h"

// Flexcomm PSELID, the present bits and the ID are read only
struct Psel {
	uint32_t value = 0x00000070U;
	operator uint32_t() const { return value; }
	Psel &operator=(uint32_t v) { value = (value & ~0xFU) | (v & 0xFU); return *this; }
};

struct FcRegs {
	uint8_t Reserved[0xFF8];
	Psel PSELID;
	uint32_t PID;
};

// I2C master model
struct I2cModel;
struct MstCtl {
	uint32_t value = 0;
	operator uint32_t() const { return value; }
	MstCtl &operator=(uint32_t v);
};
struct W1CStat {
	uint32_t value = 0;
	operator uint32_t() const;
	W1CStat &operator=(uint32_t v) { value &= ~v; return *this; }
};
struct I2cRegs {
	uint32_t CFG = 0;
	W1CStat STAT;
	uint32_t INTENSET = 0, INTENCLR = 0, TIMEOUT = 0, CLKDIV = 0, INTSTAT = 0;
	MstCtl MSTCTL;
	uint32_t MSTTIME = 0, MSTDAT = 0;
};

struct I2cModel {
	int State = 0;					// MSTSTATE
	bool bPending = true;
	uint8_t SlaveAddr = 0x1D;
	uint8_t Regs[256] = {};
	uint8_t RegPtr = 0;
	bool bFirstWrite = false;
	int NackAfter = -1;				// NACK the data byte with this index
	int WriteCnt = 0;
	std::string Log;
	I2cRegs *pRegs = nullptr;

	void Ctl(uint32_t v)
	{
		assert(bPending);
		if (v & I2C_MSTCTL_MSTSTART_MASK)
		{
			if (State == 1)
			{
				Log += "N";
			}
			uint32_t d = pRegs->MSTDAT;
			Log += State == 0 ? "S" : "Sr";
			Log += (d & 1) ? "R" : "W";
			if ((d >> 1) != SlaveAddr)
			{
				State = 3;
				Log += "n";
			}
			else if (d & 1)
			{
				pRegs->MSTDAT = Regs[RegPtr++];
				State = 1;
			}
			else
			{
				bFirstWrite = true;
				WriteCnt = 0;
				State = 2;
			}
		}
		else if (v & I2C_MSTCTL_MSTSTOP_MASK)
		{
			if (State == 1)
			{
				Log += "N";
			}
			Log += "P";
			State = 0;
		}
		else if (v & I2C_MSTCTL_MSTCONTINUE_MASK)
		{
			if (State == 2)
			{
				uint8_t b = (uint8_t)pRegs->MSTDAT;
				char s[8];
				snprintf(s, sizeof(s), "%02X", b);
				Log += s;
				if (WriteCnt++ == NackAfter)
				{
					Log += "n";
					State = 4;
				}
				else if (bFirstWrite)
				{
					RegPtr = b;
					bFirstWrite = false;
				}
				else
				{
					Regs[RegPtr++] = b;
				}
			}
			else if (State == 1)
			{
				Log += "A";
				pRegs->MSTDAT = Regs[RegPtr++];
			}
			else
			{
				assert(false);
			}
		}
		bPending = true;
	}
};

static I2cModel s_I2c;

MstCtl &MstCtl::operator=(uint32_t v)
{
	value = v;
	s_I2c.Ctl(v);
	return *this;
}

W1CStat::operator uint32_t() const
{
	return value | (s_I2c.bPending ? I2C_STAT_MSTPENDING_MASK : 0U) |
		   ((uint32_t)s_I2c.State << I2C_STAT_MSTSTATE_SHIFT);
}

// SPI loopback model, every frame not ignored comes back in the RX FIFO
static uint8_t s_SpiRx[64];
static int s_SpiRxHead = 0, s_SpiRxTail = 0;
static std::string s_SpiLog;
static uint32_t s_SpiLastCtrl = 0;
static bool SpiCsLow(void);

struct FifoWr {
	uint32_t value = 0;
	operator uint32_t() const { return value; }
	FifoWr &operator=(uint32_t v)
	{
		value = v;
		s_SpiLastCtrl = v & 0xFFFF0000U;
		char s[8];
		snprintf(s, sizeof(s), "%02X", v & 0xFFU);
		s_SpiLog += s;
		assert(SpiCsLow());
		if ((v & SPI_FIFOWR_RXIGNORE_MASK) == 0)
		{
			s_SpiRx[s_SpiRxHead++ & 63] = (uint8_t)v;
			// The driver keeps at most LPC546XX_SPI_INFLIGHT frames in flight
			assert(s_SpiRxHead - s_SpiRxTail <= 4);
		}
		return *this;
	}
};
struct FifoStat {
	operator uint32_t() const
	{
		return SPI_FIFOSTAT_TXEMPTY_MASK | SPI_FIFOSTAT_TXNOTFULL_MASK |
			   (s_SpiRxHead != s_SpiRxTail ? SPI_FIFOSTAT_RXNOTEMPTY_MASK : 0U);
	}
	FifoStat &operator=(uint32_t v) { (void)v; return *this; }
};
struct FifoRd {
	operator uint32_t() const
	{
		assert(s_SpiRxHead != s_SpiRxTail);
		return s_SpiRx[s_SpiRxTail++ & 63];
	}
};
struct SpiRegs {
	uint32_t CFG = 0, DLY = 0, STAT = SPI_STAT_MSTIDLE_MASK, INTENSET = 0, INTENCLR = 0, DIV = 0, INTSTAT = 0;
	uint32_t FIFOCFG = 0;
	FifoStat FIFOSTAT;
	uint32_t FIFOTRIG = 0, FIFOINTENSET = 0, FIFOINTENCLR = 0, FIFOINTSTAT = 0;
	FifoWr FIFOWR;
	FifoRd FIFORD;
	uint32_t FIFORDNOPOP = 0;
};

// GPIO with SET and CLR acting on PIN
struct GpioWr {
	uint32_t *pPin = nullptr;
	bool bSet = true;
	GpioWr &operator=(uint32_t v) { *pPin = bSet ? (*pPin | v) : (*pPin & ~v); return *this; }
};
struct GpioRegs {
	uint8_t B[6][32] = {};
	uint32_t DIR[6] = {}, PIN[6] = {}, NOT[6] = {}, DIRSET[6] = {}, DIRCLR[6] = {};
	GpioWr SET[6], CLR[6];
	GpioRegs()
	{
		for (int i = 0; i < 6; i++)
		{
			SET[i].pPin = &PIN[i];
			CLR[i].pPin = &PIN[i];
			CLR[i].bSet = false;
		}
	}
};

alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static FcRegs s_Fc[10];
static I2cRegs s_I2cRegs[10];
static SpiRegs s_SpiRegs[10];
static GpioRegs s_Gpio;
uint32_t g_IrqMasked = 0;
static uint32_t s_NvicEn = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }

#undef SYSCON
#undef FLEXCOMM0
#undef FLEXCOMM1
#undef FLEXCOMM2
#undef FLEXCOMM3
#undef FLEXCOMM4
#undef FLEXCOMM5
#undef FLEXCOMM6
#undef FLEXCOMM7
#undef FLEXCOMM8
#undef FLEXCOMM9
#undef I2C0
#undef I2C1
#undef I2C2
#undef I2C3
#undef I2C4
#undef I2C5
#undef I2C6
#undef I2C7
#undef I2C8
#undef I2C9
#undef SPI0
#undef SPI1
#undef SPI2
#undef SPI3
#undef SPI4
#undef SPI5
#undef SPI6
#undef SPI7
#undef SPI8
#undef SPI9
#undef GPIO
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define FLEXCOMM_Type FcRegs
#define FLEXCOMM0 (&s_Fc[0])
#define FLEXCOMM1 (&s_Fc[1])
#define FLEXCOMM2 (&s_Fc[2])
#define FLEXCOMM3 (&s_Fc[3])
#define FLEXCOMM4 (&s_Fc[4])
#define FLEXCOMM5 (&s_Fc[5])
#define FLEXCOMM6 (&s_Fc[6])
#define FLEXCOMM7 (&s_Fc[7])
#define FLEXCOMM8 (&s_Fc[8])
#define FLEXCOMM9 (&s_Fc[9])
#define I2C_Type I2cRegs
#define I2C0 (&s_I2cRegs[0])
#define I2C1 (&s_I2cRegs[1])
#define I2C2 (&s_I2cRegs[2])
#define I2C3 (&s_I2cRegs[3])
#define I2C4 (&s_I2cRegs[4])
#define I2C5 (&s_I2cRegs[5])
#define I2C6 (&s_I2cRegs[6])
#define I2C7 (&s_I2cRegs[7])
#define I2C8 (&s_I2cRegs[8])
#define I2C9 (&s_I2cRegs[9])
#define SPI_Type SpiRegs
#define SPI0 (&s_SpiRegs[0])
#define SPI1 (&s_SpiRegs[1])
#define SPI2 (&s_SpiRegs[2])
#define SPI3 (&s_SpiRegs[3])
#define SPI4 (&s_SpiRegs[4])
#define SPI5 (&s_SpiRegs[5])
#define SPI6 (&s_SpiRegs[6])
#define SPI7 (&s_SpiRegs[7])
#define SPI8 (&s_SpiRegs[8])
#define SPI9 (&s_SpiRegs[9])
#define GPIO (&s_Gpio)

#include "coredev/iopincfg.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"

// Pin configuration is not modeled
static int s_PinCfgCnt = 0;
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
	s_PinCfgCnt++;
}
void IOPinDisable(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }
void I2CBusReset(I2CDev_t * const pDev) { (void)pDev; }

#include "../../src/coredev/shared_intrf.cpp"
#include "../../ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp"
#include "../../ARM/NXP/LPC546xx/src/i2c_lpc546xx.cpp"
#include "../../ARM/NXP/LPC546xx/src/spi_lpc546xx.cpp"
#include "../../src/device_intrf.cpp"

// SPI chip select on P3_30, active low
static const IOPinCfg_t s_SpiPins[] = {
	{3, 20, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 22, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{3, 21, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 30, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static bool SpiCsLow(void)
{
	return (s_Gpio.PIN[3] & (1UL << 30)) == 0;
}

static const IOPinCfg_t s_I2cPins[] = {
	{3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{3, 24, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static void I2cTest(void)
{
	I2CDev_t i2c;
	I2CCfg_t cfg = {};

	memset((void *)&i2c, 0, sizeof(i2c));
	cfg.DevNo = 2;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = s_I2cPins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;

	// Configurations not supported
	I2CCfg_t bad = cfg;
	bad.Mode = I2CMODE_SLAVE;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.bIntEn = true;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.bDmaEn = true;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.AddrType = I2CADDR_TYPE_EXT;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.Type = I2CTYPE_SMBUS;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.DevNo = 10;
	assert(!I2CInit(&i2c, &bad));
	bad = cfg;
	bad.NbIOPins = 1;
	assert(!I2CInit(&i2c, &bad));

	// 100 kHz, 6 us low and 4 us high from 12 MHz / 8
	s_I2c.pRegs = &s_I2cRegs[2];
	assert(I2CInit(&i2c, &cfg));
	assert((uint32_t)s_Fc[2].PSELID == 0x73U);
	assert(s_I2cRegs[2].CFG == I2C_CFG_MSTEN_MASK);
	assert(i2c.Cfg.Rate == 100000 && s_I2cRegs[2].CLKDIV == 7U && s_I2cRegs[2].MSTTIME == ((9U - 2U) | ((6U - 2U) << 4)));
	assert(i2c.DevIntrf.MaxRetry == I2C_MAX_RETRY);

	// 400 kHz, 1.5 us low, 1 us high. 1 MHz, 0.67 us low, 0.33 us high.
	assert(I2CSetRate(&i2c, 400000) == 400000 && s_I2cRegs[2].CLKDIV == 1U &&
		   s_I2cRegs[2].MSTTIME == ((9U - 2U) | ((6U - 2U) << 4)));
	assert(I2CSetRate(&i2c, 1000000) == 1000000 && s_I2cRegs[2].CLKDIV == 0U &&
		   s_I2cRegs[2].MSTTIME == ((8U - 2U) | ((4U - 2U) << 4)));
	assert(I2CSetRate(&i2c, 100000) == 100000);

	// Register write and read back
	uint8_t reg = 0x2A;
	uint8_t data[6] = {0x01, 0x02, 0x03};
	s_I2c.Regs[0x0D] = 0x4A;
	s_I2c.Log.clear();
	assert(I2CWrite(&i2c, 0x1D, &reg, 1, data, 3) == 3);
	assert(s_I2c.Log == "SW2A010203P" && s_I2c.State == 0);
	assert(s_I2c.Regs[0x2A] == 1 && s_I2c.Regs[0x2B] == 2 && s_I2c.Regs[0x2C] == 3);

	s_I2c.Log.clear();
	reg = 0x0D;
	memset(data, 0, sizeof(data));
	assert(I2CRead(&i2c, 0x1D, &reg, 1, data, 1) == 1);
	assert(data[0] == 0x4A && s_I2c.Log == "SW0DSrRNP");

	// Multi byte read, all bytes acknowledged except the last
	s_I2c.Log.clear();
	reg = 0x2A;
	memset(data, 0, sizeof(data));
	assert(I2CRead(&i2c, 0x1D, &reg, 1, data, 3) == 3);
	assert(data[0] == 1 && data[1] == 2 && data[2] == 3 && s_I2c.Log == "SW2ASrRAANP");

	// Read in two RxData calls of one transfer
	s_I2c.Log.clear();
	s_I2c.RegPtr = 0x2A;
	memset(data, 0, sizeof(data));
	assert(I2CStartRx(&i2c, 0x1D));
	assert(I2CRxData(&i2c, data, 1) == 1 && I2CRxData(&i2c, &data[1], 2) == 2);
	I2CStopRx(&i2c);
	assert(data[0] == 1 && data[1] == 2 && data[2] == 3 && s_I2c.Log == "SRAANP");

	// No device at the address: every retry releases the bus
	s_I2c.Log.clear();
	assert(I2CTx(&i2c, 0x50, data, 2) == 0);
	assert(s_I2c.Log == "SWnPSWnPSWnPSWnPSWnPSWnP" && s_I2c.State == 0);
	assert(!I2CStartTx(&i2c, 0x50));
	assert(!atomic_flag_test_and_set(&i2c.DevIntrf.bBusy));
	atomic_flag_clear(&i2c.DevIntrf.bBusy);
	assert(!I2CStartTx(&i2c, 0x80));

	// Data NACK on the third byte, two bytes acknowledged
	s_I2c.Log.clear();
	s_I2c.NackAfter = 2;
	data[0] = 0x10;
	assert(I2CTx(&i2c, 0x1D, data, 4) == 2);
	assert(s_I2c.Log == "SW100203nP");
	assert(s_I2c.State == 0);
	s_I2c.NackAfter = -1;

	printf("lpc546xx i2c: PASS\n");
}

static void SpiTest(void)
{
	SPIDev_t spi;
	SPICfg_t cfg = {};

	memset((void *)&spi, 0, sizeof(spi));
	cfg.DevNo = 9;
	cfg.Mode = SPIMODE_MASTER;
	cfg.pIOPinMap = s_SpiPins;
	cfg.NbIOPins = 4;
	cfg.Rate = 1000000;
	cfg.DataSize = 8;
	cfg.DataPhase = SPIDATAPHASE_SECOND_CLK;
	cfg.ClkPol = SPICLKPOL_LOW;
	cfg.ChipSel = SPICSEL_AUTO;
	cfg.DummyByte = 0xA5;

	SPICfg_t bad = cfg;
	bad.Mode = SPIMODE_SLAVE;
	assert(!SPIInit(&spi, &bad));
	bad = cfg;
	bad.bIntEn = true;
	assert(!SPIInit(&spi, &bad));
	bad = cfg;
	bad.bDmaEn = true;
	assert(!SPIInit(&spi, &bad));
	bad = cfg;
	bad.Phy = SPIPHY_3WIRE;
	assert(!SPIInit(&spi, &bad));
	bad = cfg;
	bad.DataSize = 16;
	assert(!SPIInit(&spi, &bad));
	bad = cfg;
	bad.NbIOPins = 3;
	assert(!SPIInit(&spi, &bad));

	// SPI after I2C on the same Flexcomm changes the function
	assert(SPIInit(&spi, &cfg));
	assert((uint32_t)s_Fc[9].PSELID == 0x72U && !SpiCsLow());
	assert(s_SpiRegs[9].CFG == (SPI_CFG_ENABLE_MASK | SPI_CFG_MASTER_MASK | SPI_CFG_CPHA_MASK | SPI_CFG_CPOL_MASK));
	assert(spi.Cfg.Rate == 1000000 && s_SpiRegs[9].DIV == 11U);
	assert(SPISetRate(&spi, 5000000) == 6000000 && SPISetRate(&spi, 20000000) == 12000000);

	// Loopback receive sends the dummy byte, transmit ignores the RX data
	uint8_t rx[20];
	uint8_t tx[20];
	for (int i = 0; i < 20; i++)
	{
		tx[i] = (uint8_t)i;
	}
	s_SpiLog.clear();
	memset(rx, 0, sizeof(rx));
	assert(SPIRx(&spi, 0, rx, 20) == 20 && !SpiCsLow());
	for (int i = 0; i < 20; i++)
	{
		assert(rx[i] == 0xA5);
	}
	assert((s_SpiLastCtrl & SPI_FIFOWR_RXIGNORE_MASK) == 0 &&
		   (s_SpiLastCtrl & SPI_FIFOWR_LEN_MASK) == SPI_FIFOWR_LEN(7));
	assert((s_SpiLastCtrl & 0xF0000U) == 0xF0000U);

	assert(SPITx(&spi, 0, tx, 20) == 20 && !SpiCsLow());
	assert(s_SpiLastCtrl & SPI_FIFOWR_RXIGNORE_MASK);
	assert(s_SpiRxHead == s_SpiRxTail);

	// Command then read with the chip select held
	uint8_t cmd[3] = {0x9F, 0x12, 0x34};
	s_SpiLog.clear();
	assert(DeviceIntrfRead(&spi.DevIntrf, 0, cmd, 3, rx, 4) == 4 && !SpiCsLow());
	assert(s_SpiLog == "9F1234A5A5A5A5");

	// Chip select out of the pin map
	assert(!SPIStartTx(&spi, 1));

	printf("lpc546xx spi: PASS\n");
}

int main()
{
	I2cTest();

	// Flexcomm 9 as I2C first, then SPI
	I2CDev_t i2c9;
	I2CCfg_t cfg9 = {};
	memset((void *)&i2c9, 0, sizeof(i2c9));
	cfg9.DevNo = 9;
	cfg9.pIOPinMap = s_I2cPins;
	cfg9.NbIOPins = 2;
	cfg9.Rate = 100000;
	assert(I2CInit(&i2c9, &cfg9) && (uint32_t)s_Fc[9].PSELID == 0x73U);

	SpiTest();

	// A Flexcomm locked to another function is not taken
	s_Fc[5].PSELID.value |= FLEXCOMM_PSELID_LOCK_MASK | 1U;
	cfg9.DevNo = 5;
	assert(!I2CInit(&i2c9, &cfg9));

	return 0;
}
