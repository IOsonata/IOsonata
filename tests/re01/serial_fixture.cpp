// RE01 serial regression firmware. Synthetic pin maps are for emulation only.
// The test calls the public C helpers and C++ wrappers, not private hooks.
#include "coredev/spi.h"
#include "coredev/i2c.h"

static SPIDev_t s_Spi[2];
static I2CDev_t s_I2c[2];
static SPIDev_t s_OtherSpi;
static I2CDev_t s_OtherI2c;
static SPICfg_t s_SpiCfg[2];
static I2CCfg_t s_I2cCfg[2];

static const IOPinCfg_t s_SpiPins[2][5] = {
	{{0, 0, 13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 1, 13, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 2, 13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {0, 4, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}},
	{{1, 0, 13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {1, 1, 13, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {1, 2, 13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {1, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	 {1, 4, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}},
};
static const IOPinCfg_t s_I2cPins[2][2] = {
	{{2, 0, 15, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	 {2, 1, 15, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}},
	{{3, 0, 15, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	 {3, 1, 15, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}},
};

extern "C" {
uint8_t SerialTx[130];
uint8_t SerialRx[130];

// Bus 0/1 = SPI0/1, bus 2/3 = I2C0/1. Options are fixture-only flags.
int SerialInit(unsigned Bus, uint32_t Rate, unsigned Options)
{
	if (Bus >= 4)
		return 0;
	unsigned no = Bus & 1;
	if (Bus < 2)
	{
		SPICfg_t cfg = {};
		cfg.DevNo = no;
		cfg.Phy = (Options & 0x1000) ? SPIPHY_3WIRE : SPIPHY_NORMAL;
		cfg.Mode = (Options & 0x800) ? SPIMODE_SLAVE : SPIMODE_MASTER;
		cfg.pIOPinMap = s_SpiPins[no];
		cfg.NbIOPins = 5;
		cfg.Rate = Rate;
		cfg.DataSize = Options & 31;
		cfg.BitOrder = (Options & 0x80) ? SPIDATABIT_LSB : SPIDATABIT_MSB;
		cfg.DataPhase = (Options & 0x20) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
		cfg.ClkPol = (Options & 0x40) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
		cfg.ChipSel = (Options & 0x100) ? SPICSEL_MAN : SPICSEL_AUTO;
		cfg.bIntEn = (Options & 0x200) != 0;
		cfg.bDmaEn = (Options & 0x400) != 0;
		cfg.DummyByte = 0xA5;
		s_SpiCfg[no] = cfg;
		return SPIInit(&s_Spi[no], &s_SpiCfg[no]);
	}
	I2CCfg_t cfg = {};
	cfg.DevNo = no;
	cfg.Type = (Options & 16) ? I2CTYPE_SMBUS : I2CTYPE_STANDARD;
	cfg.Mode = (Options & 8) ? I2CMODE_SLAVE : I2CMODE_MASTER;
	cfg.pIOPinMap = s_I2cPins[no];
	cfg.NbIOPins = 2;
	cfg.Rate = Rate;
	cfg.AddrType = (Options & 4) ? I2CADDR_TYPE_EXT : I2CADDR_TYPE_NORMAL;
	cfg.bIntEn = (Options & 1) != 0;
	cfg.bDmaEn = (Options & 2) != 0;
	s_I2cCfg[no] = cfg;
	return I2CInit(&s_I2c[no], &s_I2cCfg[no]);
}

int SerialCall(unsigned Bus, unsigned Op, uint32_t Arg, unsigned Len)
{
	unsigned no = Bus & 1;
	DevIntrf_t *dev = Bus < 2 ? &s_Spi[no].DevIntrf : &s_I2c[no].DevIntrf;
	switch (Op)
	{
		case 0: return DeviceIntrfTx(dev, Arg, SerialTx + 1, Len);
		case 1: return DeviceIntrfRx(dev, Arg, SerialRx + 1, Len);
		case 2: return DeviceIntrfRead(dev, Arg, SerialTx + 1, Len >> 16,
									 SerialRx + 1, Len & 0xFFFF);
		case 3: return DeviceIntrfWrite(dev, Arg, SerialTx + 1, Len >> 16,
									  SerialTx + 16, Len & 0xFFFF);
		case 4: return DeviceIntrfStartTx(dev, Arg);
		case 5: return DeviceIntrfTxData(dev, SerialTx + 1, Len);
		case 6: DeviceIntrfStopTx(dev); return 0;
		case 7: return DeviceIntrfStartRx(dev, Arg);
		case 8: return DeviceIntrfRxData(dev, SerialRx + 1, Len);
		case 9: DeviceIntrfStopRx(dev); return 0;
		case 10: return dev->SetRate(dev, Arg);
		case 11: return dev->GetRate(dev);
		case 12: DeviceIntrfEnable(dev); return dev->EnCnt;
		case 13: DeviceIntrfDisable(dev); return dev->EnCnt;
		case 14: DeviceIntrfReset(dev); return 0;
		case 15: DeviceIntrfPowerOff(dev); return dev->pDevData == NULL;
		case 16:
		{
			bool busy = atomic_flag_test_and_set(&dev->bBusy);
			if (!busy)
				atomic_flag_clear(&dev->bBusy);
			return busy;
		}
		case 17: return dev->EnCnt;
		case 18: return Bus < 2 ? SPIInit(&s_OtherSpi, &s_SpiCfg[no]) :
								 I2CInit(&s_OtherI2c, &s_I2cCfg[no]);
		case 19: return dev->GetHandle(dev) == (Bus < 2 ? (void*)&s_Spi[no] : (void*)&s_I2c[no]);
		case 20: return Bus < 2 ? SPISetPhy(&s_Spi[no], SPIPHY_3WIRE) == SPIPHY_NORMAL : 0;
		case 21: return dev->StartRx(dev, Arg);
		case 22: return Bus < 2 ? s_Spi[no].FirstRdData : 0;
		case 23:
		{
			if (Bus < 2)
			{
				SPICfg_t cfg = s_SpiCfg[no];
				IOPinCfg_t pins[5];
				memcpy(pins, cfg.pIOPinMap, sizeof(pins));
				pins[1] = pins[0];
				cfg.pIOPinMap = pins;
				return SPIInit(&s_Spi[no], &cfg);
			}
			I2CCfg_t cfg = s_I2cCfg[no];
			IOPinCfg_t pins[2];
			memcpy(pins, cfg.pIOPinMap, sizeof(pins));
			pins[0].Res = (IOPINRES)Arg;
			cfg.pIOPinMap = pins;
			return I2CInit(&s_I2c[no], &cfg);
		}
		case 24:
		{
			if (Bus < 2)
			{
				SPICfg_t cfg = s_SpiCfg[no];
				cfg.DevNo = 1 - no;
				return SPIInit(&s_Spi[no], &cfg);
			}
			I2CCfg_t cfg = s_I2cCfg[no];
			cfg.DevNo = 1 - no;
			return I2CInit(&s_I2c[no], &cfg);
		}
	}
	return -99;
}

int SerialCppProbe(unsigned Bus)
{
	if (Bus == 0)
	{
		SPI spi;
		if (!spi.Init(s_SpiCfg[0]))
			return -1;
		DeviceIntrf &base = spi;
		int n = base.Read(0, SerialTx + 1, 1, SerialRx + 1, 4);
		spi.PowerOff();
		return n;
	}
	I2C i2c;
	if (!i2c.Init(s_I2cCfg[0]))
		return -1;
	DeviceIntrf &base = i2c;
	int n = base.Read(0x50, SerialTx + 1, 1, SerialRx + 1, 4);
	i2c.PowerOff();
	return n;
}

int main(void) { for (;;) {} }
}
