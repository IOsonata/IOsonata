#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <vector>

#include "pwrmgnt/pm_npm1300.h"
#ifdef NPM1300_FUEL_TEST
#include "pwrmgnt/pm_npm1300_fuel.h"
#include "nrf_fuel_gauge.h"
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

struct WriteOp {
	uint16_t Reg;
	uint8_t Data[4];
	int Len;
};

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf()
	{
		ResetState();
	}

	void ResetState()
	{
		memset(vReg, 0, sizeof(vReg));
		vWrites.clear();
		vType = DEVINTRF_TYPE_I2C;
		vFailReadReg = 0xFFFF;
		vFailWriteReg = 0xFFFF;
		vShortReadReg = 0xFFFF;
		vAdcTimeout = false;
		vRate = 100000;
	}

	operator DevIntrf_t * () override
	{
		return NULL;
	}

	DEVINTRF_TYPE Type() override
	{
		return vType;
	}

	uint32_t Rate(uint32_t Rate) override
	{
		vRate = Rate;
		return vRate;
	}

	uint32_t Rate(void) override
	{
		return vRate;
	}

	void Disable(void) override {}
	void Enable(void) override {}
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override
	{
		(void)DevAddr;
		(void)pBuff;
		(void)BuffLen;
		return 0;
	}
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override
	{
		(void)DevAddr;
		(void)pData;
		(void)DataLen;
		return 0;
	}
	bool StartRx(uint32_t DevAddr) override
	{
		(void)DevAddr;
		return false;
	}
	int RxData(uint8_t *pBuff, int BuffLen) override
	{
		(void)pBuff;
		(void)BuffLen;
		return 0;
	}
	void StopRx(void) override {}
	bool StartTx(uint32_t DevAddr) override
	{
		(void)DevAddr;
		return false;
	}
	int TxData(const uint8_t *pData, int DataLen) override
	{
		(void)pData;
		(void)DataLen;
		return 0;
	}
	void StopTx(void) override {}
	bool RequestToSend(int NbBytes) override
	{
		(void)NbBytes;
		return true;
	}
	void Reset(void) override {}

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
		uint8_t *pBuff, int BuffLen) override
	{
		uint16_t reg;
		int len;
		int i;

		(void)DevAddr;
		if (pAdCmd == NULL || AdCmdLen != 2 || pBuff == NULL || BuffLen <= 0)
			return 0;
		reg = ((uint16_t)pAdCmd[0] << 8) | pAdCmd[1];
		if (reg == vFailReadReg)
			return 0;
		len = reg == vShortReadReg ? BuffLen - 1 : BuffLen;
		if (len < 0)
			len = 0;
		for (i = 0; i < len; i++)
			pBuff[i] = vReg[reg + i];
		return len;
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
		const uint8_t *pData, int DataLen) override
	{
		WriteOp op;
		uint16_t reg;
		int i;

		(void)DevAddr;
		if (pAdCmd == NULL || AdCmdLen != 2 || pData == NULL || DataLen <= 0)
			return 0;
		reg = ((uint16_t)pAdCmd[0] << 8) | pAdCmd[1];
		if (reg == vFailWriteReg)
			return 0;

		op.Reg = reg;
		op.Len = DataLen;
		memset(op.Data, 0, sizeof(op.Data));
		for (i = 0; i < DataLen && i < (int)sizeof(op.Data); i++)
			op.Data[i] = pData[i];
		vWrites.push_back(op);

		for (i = 0; i < DataLen; i++)
			vReg[reg + i] = pData[i];

		if (reg == NPM1300_MAIN_EVENTS_ADCCLR_REG)
			vReg[NPM1300_MAIN_EVENTS_ADCSET_REG] &= (uint8_t)~pData[0];
		else if (reg == NPM1300_MAIN_EVENTS_BCHARGER0CLR_REG)
			vReg[NPM1300_MAIN_EVENTS_BCHARGER0SET_REG] &= (uint8_t)~pData[0];
		else if (reg == NPM1300_MAIN_EVENTS_BCHARGER1CLR_REG)
			vReg[NPM1300_MAIN_EVENTS_BCHARGER1SET_REG] &= (uint8_t)~pData[0];
		else if (reg == NPM1300_MAIN_EVENTS_VBUSIN0CLR_REG)
			vReg[NPM1300_MAIN_EVENTS_VBUSIN0SET_REG] &= (uint8_t)~pData[0];
		else if (reg == NPM1300_MAIN_EVENTS_VBUSIN1CLR_REG)
			vReg[NPM1300_MAIN_EVENTS_VBUSIN1SET_REG] &= (uint8_t)~pData[0];

		if (!vAdcTimeout && reg == NPM1300_ADC_TASKVBAT_MEASURE_REG)
			vReg[NPM1300_MAIN_EVENTS_ADCSET_REG] |= NPM1300_MAIN_EVENTS_ADCSET_EVENTADCVBATRDY;
		else if (!vAdcTimeout && reg == NPM1300_ADC_TASKIBAT_MEASURE_REG)
			vReg[NPM1300_MAIN_EVENTS_ADCSET_REG] |= NPM1300_MAIN_EVENTS_ADCSET_EVENTADCIBATRDY;
		else if (!vAdcTimeout && reg == NPM1300_ADC_TASKNTC_MEASURE_REG)
			vReg[NPM1300_MAIN_EVENTS_ADCSET_REG] |= NPM1300_MAIN_EVENTS_ADCSET_EVENTADCNTCRDY;

		return DataLen;
	}

	void SetAdcVbat(uint16_t Code)
	{
		vReg[NPM1300_ADC_ADCVBAT_RESULTMSB_REG] = (uint8_t)(Code >> 2);
		vReg[NPM1300_ADC_ADCGP0_RESULTLSBS_REG] &= (uint8_t)~3U;
		vReg[NPM1300_ADC_ADCGP0_RESULTLSBS_REG] |= (uint8_t)(Code & 3U);
	}

	void SetAdcNtc(uint16_t Code)
	{
		vReg[NPM1300_ADC_ADCNTC_RESULTMSB_REG] = (uint8_t)(Code >> 2);
		vReg[NPM1300_ADC_ADCGP0_RESULTLSBS_REG] &= (uint8_t)~(3U << 2);
		vReg[NPM1300_ADC_ADCGP0_RESULTLSBS_REG] |= (uint8_t)((Code & 3U) << 2);
	}

	int WriteIndex(uint16_t Reg) const
	{
		size_t i;

		for (i = 0; i < vWrites.size(); i++)
		{
			if (vWrites[i].Reg == Reg)
				return (int)i;
		}
		return -1;
	}

	int WriteCount(uint16_t Reg) const
	{
		int count;
		size_t i;

		count = 0;
		for (i = 0; i < vWrites.size(); i++)
		{
			if (vWrites[i].Reg == Reg)
				count++;
		}
		return count;
	}

	uint8_t vReg[0xE20];
	std::vector<WriteOp> vWrites;
	DEVINTRF_TYPE vType;
	uint16_t vFailReadReg;
	uint16_t vFailWriteReg;
	uint16_t vShortReadReg;
	bool vAdcTimeout;
	uint32_t vRate;
};

static PWRMGNT_EVT s_Events[16];
static int s_EventCount;

static void EventHandler(PowerMgnt *pSelf, PWRMGNT_EVT Evt)
{
	(void)pSelf;
	assert(s_EventCount < (int)ARRAY_SIZE(s_Events));
	s_Events[s_EventCount++] = Evt;
}

static BatProfile_t BatteryProfile(uint32_t Beta, uint32_t ThermKohm)
{
	BatProfile_t profile;

	profile.OpVolt = 3700;
	profile.ChrgVolt = 4200;
	profile.Capacity = 1000;
	profile.ThermBetaConst = Beta;
	profile.ThermResistor = ThermKohm;
	return profile;
}

static PwrMgntCfg_t PowerConfig(const BatProfile_t *pProfile,
	const PwrMgntVoutCfg_t *pVout = NULL, size_t NbVout = 0, uint32_t ChargeCurrent = 400,
	uint32_t ChargeTimeout = 0, bool Interrupts = false, PwrMgntEvtHandler_t Handler = NULL)
{
	PwrMgntCfg_t cfg = {
		NPM1300_I2C_7BITS_ADDR,
		pVout,
		NbVout,
		4200,
		ChargeCurrent,
		ChargeTimeout,
		pProfile,
		Interrupts,
		0,
		0,
		0,
		NULL,
		0,
		Handler
	};

	return cfg;
}

static void Attach(PmnPM1300 &pm, MockIntrf &bus)
{
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 0;
	bus.vReg[NPM1300_LDSW_LSDWSTATUS_REG] = 0;
	bus.vReg[NPM1300_BUCK_BUCK1VOUT_STATUS_REG] = 0;
	bus.vReg[NPM1300_BUCK_BUCK2VOUT_STATUS_REG] = 0;
	assert(pm.Init(&bus));
	assert(pm.Valid());
}

static void TestInit()
{
	MockIntrf bus;
	PmnPM1300 pm;
	BatProfile_t profile;
	BatProfile_t badProfile;
	PwrMgntVoutCfg_t vout[2];

	assert(!pm.Init(NULL));
	assert(!pm.Valid());
	bus.vType = DEVINTRF_TYPE_UNKOWN;
	assert(!pm.Init(&bus));
	bus.vType = DEVINTRF_TYPE_I2C;
	assert(!pm.Init(&bus, 0x6A));

	bus.ResetState();
	Attach(pm, bus);
	assert(bus.vWrites.empty());

	bus.ResetState();
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 0x80;
	assert(!pm.Init(&bus));
	assert(!pm.Valid());

	profile = BatteryProfile(3435, 10);
	badProfile = profile;
	badProfile.ThermResistor = 12;
	vout[0] = { 1800, 0 };
	vout[1] = { 3300, 0 };
	PwrMgntCfg_t cfgTimeout = PowerConfig(&profile, NULL, 0, 400, 1);
	PwrMgntCfg_t cfgTooMany = PowerConfig(&profile, vout, 3);
	PwrMgntCfg_t cfgNullVout = PowerConfig(&profile, NULL, 1);
	PwrMgntCfg_t cfgBadTherm = PowerConfig(&badProfile);
	PwrMgntCfg_t cfgGood = PowerConfig(&profile, vout, 2);
	assert(!pm.Init(cfgTimeout, &bus));
	assert(!pm.Init(cfgTooMany, &bus));
	assert(!pm.Init(cfgNullVout, &bus));
	assert(!pm.Init(cfgBadTherm, &bus));

	bus.ResetState();
	assert(pm.Init(cfgGood, &bus));
	assert(pm.Valid());

	bus.ResetState();
	bus.vFailWriteReg = NPM1300_ADC_ADCNTCRSEL_REG;
	assert(!pm.Init(cfgGood, &bus));
	assert(!pm.Valid());
	assert(!pm.SetLdoVoltage(0, 1800));
}

static void TestBuckLdoGpio()
{
	MockIntrf bus;
	PmnPM1300 pm;
	uint8_t voltage;
	uint8_t status;
	bool high;

	Attach(pm, bus);
	assert(pm.SetVout(0, 1000, 0) == 1000);
	assert(bus.vReg[NPM1300_BUCK_BUCK1NORM_VOUT_REG] == 0);
	assert((bus.vReg[NPM1300_BUCK_BUCKSWCTRLSEL_REG] & 1U) != 0);
	assert(pm.SetVout(1, 3300, 0) == 3300);
	assert(bus.vReg[NPM1300_BUCK_BUCK2NORM_VOUT_REG] == 23);
	assert((bus.vReg[NPM1300_BUCK_BUCKSWCTRLSEL_REG] & 2U) != 0);
	assert(pm.SetVout(0, 0, 0) == 0);
	assert(pm.SetVout(0, 950, 0) < 0);
	assert(pm.SetVout(0, 1050, 0) < 0);
	assert(pm.SetVout(2, 1800, 0) < 0);

	assert(pm.SetBuckRetention(0, 1000));
	assert(pm.SetBuckRetention(1, 3300));
	assert(bus.vReg[0x40B] == 23);
	assert(!pm.SetBuckRetention(0, 1050));
	assert(pm.SetBuckPwm(1, true));
	assert(bus.WriteCount(0x406) == 1);
	assert(pm.SetBuckPwm(1, false));
	assert(bus.WriteCount(0x407) == 1);
	assert(pm.SetBuckPfm(0, true));
	assert((bus.vReg[NPM1300_BUCK_BUCKCTRL0_REG] & 1U) != 0);
	assert(pm.SetBuckPfm(0, false));
	assert((bus.vReg[NPM1300_BUCK_BUCKCTRL0_REG] & 1U) == 0);
	assert(pm.SetBuckDischarge(1, true));
	assert((bus.vReg[NPM1300_BUCK_BUCKCTRL0_REG] & (1U << 3)) != 0);
	assert(pm.SetBuckDischarge(1, false));
	assert((bus.vReg[NPM1300_BUCK_BUCKCTRL0_REG] & (1U << 3)) == 0);

	assert(pm.SetBuckEnableGpio(0, 2, false));
	assert(bus.vReg[NPM1300_BUCK_BUCKENCTRL_REG] == 0x03);
	assert(pm.SetBuckEnableGpio(1, 4, true));
	assert(bus.vReg[NPM1300_BUCK_BUCKENCTRL_REG] == 0xAB);
	assert(pm.SetBuckEnableGpio(0, -1, true));
	assert(bus.vReg[NPM1300_BUCK_BUCKENCTRL_REG] == 0xA8);
	assert(!pm.SetBuckEnableGpio(2, 0, false));
	assert(!pm.SetBuckEnableGpio(0, -2, false));
	assert(!pm.SetBuckEnableGpio(0, 5, false));

	assert(pm.SetBuckRetentionGpio(0, 1, true));
	assert(bus.vReg[NPM1300_BUCK_BUCKVRETCTRL_REG] == 0x42);
	assert(pm.SetBuckPwmGpio(1, 0, false));
	assert(bus.vReg[NPM1300_BUCK_BUCKPWMCTRL_REG] == 0x08);
	bus.vFailReadReg = NPM1300_BUCK_BUCKPWMCTRL_REG;
	assert(!pm.SetBuckPwmGpio(1, 1, false));
	bus.vFailReadReg = 0xFFFF;

	bus.vReg[NPM1300_BUCK_BUCK1VOUT_STATUS_REG] = 8;
	bus.vReg[NPM1300_BUCK_BUCKSTATUS_REG] = 0x0C;
	assert(pm.ReadBuckStatus(0, voltage, status));
	assert(voltage == 8 && status == 0x0C);

	assert(pm.SetLdoVoltage(0, 1000));
	assert(bus.vReg[NPM1300_LDSW_LDSW1VOUTSEL_REG] == 0);
	assert(bus.vReg[NPM1300_LDSW_LDSW1LDOSEL_REG] == 1);
	assert(pm.SetLdoVoltage(1, 3300));
	assert(bus.vReg[NPM1300_LDSW_LDSW2VOUTSEL_REG] == 23);
	assert(!pm.SetLdoVoltage(0, 1050));
	assert(pm.SetLoadSwitchMode(0));
	assert(bus.vReg[NPM1300_LDSW_LDSW1LDOSEL_REG] == 0);
	assert(pm.SetLoadSwitchEnabled(0, true));
	assert(pm.SetLoadSwitchEnabled(1, false));
	assert(bus.WriteCount(NPM1300_LDSW_TASKLDSW1_SET_REG) == 1);
	assert(bus.WriteCount(NPM1300_LDSW_TASKLDSW2_CLR_REG) == 1);

	assert(pm.SetLoadSwitchEnableGpio(0, 3, true));
	assert(bus.vReg[NPM1300_LDSW_LDSW1GPISEL_REG] == 0x0C);
	assert(pm.SetLoadSwitchEnableGpio(1, 4, false));
	assert(bus.vReg[NPM1300_LDSW_LDSW2GPISEL_REG] == 0x05);
	assert(pm.SetLoadSwitchEnableGpio(0, -1, true));
	assert(bus.vReg[NPM1300_LDSW_LDSW1GPISEL_REG] == 0);
	assert(!pm.SetLoadSwitchEnableGpio(2, 0, false));
	assert(!pm.SetLoadSwitchEnableGpio(0, -2, false));
	assert(!pm.SetLoadSwitchEnableGpio(0, 5, false));
	bus.vFailWriteReg = NPM1300_LDSW_LDSW2GPISEL_REG;
	assert(!pm.SetLoadSwitchEnableGpio(1, 1, false));
	bus.vFailWriteReg = 0xFFFF;

	bus.vReg[NPM1300_LDSW_LDSWCONFIG_REG] = 0x80;
	assert(pm.SetLoadSwitchSoftStart(0, 25, true));
	assert((bus.vReg[NPM1300_LDSW_LDSWCONFIG_REG] & 0x0D) == 0);
	assert(pm.SetLoadSwitchSoftStart(1, 75, true));
	assert((bus.vReg[NPM1300_LDSW_LDSWCONFIG_REG] & 0x30) == 0x20);
	assert(pm.SetLoadSwitchSoftStart(1, 100, false));
	assert((bus.vReg[NPM1300_LDSW_LDSWCONFIG_REG] & 0x32) == 0x32);
	assert(!pm.SetLoadSwitchSoftStart(0, 10, true));
	assert(pm.SetLoadSwitchDischarge(0, true));
	assert((bus.vReg[NPM1300_LDSW_LDSWCONFIG_REG] & 0x40) != 0);
	assert(pm.ReadLoadSwitchStatus(status));

	bus.vReg[NPM1300_GPIOS_GPIOPUEN_BASE_REG + 4] = 1;
	bus.vReg[NPM1300_GPIOS_GPIOPDEN_BASE_REG + 4] = 1;
	assert(pm.SetGpioMode(4, 9));
	assert(bus.vReg[NPM1300_GPIOS_GPIOMODE_BASE_REG + 4] == 9);
	assert(bus.vReg[NPM1300_GPIOS_GPIOPUEN_BASE_REG + 4] == 0);
	assert(bus.vReg[NPM1300_GPIOS_GPIOPDEN_BASE_REG + 4] == 0);
	assert(!pm.SetGpioMode(5, 0));
	assert(pm.SetGpioPull(2, true, false));
	assert(bus.vReg[NPM1300_GPIOS_GPIOPUEN_BASE_REG + 2] == 1);
	assert(bus.vReg[NPM1300_GPIOS_GPIOPDEN_BASE_REG + 2] == 0);
	assert(!pm.SetGpioPull(2, true, true));
	assert(pm.SetGpioOpenDrain(3, true));
	assert(pm.SetGpioDrive(3, 6));
	assert(!pm.SetGpioDrive(3, 2));
	bus.vReg[NPM1300_GPIOS_GPIOSTATUS_REG] = 1U << 4;
	assert(pm.GpioState(4, high));
	assert(high);
}

static void TestCharger()
{
	MockIntrf bus;
	PmnPM1300 pm;
	int disableIndex;
	int msbIndex;
	int enableIndex;

	Attach(pm, bus);
	bus.vWrites.clear();
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 32) == 32);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_MSB_REG] == 8);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_LSB_REG] == 0);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGVTERM_REG] == 8);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGVTERMR_REG] == 8);
	disableIndex = bus.WriteIndex(NPM1300_BCHARGER_BCHGENABLE_CLR_REG);
	msbIndex = bus.WriteIndex(NPM1300_BCHARGER_BCHGISET_MSB_REG);
	enableIndex = bus.WriteIndex(NPM1300_BCHARGER_BCHGENABLE_SET_REG);
	assert(disableIndex >= 0 && disableIndex < msbIndex && msbIndex < enableIndex);

	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_AUTO, 4450, 34) == 34);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_MSB_REG] == 8);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_LSB_REG] == 1);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGVTERM_REG] == 13);
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 3500, 800) == 800);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_MSB_REG] == 200);
	assert(!pm.SetCharge(PWRMGNT_CHARGE_TYPE_TRICKLE, 4200, 100));
	assert(!pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 3750, 100));
	assert(!pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 31));
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 33) == 32);
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 675) == 674);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_MSB_REG] == 168);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_LSB_REG] == 1);
	assert(!pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 802));

	bus.vWrites.clear();
	bus.vFailWriteReg = NPM1300_BCHARGER_BCHGISET_LSB_REG;
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 34) == 0);
	assert(bus.WriteIndex(NPM1300_BCHARGER_BCHGENABLE_CLR_REG) >= 0);
	assert(bus.WriteIndex(NPM1300_BCHARGER_BCHGENABLE_SET_REG) < 0);
	bus.vFailWriteReg = 0xFFFF;

	assert(pm.SetWarmTerminationVoltage(4100));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGVTERMR_REG] == 6);
	assert(!pm.SetWarmTerminationVoltage(3900));
	assert(pm.SetTerminationCurrent(10));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGITERM_SEL_REG] == 0);
	assert(pm.SetTerminationCurrent(20));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGITERM_SEL_REG] == 1);
	assert(!pm.SetTerminationCurrent(15));
	assert(pm.SetTrickleVoltage(2500));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGV_TRICKLE_SEL_REG] == 1);
	assert(pm.SetTrickleVoltage(2900));
	assert(!pm.SetTrickleVoltage(2700));
	assert(pm.SetAutomaticRecharge(false));
	assert(bus.vWrites.back().Reg == NPM1300_BCHARGER_BCHGDISABLE_SET_REG);
	assert(pm.SetAutomaticRecharge(true));
	assert(bus.vWrites.back().Reg == NPM1300_BCHARGER_BCHGDISABLE_CLR_REG);
	assert(pm.SetNtcProtection(false));
	assert(bus.vWrites.back().Data[0] == 2);

	assert(pm.SetDischargeCurrentLimit(200));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_DISCHARGE_MSB_REG] == 42);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_DISCHARGE_LSB_REG] == 0);
	assert(pm.SetDischargeCurrentLimit(1000));
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_DISCHARGE_MSB_REG] == 207);
	assert(bus.vReg[NPM1300_BCHARGER_BCHGISET_DISCHARGE_LSB_REG] == 1);
	assert(!pm.SetDischargeCurrentLimit(500));

	assert(pm.SetInputCurrentLimit(100));
	assert(bus.vReg[NPM1300_VBUSIN_VBUSIN_ILIM0_REG] == 1);
	assert(pm.SetInputCurrentLimit(500));
	assert(bus.vReg[NPM1300_VBUSIN_VBUSIN_ILIM0_REG] == 5);
	assert(pm.SetInputCurrentLimit(1500));
	assert(bus.vReg[NPM1300_VBUSIN_VBUSIN_ILIM0_REG] == 15);
	assert(!pm.SetInputCurrentLimit(400));
	assert(!pm.SetInputCurrentLimit(1550));

	assert(pm.ClearChargerError());
	assert(pm.RecoverCharger());
	assert(bus.WriteIndex(NPM1300_BCHARGER_TASK_CLEAR_CHGERR_REG) >= 0);
	assert(bus.WriteIndex(NPM1300_BCHARGER_TASK_RELEASE_ERR_REG) >= 0);
	assert(pm.ClearChargerSafetyTimers());
	assert(bus.WriteIndex(NPM1300_BCHARGER_TASK_CLEAR_SAFTY_TIMER_REG) >= 0);

	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 1U << 5;
	assert(!pm.Charging());
	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 1U << 3;
	assert(pm.Charging());
	assert(pm.BatteryState() == PWRMGNT_BATTERY_UNKNOWN);
	assert(!pm.Battery());
	assert(pm.Level() == UINT16_MAX);
}

static void TestThermalConfiguration()
{
	MockIntrf bus;
	PmnPM1300 pm;
	BatProfile_t profile;
	profile = BatteryProfile(3435, 10);
	FuelGaugeCfg_t realCfg = { NPM1300_I2C_7BITS_ADDR, profile, NULL };
	assert(pm.Init(realCfg, &bus, NULL));
	assert(pm.SetNtcTemperatureLimits(0, 10, 45, 55));
	assert(bus.WriteCount(NPM1300_BCHARGER_NTCCOLD_REG) == 1);
	assert(bus.WriteCount(NPM1300_BCHARGER_NTCCOOL_REG) == 1);
	assert(bus.WriteCount(NPM1300_BCHARGER_NTCWARM_REG) == 1);
	assert(bus.WriteCount(NPM1300_BCHARGER_NTCHOT_REG) == 1);
	assert(!pm.SetNtcTemperatureLimits(20, 10, 45, 55));
	assert(!pm.SetNtcTemperatureLimits(-30, 10, 45, 55));
	assert(pm.SetDieTemperatureLimits(100, 80));
	assert(bus.WriteCount(NPM1300_BCHARGER_DIE_TEMP_STOP_REG) == 1);
	assert(bus.WriteCount(NPM1300_BCHARGER_DIE_TEMP_RESUME_REG) == 1);
	assert(!pm.SetDieTemperatureLimits(80, 100));
	assert(pm.SetWarmCharging(false));
	assert((bus.vReg[NPM1300_BCHARGER_BCHG_CONFIG_REG] & 1U) != 0);
	assert(pm.SetWarmCharging(true));
	assert((bus.vReg[NPM1300_BCHARGER_BCHG_CONFIG_REG] & 1U) == 0);
	assert(pm.SetCoolFullCurrent(true));
	assert(bus.vWrites.back().Reg == NPM1300_BCHARGER_BCHGENABLE_SET_REG);
	assert(bus.vWrites.back().Data[0] == 2);
	assert(pm.SetCoolFullCurrent(false));
	assert(bus.vWrites.back().Reg == NPM1300_BCHARGER_BCHGENABLE_CLR_REG);

}

static void TestAdc()
{
	MockIntrf bus;
	PmnPM1300 pm;
	BatProfile_t profile;
	int32_t temp;

	Attach(pm, bus);
	bus.SetAdcVbat(0);
	assert(pm.Voltage() == 0);
	bus.SetAdcVbat(1023);
	assert(pm.Voltage() == 4995);
	bus.SetAdcVbat(512);
	assert(pm.Voltage() == (512 * 5000) / 1024);

	bus.vAdcTimeout = true;
	assert(pm.Voltage() == -1);
	bus.vAdcTimeout = false;
	bus.vShortReadReg = NPM1300_ADC_ADCVBAT_RESULTMSB_REG;
	assert(pm.Voltage() == -1);
	bus.vShortReadReg = 0xFFFF;

	profile = BatteryProfile(3435, 10);
	FuelGaugeCfg_t fgCfg = { NPM1300_I2C_7BITS_ADDR, profile, NULL };
	assert(pm.Init(fgCfg, &bus, NULL));
	bus.SetAdcNtc(512);
	temp = pm.Temperature();
	assert(temp >= 245 && temp <= 255);
	bus.SetAdcNtc(0);
	assert(pm.Temperature() == INT32_MIN);
	bus.SetAdcNtc(1023);
	assert(pm.Temperature() == INT32_MIN);
	bus.vAdcTimeout = true;
	assert(pm.Temperature() == INT32_MIN);

}

static void TestInterruptsAndStatus()
{
	MockIntrf bus;
	PmnPM1300 pm;
	BatProfile_t profile;
	uint8_t charger;
	uint8_t errors;
	uint8_t vbus;
	uint8_t ntc;
	uint8_t reset;
	uint8_t sensor;

	profile = BatteryProfile(3435, 10);
	PwrMgntCfg_t cfg = PowerConfig(&profile, NULL, 0, 0, 0, true, EventHandler);
	assert(pm.Init(cfg, &bus));
	assert(bus.WriteCount(NPM1300_GPIOS_GPIOMODE_BASE_REG + 3) == 0);
	assert(pm.SetGpioMode(3, NPM1300_GPIOS_GPIOMODE_GPOIRQ));
	assert(bus.vReg[(NPM1300_GPIOS_GPIOMODE_BASE_REG + 3)] == NPM1300_GPIOS_GPIOMODE_GPOIRQ);
	assert(bus.vReg[(NPM1300_GPIOS_GPIOPUEN_BASE_REG + 3)] == 0);
	assert(bus.vReg[(NPM1300_GPIOS_GPIOPDEN_BASE_REG + 3)] == 0);
	assert(bus.vReg[NPM1300_MAIN_INTENEVENT_SBCHARGER0SET_REG] == 0x1C);
	assert(bus.vReg[NPM1300_MAIN_INTENEVENTS_BCHARGER1SET_REG] == 0x10);
	assert(bus.vReg[0x010] == 0);
	assert(bus.vReg[NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_REG] == 0x01);
	assert(bus.vReg[NPM1300_MAIN_INTENEVENTS_VBUSIN1SET_REG] == 0x05);
	assert(pm.SetInputCurrentLimit(900));

	s_EventCount = 0;
	bus.vReg[NPM1300_MAIN_EVENTS_BCHARGER0SET_REG] = 0x05;
	pm.IrqHandler();
	assert(s_EventCount == 1 && s_Events[0] == PWRMGNT_EVT_OVER_HEAT);
	assert(bus.vReg[NPM1300_MAIN_EVENTS_BCHARGER0SET_REG] == 0x01);

	bus.vReg[NPM1300_MAIN_EVENTS_BCHARGER1SET_REG] = 0x10;
	pm.IrqHandler();
	assert(s_EventCount == 2 && s_Events[1] == PWRMGNT_EVT_CHARGE_FULL);

	bus.vWrites.clear();
	bus.vReg[NPM1300_MAIN_EVENTS_VBUSIN0SET_REG] = 0x01;
	pm.IrqHandler();
	assert(s_EventCount == 3 && s_Events[2] == PWRMGNT_EVT_CHARGE_DETECTED);
	assert(bus.WriteIndex(NPM1300_VBUSIN_TASKUPDATE_ILIMSW_REG) >= 0);

	bus.vReg[NPM1300_MAIN_EVENTS_VBUSIN1SET_REG] = 0x04;
	pm.IrqHandler();
	assert(s_EventCount == 4 && s_Events[3] == PWRMGNT_EVT_OVER_HEAT);

	bus.vReg[0x00E] = 0x07;
	pm.IrqHandler();
	assert(s_EventCount == 4);
	assert(bus.vReg[0x00E] == 0x07);

	bus.vReg[NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG] = 0x12;
	bus.vReg[NPM1300_BCHARGER_BCHG_ERR_REASON_REG] = 0x23;
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 0x01;
	bus.vReg[NPM1300_BCHARGER_NTC_STATUS_REG] = 0x04;
	bus.vReg[NPM1300_ERRLOG_RSTCAUSE_REG] = 0x40;
	assert(pm.ReadStatus(charger, errors, vbus, ntc, reset));
	assert(charger == 0x12 && errors == 0x23 && vbus == 0x01 && ntc == 0x04 && reset == 0x40);
	bus.vReg[NPM1300_BCHARGER_BCHG_ERR_SENSOR_REG] = 0xA5;
	assert(pm.ReadChargerError(errors, sensor));
	assert(errors == 0x23 && sensor == 0xA5);
	bus.vShortReadReg = NPM1300_BCHARGER_NTC_STATUS_REG;
	assert(!pm.ReadStatus(charger, errors, vbus, ntc, reset));
	bus.vShortReadReg = 0xFFFF;
	assert(pm.ClearErrorLog());
}

static void TestPowerControl()
{
	MockIntrf bus;
	PmnPM1300 pm;

	Attach(pm, bus);
	bus.vWrites.clear();
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINPRESENT_DETECTED;
	assert(!pm.EnterShipMode());
	assert(bus.WriteIndex(NPM1300_SHIP_TASKENTER_SHIPMODE_REG) < 0);
	bus.vReg[NPM1300_VBUSIN_VBUSIN_STATUS_REG] = 0;
	assert(pm.EnterShipMode());
	assert(bus.WriteIndex(NPM1300_SHIP_TASKENTER_SHIPMODE_REG) >= 0);
	assert(!pm.Valid());

	bus.ResetState();
	Attach(pm, bus);
	bus.vWrites.clear();
	pm.Reset();
	assert(bus.WriteIndex(NPM1300_MAIN_TASKSW_RESET_REG) >= 0);
	assert(bus.WriteIndex(0xB03) < 0);
	assert(!pm.Valid());
}


static void TestCurrent()
{
	MockIntrf bus;
	PmnPM1300 pm;
	int32_t expected;

	Attach(pm, bus);
	assert(pm.Current() == INT32_MIN);
	bus.vReg[0x510] = 4;
	bus.vReg[0x30A] = 207;
	bus.vReg[0x30B] = 1;
	bus.vReg[0x518] = 128;
	bus.vReg[0x51A] = 0;
	expected = 512 * 1120000 / 1023;
	assert(pm.Current() == expected);
	bus.vReg[0x30A] = 42;
	bus.vReg[0x30B] = 0;
	assert(pm.Current() == 512 * 224000 / 1023);
	bus.vReg[0x518] = 255;
	bus.vReg[0x51A] = 0x30;
	assert(pm.Current() == 224000);
	assert(pm.SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, 4200, 800) == 800);
	bus.vReg[0x510] = 0x0F;
	assert(pm.Current() == -1000000);
	bus.vReg[0x510] = 0x0D;
	assert(pm.Current() == -1000000);
	bus.vReg[0x510] = 0x0C;
	assert(pm.Current() == -1000000);
	bus.vReg[0x510] = 0x1F;
	assert(pm.Current() == INT32_MIN);
	bus.vReg[0x510] = 0x0F;
	bus.vAdcTimeout = true;
	assert(pm.Current() == INT32_MIN);
	bus.vAdcTimeout = false;
	bus.vShortReadReg = 0x518;
	assert(pm.Current() == INT32_MIN);
	bus.vShortReadReg = 0xFFFF;
	bus.vReg[0x509] = 2;
	assert(pm.Current() == INT32_MIN);
}

#ifdef NPM1300_FUEL_TEST
#include "npm1300_fuel_test.inc"
#endif

int main()
{
	static_assert(NPM1300_BUCK_BUCKSTATUS_REG == 0x434, "current BUCKSTATUS address");
	static_assert(NPM1300_BCHARGER_BCHGITERM_SEL_BCHGITERMSEL_SEL20 == 1, "20 percent termination code");

	TestInit();
	TestBuckLdoGpio();
	TestCharger();
	TestThermalConfiguration();
	TestAdc();
	TestInterruptsAndStatus();
	TestPowerControl();
	TestCurrent();
#ifdef NPM1300_FUEL_TEST
	TestFuelGauge();
#endif
	puts("nPM1300 production-driver tests: PASS");
	return 0;
}
