/**-------------------------------------------------------------------------
@file	pm_npm1300.cpp

@brief	Power management implementation of the Nordic nPM1300

@author	Hoang Nguyen Hoan
@date	Apr. 10, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "convutil.h"
#include "pwrmgnt/pm_npm1300.h"

#define NPM1300_ADC_POLL_COUNT		64
#define NPM1300_VBUS_STATUS_VALID_MASK	0x3FU
#define NPM1300_LDSW_STATUS_VALID_MASK	0x1FU

#define NPM1300_IRQ_BCHARGER0_MASK	(NPM1300_MAIN_EVENTS_BCHARGER0SET_EVENTNTCWARM | \
						 NPM1300_MAIN_EVENTS_BCHARGER0SET_EVENTNTCHOT | \
						 NPM1300_MAIN_EVENTS_BCHARGER0SET_EVENTDIETEMPHIGH)
#define NPM1300_IRQ_BCHARGER1_MASK	NPM1300_MAIN_EVENTS_BCHARGER1SET_EVENTCHGCOMPLETED
#define NPM1300_IRQ_VBUSIN0_MASK	NPM1300_MAIN_EVENTS_VBUSIN0SET_EVENTVBUSDETECTED
#define NPM1300_IRQ_VBUSIN1_MASK	(NPM1300_MAIN_EVENTS_VBUSIN1SET_EVENTTHERMALWARNDETECTED | \
						 NPM1300_MAIN_EVENTS_VBUSIN1SET_EVENTTHERMALSHUTDOWNDETECTED)

typedef struct {
	uint16_t EventReg;
	uint16_t EventClrReg;
	uint16_t IntSetReg;
	uint16_t IntClrReg;
	uint8_t Mask;
} Npm1300IrqGroup_t;

static const Npm1300IrqGroup_t s_Npm1300IrqGroups[] = {
	{ NPM1300_MAIN_EVENTS_BCHARGER0SET_REG,
		NPM1300_MAIN_EVENTS_BCHARGER0CLR_REG,
		NPM1300_MAIN_INTENEVENT_SBCHARGER0SET_REG,
		NPM1300_MAIN_INTENEVENT_SBCHARGER0CLR_REG,
		NPM1300_IRQ_BCHARGER0_MASK },
	{ NPM1300_MAIN_EVENTS_BCHARGER1SET_REG,
		NPM1300_MAIN_EVENTS_BCHARGER1CLR_REG,
		NPM1300_MAIN_INTENEVENTS_BCHARGER1SET_REG,
		NPM1300_MAIN_INTENEVENTS_BCHARGER1CLR_REG,
		NPM1300_IRQ_BCHARGER1_MASK },
	{ NPM1300_MAIN_EVENTS_VBUSIN0SET_REG,
		NPM1300_MAIN_EVENTS_VBUSIN0CLR_REG,
		NPM1300_MAIN_INTENEVENTS_VBUSIN0SET_REG,
		NPM1300_MAIN_INTENEVENTS_VBUSIN0CLR_REG,
		NPM1300_IRQ_VBUSIN0_MASK },
	{ NPM1300_MAIN_EVENTS_VBUSIN1SET_REG,
		NPM1300_MAIN_EVENTS_VBUSIN1CLR_REG,
		NPM1300_MAIN_INTENEVENTS_VBUSIN1SET_REG,
		NPM1300_MAIN_INTENEVENTS_VBUSIN1CLR_REG,
		NPM1300_IRQ_VBUSIN1_MASK },
};

PmnPM1300::PmnPM1300()
{
	vChrgCurr = 0;
	vpEvtHandler = NULL;
	vBatLowHandler = NULL;
	vpPwrMgnt = NULL;
	vBatProfile.OpVolt = 0;
	vBatProfile.ChrgVolt = 0;
	vBatProfile.Capacity = 0;
	vBatProfile.ThermBetaConst = 0;
	vBatProfile.ThermResistor = 0;
	vNtcBeta = 0;
	vInputCurrentLimit = 0;
}

void PmnPM1300::Invalidate()
{
	Valid(false);
	InterruptEnabled(false);
	Interface(NULL);
	vDevAddr = 0;
	vChrgCurr = 0;
	vpEvtHandler = NULL;
	vBatLowHandler = NULL;
	vpPwrMgnt = NULL;
	vBatProfile.OpVolt = 0;
	vBatProfile.ChrgVolt = 0;
	vBatProfile.Capacity = 0;
	vBatProfile.ThermBetaConst = 0;
	vBatProfile.ThermResistor = 0;
	vNtcBeta = 0;
	vInputCurrentLimit = 0;
}

bool PmnPM1300::ReadReg8(uint16_t Reg, uint8_t &Value)
{
	uint16_t addr;

	if (Interface() == NULL)
		return false;

	addr = EndianCvt16(Reg);
	return Read((uint8_t *)&addr, 2, &Value, 1) == 1;
}

bool PmnPM1300::WriteReg8(uint16_t Reg, uint8_t Value)
{
	uint16_t addr;

	if (Interface() == NULL)
		return false;

	addr = EndianCvt16(Reg);
	return Write8((uint8_t *)&addr, 2, Value);
}

bool PmnPM1300::UpdateReg8(uint16_t Reg, uint8_t Mask, uint8_t Value)
{
	uint8_t data;

	if (!ReadReg8(Reg, data))
		return false;

	data = (uint8_t)((data & ~Mask) | (Value & Mask));
	return WriteReg8(Reg, data);
}

bool PmnPM1300::WriteReg10(uint16_t Reg, uint16_t Value)
{
	uint16_t addr;
	uint8_t data[2];

	if (Interface() == NULL || Value > 1023)
		return false;

	addr = EndianCvt16(Reg);
	data[0] = (uint8_t)(Value >> 2);
	data[1] = (uint8_t)(Value & 3U);
	return Write((uint8_t *)&addr, 2, data, 2) == 2;
}

bool PmnPM1300::Probe()
{
	uint8_t vbus;
	uint8_t ldsw;
	uint8_t buck1;
	uint8_t buck2;

	if (!ReadReg8(NPM1300_VBUSIN_VBUSIN_STATUS_REG, vbus))
		return false;
	if (!ReadReg8(NPM1300_LDSW_LSDWSTATUS_REG, ldsw))
		return false;
	if (!ReadReg8(NPM1300_BUCK_BUCK1VOUT_STATUS_REG, buck1))
		return false;
	if (!ReadReg8(NPM1300_BUCK_BUCK2VOUT_STATUS_REG, buck2))
		return false;

	if ((vbus & ~NPM1300_VBUS_STATUS_VALID_MASK) != 0)
		return false;
	if ((ldsw & ~NPM1300_LDSW_STATUS_VALID_MASK) != 0)
		return false;
	if (buck1 > NPM1300_BUCK_VOUT_CODE_MAX || buck2 > NPM1300_BUCK_VOUT_CODE_MAX)
		return false;

	return true;
}

bool PmnPM1300::Init(DeviceIntrf * const pIntrf, uint32_t DevAddr)
{
	Invalidate();

	if (pIntrf == NULL || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		DevAddr != NPM1300_I2C_7BITS_ADDR)
		return false;

	vDevAddr = DevAddr;
	Interface(pIntrf);

	if (!Probe())
	{
		Invalidate();
		return false;
	}

	Valid(true);
	return true;
}

bool PmnPM1300::SetLdoVoltage(size_t Index, int32_t mVolt)
{
	uint16_t reg;
	uint8_t code;

	if (Index > 1 || mVolt < 1000 || mVolt > 3300 ||
		(mVolt - 1000) % 100 != 0 || Interface() == NULL)
		return false;

	code = (uint8_t)((mVolt - 1000) / 100);
	reg = Index == 0 ? NPM1300_LDSW_LDSW1VOUTSEL_REG :
		NPM1300_LDSW_LDSW2VOUTSEL_REG;
	if (!WriteReg8(reg, code))
		return false;

	reg = Index == 0 ? NPM1300_LDSW_LDSW1LDOSEL_REG :
		NPM1300_LDSW_LDSW2LDOSEL_REG;
	return WriteReg8(reg, NPM1300_LDSW_LDSW1LDOSEL_LDO);
}

bool PmnPM1300::SetLoadSwitchMode(size_t Index)
{
	uint16_t reg;

	if (Index > 1 || Interface() == NULL)
		return false;

	reg = Index == 0 ? NPM1300_LDSW_LDSW1LDOSEL_REG :
		NPM1300_LDSW_LDSW2LDOSEL_REG;
	return WriteReg8(reg, 0);
}

bool PmnPM1300::SetLoadSwitchEnabled(size_t Index, bool Enable)
{
	uint16_t reg;

	if (Index > 1 || Interface() == NULL)
		return false;

	if (Index == 0)
		reg = Enable ? NPM1300_LDSW_TASKLDSW1_SET_REG :
			NPM1300_LDSW_TASKLDSW1_CLR_REG;
	else
		reg = Enable ? NPM1300_LDSW_TASKLDSW2_SET_REG :
			NPM1300_LDSW_TASKLDSW2_CLR_REG;

	return WriteReg8(reg, 1);
}

bool PmnPM1300::SetLoadSwitchEnableGpio(size_t Index, int GpioIndex, bool Invert)
{
	uint16_t reg;
	uint8_t value;

	if (Index > 1 || GpioIndex < -1 || GpioIndex > 4 || Interface() == NULL)
		return false;

	reg = Index == 0 ? NPM1300_LDSW_LDSW1GPISEL_REG :
		NPM1300_LDSW_LDSW2GPISEL_REG;
	value = GpioIndex < 0 ? 0 : (uint8_t)(GpioIndex + 1);
	if (GpioIndex >= 0 && Invert)
		value |= NPM1300_LDSW_LDSW1GPISEL_LDSW1GPIINV_INVERTED;

	return UpdateReg8(reg, 0x0F, value);
}

bool PmnPM1300::SetLoadSwitchSoftStart(size_t Index, uint32_t mA, bool Enable)
{
	uint8_t disableMask;
	uint8_t selectMask;
	uint8_t shift;
	uint8_t code;
	uint8_t data;

	if (Index > 1 || Interface() == NULL)
		return false;

	switch (mA)
	{
	case 25:
		code = 0;
		break;
	case 50:
		code = 1;
		break;
	case 75:
		code = 2;
		break;
	case 100:
		code = 3;
		break;
	default:
		return false;
	}

	disableMask = Index == 0 ?
		NPM1300_LDSW_LDSWCONFIG_LDSW1SOFTSTARTDISABLE :
		NPM1300_LDSW_LDSWCONFIG_LDSW2SOFTSTARTDISABLE;
	selectMask = Index == 0 ?
		NPM1300_LDSW_LDSWCONFIG_LDSW1SOFTSTARTSEL_MASK :
		NPM1300_LDSW_LDSWCONFIG_LDSW2SOFTSTARTSEL_MASK;
	shift = Index == 0 ? 2 : 4;

	if (!ReadReg8(NPM1300_LDSW_LDSWCONFIG_REG, data))
		return false;
	data &= (uint8_t)~(disableMask | selectMask);
	data |= (uint8_t)((code << shift) & selectMask);
	if (!Enable)
		data |= disableMask;

	return WriteReg8(NPM1300_LDSW_LDSWCONFIG_REG, data);
}

bool PmnPM1300::SetLoadSwitchDischarge(size_t Index, bool Enable)
{
	uint8_t mask;

	if (Index > 1 || Interface() == NULL)
		return false;

	mask = Index == 0 ? NPM1300_LDSW_LDSWCONFIG_LDSW1ACTIVEDISCHARGE :
		NPM1300_LDSW_LDSWCONFIG_LDSW2ACTIVEDISCHARGE;
	return UpdateReg8(NPM1300_LDSW_LDSWCONFIG_REG, mask, Enable ? mask : 0);
}

bool PmnPM1300::ReadLoadSwitchStatus(uint8_t &Status)
{
	return ReadReg8(NPM1300_LDSW_LSDWSTATUS_REG, Status);
}

bool PmnPM1300::GpioState(size_t Index, bool &High)
{
	uint8_t status;

	if (Index > 4 || Interface() == NULL)
		return false;
	if (!ReadReg8(NPM1300_GPIOS_GPIOSTATUS_REG, status))
		return false;

	High = (status & (1U << Index)) != 0;
	return true;
}

bool PmnPM1300::SetGpioPull(size_t Index, bool PullUp, bool PullDown)
{
	if (Index > 4 || (PullUp && PullDown) || Interface() == NULL)
		return false;

	if (!WriteReg8((uint16_t)(NPM1300_GPIOS_GPIOPUEN_BASE_REG + Index), PullUp ? 1 : 0))
		return false;
	return WriteReg8((uint16_t)(NPM1300_GPIOS_GPIOPDEN_BASE_REG + Index), PullDown ? 1 : 0);
}

bool PmnPM1300::SetGpioOpenDrain(size_t Index, bool Enable)
{
	if (Index > 4 || Interface() == NULL)
		return false;
	return WriteReg8((uint16_t)(NPM1300_GPIOS_GPIOOPENDRAIN_BASE_REG + Index), Enable ? 1 : 0);
}

bool PmnPM1300::SetGpioDrive(size_t Index, uint32_t mA)
{
	uint8_t value;

	if (Index > 4 || Interface() == NULL)
		return false;
	if (mA == 1)
		value = NPM1300_GPIOS_GPIODRIVE_1MA;
	else if (mA == 6)
		value = NPM1300_GPIOS_GPIODRIVE_6MA;
	else
		return false;

	return WriteReg8((uint16_t)(NPM1300_GPIOS_GPIODRIVE_BASE_REG + Index), value);
}

bool PmnPM1300::ValidThermistor(const BatProfile_t * const pProfile) const
{
	if (pProfile == NULL)
		return true;
	if (pProfile->ThermBetaConst == 0 && pProfile->ThermResistor == 0)
		return true;
	if (pProfile->ThermBetaConst == 0 || pProfile->ThermResistor == 0)
		return false;

	return pProfile->ThermResistor == 10 || pProfile->ThermResistor == 47 ||
		pProfile->ThermResistor == 100;
}

bool PmnPM1300::ValidCharge(int32_t mVoltEoC, uint32_t mACurr) const
{
	if (mACurr == 0)
		return true;
	if (mACurr < 32 || mACurr > 800)
		return false;
	if (mVoltEoC < 3500 || mVoltEoC > 4450 || mVoltEoC % 50 != 0)
		return false;
	if (mVoltEoC >= 3700 && mVoltEoC < 4000)
		return false;
	return true;
}

bool PmnPM1300::ValidVout(int32_t mVolt) const
{
	if (mVolt == 0)
		return true;
	return mVolt >= 1000 && mVolt <= 3300 && (mVolt - 1000) % 100 == 0;
}

bool PmnPM1300::ConfigureNtc(const BatProfile_t * const pProfile)
{
	uint8_t ntc;

	if (pProfile == NULL || pProfile->ThermBetaConst == 0)
		return true;

	switch (pProfile->ThermResistor)
	{
	case 10:
		ntc = NPM1300_ADC_ADCNTCRSEL_10K;
		break;
	case 47:
		ntc = NPM1300_ADC_ADCNTCRSEL_47K;
		break;
	case 100:
		ntc = NPM1300_ADC_ADCNTCRSEL_100K;
		break;
	default:
		return false;
	}

	return WriteReg8(NPM1300_ADC_ADCNTCRSEL_REG, ntc);
}

bool PmnPM1300::ConfigureInterrupts(bool Enable)
{
	size_t i;
	bool result;

	result = true;
	for (i = 0; i < sizeof(s_Npm1300IrqGroups) / sizeof(s_Npm1300IrqGroups[0]); i++)
	{
		if (Enable)
		{
			if (!WriteReg8(s_Npm1300IrqGroups[i].EventClrReg, s_Npm1300IrqGroups[i].Mask))
				result = false;
			if (!WriteReg8(s_Npm1300IrqGroups[i].IntSetReg, s_Npm1300IrqGroups[i].Mask))
				result = false;
		}
		else if (!WriteReg8(s_Npm1300IrqGroups[i].IntClrReg, s_Npm1300IrqGroups[i].Mask))
			result = false;
	}

	return result;
}

bool PmnPM1300::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	size_t i;
	bool attached;

	Invalidate();
	attached = false;

	if (pIntrf == NULL || pIntrf->Type() != DEVINTRF_TYPE_I2C ||
		Cfg.DevAddr != NPM1300_I2C_7BITS_ADDR || Cfg.NbVout > 2 ||
		(Cfg.NbVout > 0 && Cfg.pVout == NULL) || Cfg.ChrgTimeout != 0 ||
		!ValidThermistor(Cfg.pBatProf) || !ValidCharge(Cfg.VEndChrg, Cfg.ChrgCurr))
		return false;

	for (i = 0; i < Cfg.NbVout; i++)
	{
		if (!ValidVout(Cfg.pVout[i].mVout))
			return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);
	attached = true;
	if (!Probe())
		goto fail;

	vpEvtHandler = Cfg.pEvtHandler;
	if (Cfg.pBatProf != NULL)
	{
		vNtcBeta = Cfg.pBatProf->ThermBetaConst;
		vBatProfile = *Cfg.pBatProf;
	}

	if (!ConfigureNtc(Cfg.pBatProf))
		goto fail;

	if (Cfg.bIntEn)
	{
		if (!ConfigureInterrupts(true))
			goto fail;
	}
	else if (!ConfigureInterrupts(false))
		goto fail;


	for (i = 0; i < Cfg.NbVout; i++)
	{
		if (SetVout(i, Cfg.pVout[i].mVout, Cfg.pVout[i].mAlimit) < 0)
			goto fail;
	}

	if (Cfg.ChrgCurr == 0)
	{
		if (!WriteReg8(NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
			NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING))
			goto fail;
	}
	else if (SetCharge(PWRMGNT_CHARGE_TYPE_NORMAL, Cfg.VEndChrg, Cfg.ChrgCurr) == 0)
		goto fail;

	InterruptEnabled(Cfg.bIntEn);
	Valid(true);
	return true;

fail:
	if (attached)
	{
		WriteReg8(NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
			NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING);
		ConfigureInterrupts(false);
	}
	Invalidate();
	return false;
}

bool PmnPM1300::Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf,
	PowerMgnt * const pPwrMnt)
{
	if (!ValidThermistor(&Cfg.BatProf))
		return false;
	if (!Init(pIntrf, Cfg.DevAddr))
		return false;

	vNtcBeta = Cfg.BatProf.ThermBetaConst;
	vBatProfile = Cfg.BatProf;
	vBatLowHandler = Cfg.BatLowHandler;
	vpPwrMgnt = pPwrMnt;

	if (!ConfigureNtc(&Cfg.BatProf))
	{
		Invalidate();
		return false;
	}

	Valid(true);
	return true;
}

uint16_t PmnPM1300::Level()
{
	return UINT16_MAX;
}

bool PmnPM1300::ReadAdc(uint16_t TaskReg, uint8_t EventMask, uint16_t MsbReg,
	uint8_t LsbShift, uint16_t &Code, uint16_t LsbReg)
{
	uint8_t events;
	uint8_t msb;
	uint8_t lsb;
	int i;

	if (Interface() == NULL)
		return false;
	if (!WriteReg8(NPM1300_MAIN_EVENTS_ADCCLR_REG, EventMask))
		return false;
	if (!WriteReg8(TaskReg, 1))
		return false;

	for (i = 0; i < NPM1300_ADC_POLL_COUNT; i++)
	{
		if (!ReadReg8(NPM1300_MAIN_EVENTS_ADCSET_REG, events))
			return false;
		if ((events & EventMask) != 0)
			break;
	}
	if (i == NPM1300_ADC_POLL_COUNT)
		return false;

	if (!ReadReg8(MsbReg, msb))
		return false;
	if (!ReadReg8(LsbReg, lsb))
		return false;
	if (!WriteReg8(NPM1300_MAIN_EVENTS_ADCCLR_REG, EventMask))
		return false;

	Code = ((uint16_t)msb << 2) | ((lsb >> LsbShift) & 3U);
	return true;
}

int32_t PmnPM1300::Temperature()
{
	uint16_t adc;
	float kelvin;
	float temp;

	if (vNtcBeta == 0 || Interface() == NULL)
		return INT32_MIN;
	if (!ReadAdc(NPM1300_ADC_TASKNTC_MEASURE_REG,
		NPM1300_MAIN_EVENTS_ADCSET_EVENTADCNTCRDY,
		NPM1300_ADC_ADCNTC_RESULTMSB_REG, 2, adc))
		return INT32_MIN;
	if (adc == 0 || adc >= 1023)
		return INT32_MIN;

	kelvin = 1.0f / (1.0f / 298.15f - logf(1024.0f / (float)adc - 1.0f) /
		(float)vNtcBeta);
	temp = (kelvin - 273.15f) * 10.0f;
	return (int32_t)lroundf(temp);
}

int32_t PmnPM1300::Voltage()
{
	uint16_t adc;

	if (!ReadAdc(NPM1300_ADC_TASKVBAT_MEASURE_REG,
		NPM1300_MAIN_EVENTS_ADCSET_EVENTADCVBATRDY,
		NPM1300_ADC_ADCVBAT_RESULTMSB_REG, 0, adc))
		return -1;

	return (int32_t)(((uint32_t)adc * 5000UL) / 1024UL);
}

int32_t PmnPM1300::Current()
{
	uint16_t adc;
	uint16_t limit;
	uint8_t status;
	uint8_t msb;
	uint8_t lsb;
	uint8_t config;
	int32_t scale;

	// IBAT uses the VBAT burst result-2 register and GP1 bits 5:4.
	// Burst mode would overwrite these registers with voltage samples.
	if (!ReadReg8(NPM1300_ADC_ADCCONFIG_REG, config) ||
		(config & NPM1300_ADC_ADCCONFIG_BURSTMODE) != 0)
		return INT32_MIN;
	if (!ReadAdc(NPM1300_ADC_TASKIBAT_MEASURE_REG,
		NPM1300_MAIN_EVENTS_ADCSET_EVENTADCIBATRDY,
		NPM1300_ADC_ADCVBAT2_RESULTMSB_REG, 4, adc,
		NPM1300_ADC_ADCGP1_RESULTLSBS_REG) ||
		!ReadReg8(NPM1300_ADC_ADCIBATMEAS_STATUS_REG, status))
		return INT32_MIN;

	if (status == 0x04)
	{
		if (!ReadReg8(NPM1300_BCHARGER_BCHGISET_DISCHARGE_MSB_REG, msb) ||
			!ReadReg8(NPM1300_BCHARGER_BCHGISET_DISCHARGE_LSB_REG, lsb))
			return INT32_MIN;
		limit = ((uint16_t)msb << 1) | (lsb & 1U);
		if (limit != 84 && limit != 415)
			return INT32_MIN;
		scale = limit == 84 ? 224000 : 1120000;
	}
	else if (status == 0x0C || status == 0x0D || status == 0x0F)
	{
		if (!ReadReg8(NPM1300_BCHARGER_BCHGISET_MSB_REG, msb) ||
			!ReadReg8(NPM1300_BCHARGER_BCHGISET_LSB_REG, lsb))
			return INT32_MIN;
		limit = ((uint16_t)msb << 1) | (lsb & 1U);
		if (limit < 16 || limit > 400)
			return INT32_MIN;
		scale = -(int32_t)limit * 2500;
	}
	else
	{
		// Includes IBATMEASINVALID; do not pass a false zero to the estimator.
		return INT32_MIN;
	}

	return (int32_t)((int64_t)adc * scale / 1023);
}

bool PmnPM1300::ReadBatterySample(Npm1300BatterySample_t &Sample)
{
	Npm1300BatterySample_t sample = {};
	uint8_t msb;
	uint8_t lsb;
	uint8_t term;

	sample.VoltageMv = Voltage();
	sample.CurrentUa = Current();
	sample.TemperatureDeciC = Temperature();
	if (sample.VoltageMv <= 0 || sample.CurrentUa == INT32_MIN ||
		sample.TemperatureDeciC == INT32_MIN)
		return false;
	if (!ReadReg8(NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG, sample.ChargerStatus) ||
		!ReadReg8(NPM1300_VBUSIN_VBUSIN_STATUS_REG, sample.VbusStatus) ||
		!ReadReg8(NPM1300_BCHARGER_BCHGISET_MSB_REG, msb) ||
		!ReadReg8(NPM1300_BCHARGER_BCHGISET_LSB_REG, lsb) ||
		!ReadReg8(NPM1300_BCHARGER_BCHGITERM_SEL_REG, term))
		return false;
	sample.ChargeLimitMa = (((uint32_t)msb << 1) | (lsb & 1U)) * 2;
	if (sample.ChargeLimitMa < 32 || sample.ChargeLimitMa > 800 || term > 1)
		return false;
	sample.TerminationPercent = term == 0 ? 10 : 20;
	Sample = sample;
	return true;
}

int32_t PmnPM1300::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit)
{
	uint16_t voltageReg;
	uint16_t enableReg;
	uint8_t controlMask;
	uint8_t code;

	(void)CurrLimit;
	if (Interface() == NULL || VoutIdx > 1 || !ValidVout(mVolt))
		return -1;

	if (mVolt != 0)
	{
		voltageReg = VoutIdx == 0 ? NPM1300_BUCK_BUCK1NORM_VOUT_REG :
			NPM1300_BUCK_BUCK2NORM_VOUT_REG;
		code = (uint8_t)((mVolt - 1000) / 100);
		if (!WriteReg8(voltageReg, code))
			return -1;
	}

	controlMask = VoutIdx == 0 ?
		NPM1300_BUCK_BUCKSWCTRLSEL_BUCK1SWCTRLSEL_SWCTRL :
		NPM1300_BUCK_BUCKSWCTRLSEL_BUCK2SWCTRLSEL_SWCTRL;
	if (!UpdateReg8(NPM1300_BUCK_BUCKSWCTRLSEL_REG, controlMask, controlMask))
		return -1;

	enableReg = VoutIdx == 0 ?
		(mVolt != 0 ? NPM1300_BUCK_BUCK1ENA_SET_REG : NPM1300_BUCK_BUCK1ENA_CLR_REG) :
		(mVolt != 0 ? NPM1300_BUCK_BUCK2ENA_SET_REG : NPM1300_BUCK_BUCK2ENA_CLR_REG);
	return WriteReg8(enableReg, 1) ? mVolt : -1;
}

bool PmnPM1300::Enable()
{
	return WriteReg8(NPM1300_BCHARGER_BCHGENABLE_SET_REG,
		NPM1300_BCHARGER_BCHGENABLE_SET_ENABLECHARGING);
}

void PmnPM1300::Disable()
{
	WriteReg8(NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
		NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING);
}

void PmnPM1300::Reset()
{
	if (WriteReg8(NPM1300_MAIN_TASKSW_RESET_REG, 1))
		Invalidate();
}

bool PmnPM1300::EnterShipMode()
{
	uint8_t status;

	if (!ReadReg8(NPM1300_VBUSIN_VBUSIN_STATUS_REG, status))
		return false;
	if ((status & NPM1300_VBUSIN_VBUSIN_STATUS_VBUSINPRESENT_DETECTED) != 0)
		return false;
	if (!WriteReg8(NPM1300_SHIP_TASKENTER_SHIPMODE_REG, 1))
		return false;

	Invalidate();
	return true;
}

void PmnPM1300::PowerOff()
{
	EnterShipMode();
}

bool PmnPM1300::SetTerminationVoltage(uint16_t Reg, int32_t mVolt)
{
	uint8_t code;

	if (mVolt < 3500 || mVolt > 4450 || mVolt % 50 != 0)
		return false;
	if (mVolt >= 3700 && mVolt < 4000)
		return false;

	code = mVolt < 3700 ? (uint8_t)((mVolt - 3500) / 50) :
		(uint8_t)((mVolt - 4000) / 50 + 4);
	return WriteReg8(Reg, code);
}

uint32_t PmnPM1300::SetCharge(PWRMGNT_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
{
	uint32_t actual;
	uint16_t raw;

	if (Interface() == NULL)
		return 0;
	if (mACurr == 0)
	{
		if (!WriteReg8(NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
			NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING))
			return 0;
		vChrgCurr = 0;
		return 0;
	}
	if (Type != PWRMGNT_CHARGE_TYPE_NORMAL && Type != PWRMGNT_CHARGE_TYPE_AUTO)
		return 0;
	if (!ValidCharge(mVoltEoC, mACurr))
		return 0;

	if (!WriteReg8(NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
		NPM1300_BCHARGER_BCHGENABLE_CLR_ENABLECHARGING))
		return 0;

	actual = mACurr & ~1U;
	raw = (uint16_t)(actual / 2);
	if (!WriteReg8(NPM1300_BCHARGER_BCHGISET_MSB_REG, (uint8_t)(raw >> 1)))
		return 0;
	if (!WriteReg8(NPM1300_BCHARGER_BCHGISET_LSB_REG, (uint8_t)(raw & 1U)))
		return 0;
	if (!SetTerminationVoltage(NPM1300_BCHARGER_BCHGVTERM_REG, mVoltEoC))
		return 0;
	if (!SetTerminationVoltage(NPM1300_BCHARGER_BCHGVTERMR_REG, mVoltEoC))
		return 0;
	if (!Enable())
		return 0;

	vChrgCurr = actual;
	return actual;
}

bool PmnPM1300::SetWarmTerminationVoltage(int32_t mVolt)
{
	return SetTerminationVoltage(NPM1300_BCHARGER_BCHGVTERMR_REG, mVolt);
}

bool PmnPM1300::SetTerminationCurrent(uint32_t Percent)
{
	uint8_t value;

	if (Percent == 10)
		value = NPM1300_BCHARGER_BCHGITERM_SEL_BCHGITERMSEL_SEL10;
	else if (Percent == 20)
		value = NPM1300_BCHARGER_BCHGITERM_SEL_BCHGITERMSEL_SEL20;
	else
		return false;

	return WriteReg8(NPM1300_BCHARGER_BCHGITERM_SEL_REG, value);
}

bool PmnPM1300::SetTrickleVoltage(int32_t mVolt)
{
	uint8_t value;

	if (mVolt == 2900)
		value = NPM1300_BCHARGER_BCHGV_TRICKLE_SEL_BCHGVTRICKLESEL_2V9;
	else if (mVolt == 2500)
		value = NPM1300_BCHARGER_BCHGV_TRICKLE_SEL_BCHGVTRICKLESEL_2V5;
	else
		return false;

	return WriteReg8(NPM1300_BCHARGER_BCHGV_TRICKLE_SEL_REG, value);
}

bool PmnPM1300::SetAutomaticRecharge(bool Enable)
{
	return WriteReg8(Enable ? NPM1300_BCHARGER_BCHGDISABLE_CLR_REG :
		NPM1300_BCHARGER_BCHGDISABLE_SET_REG,
		NPM1300_BCHARGER_BCHGDISABLE_SET_DISABLERECHARGE);
}

bool PmnPM1300::SetNtcProtection(bool Enable)
{
	return WriteReg8(Enable ? NPM1300_BCHARGER_BCHGDISABLE_CLR_REG :
		NPM1300_BCHARGER_BCHGDISABLE_SET_REG,
		NPM1300_BCHARGER_BCHGDISABLE_SET_DISABLENTC);
}

bool PmnPM1300::NtcCodeForTemperature(int32_t Celsius, uint16_t &Code) const
{
	float target;
	float exponent;
	float code;

	if (vNtcBeta == 0 || Celsius < -20 || Celsius > 60)
		return false;

	target = (float)Celsius + 273.15f;
	exponent = (1.0f / 298.15f - 1.0f / target) * (float)vNtcBeta;
	code = 1024.0f / (expf(exponent) + 1.0f);
	if (code < 0.0f || code > 1023.0f)
		return false;

	Code = (uint16_t)lroundf(code);
	return true;
}

bool PmnPM1300::SetNtcTemperatureLimits(int32_t Cold, int32_t Cool, int32_t Warm, int32_t Hot)
{
	uint16_t coldCode;
	uint16_t coolCode;
	uint16_t warmCode;
	uint16_t hotCode;

	if (Cold > Cool || Cool > Warm || Warm > Hot)
		return false;
	if (!NtcCodeForTemperature(Cold, coldCode) || !NtcCodeForTemperature(Cool, coolCode) ||
		!NtcCodeForTemperature(Warm, warmCode) || !NtcCodeForTemperature(Hot, hotCode))
		return false;

	if (!WriteReg10(NPM1300_BCHARGER_NTCCOLD_REG, coldCode))
		return false;
	if (!WriteReg10(NPM1300_BCHARGER_NTCCOOL_REG, coolCode))
		return false;
	if (!WriteReg10(NPM1300_BCHARGER_NTCWARM_REG, warmCode))
		return false;
	return WriteReg10(NPM1300_BCHARGER_NTCHOT_REG, hotCode);
}

bool PmnPM1300::SetDieTemperatureLimits(int32_t StopCelsius, int32_t ResumeCelsius)
{
	int32_t stopCode;
	int32_t resumeCode;

	if (StopCelsius < -40 || StopCelsius > 125 || ResumeCelsius < -40 ||
		ResumeCelsius > 125 || ResumeCelsius >= StopCelsius)
		return false;

	stopCode = (int32_t)lroundf((3946700.0f - (float)StopCelsius * 10000.0f) / 7926.0f);
	resumeCode = (int32_t)lroundf((3946700.0f - (float)ResumeCelsius * 10000.0f) / 7926.0f);
	if (stopCode < 0 || stopCode > 1023 || resumeCode < 0 || resumeCode > 1023)
		return false;

	if (!WriteReg10(NPM1300_BCHARGER_DIE_TEMP_STOP_REG, (uint16_t)stopCode))
		return false;
	return WriteReg10(NPM1300_BCHARGER_DIE_TEMP_RESUME_REG, (uint16_t)resumeCode);
}

bool PmnPM1300::SetWarmCharging(bool Enable)
{
	return UpdateReg8(NPM1300_BCHARGER_BCHG_CONFIG_REG,
		NPM1300_BCHARGER_BCHG_CONFIG_DISABLECHARGEWARM,
		Enable ? 0 : NPM1300_BCHARGER_BCHG_CONFIG_DISABLECHARGEWARM);
}

bool PmnPM1300::SetCoolFullCurrent(bool Enable)
{
	return WriteReg8(Enable ? NPM1300_BCHARGER_BCHGENABLE_SET_REG :
		NPM1300_BCHARGER_BCHGENABLE_CLR_REG,
		NPM1300_BCHARGER_BCHGENABLE_SET_ENABLEFULLCHGCOOL);
}

bool PmnPM1300::SetDischargeCurrentLimit(uint32_t mA)
{
	uint16_t code;

	if (mA == 200)
		code = 84;
	else if (mA == 1000)
		code = 415;
	else
		return false;

	if (!WriteReg8(NPM1300_BCHARGER_BCHGISET_DISCHARGE_MSB_REG, (uint8_t)(code >> 1)))
		return false;
	return WriteReg8(NPM1300_BCHARGER_BCHGISET_DISCHARGE_LSB_REG, (uint8_t)(code & 1U));
}

bool PmnPM1300::SetInputCurrentLimit(uint32_t mA)
{
	uint8_t code;

	if (mA == 100)
		code = NPM1300_VBUSIN_VBUSIN_ILIM0_100MA;
	else if (mA == 500)
		code = NPM1300_VBUSIN_VBUSIN_ILIM0_500MA;
	else if (mA >= 600 && mA <= 1500 && mA % 100 == 0)
		code = (uint8_t)(mA / 100);
	else
		return false;

	if (!WriteReg8(NPM1300_VBUSIN_VBUSIN_ILIM0_REG, code))
		return false;
	if (!WriteReg8(NPM1300_VBUSIN_TASKUPDATE_ILIMSW_REG,
		NPM1300_VBUSIN_TASKUPDATE_ILIMSW_SELVBUSILIM0))
		return false;

	vInputCurrentLimit = mA;
	return true;
}

bool PmnPM1300::ClearChargerError()
{
	return WriteReg8(NPM1300_BCHARGER_TASK_CLEAR_CHGERR_REG, 1);
}

bool PmnPM1300::RecoverCharger()
{
	if (!ClearChargerError())
		return false;
	return WriteReg8(NPM1300_BCHARGER_TASK_RELEASE_ERR_REG, 1);
}

bool PmnPM1300::ClearChargerSafetyTimers()
{
	return WriteReg8(NPM1300_BCHARGER_TASK_CLEAR_SAFTY_TIMER_REG, 1);
}

bool PmnPM1300::Charging()
{
	uint8_t status;
	uint8_t active;

	if (!ReadReg8(NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG, status))
		return false;

	active = NPM1300_BCHARGER_BCHG_CHARGE_STATUS_TRICKLECHARGE |
		NPM1300_BCHARGER_BCHG_CHARGE_STATUS_CONSTANTCURRENT |
		NPM1300_BCHARGER_BCHG_CHARGE_STATUS_CONSTANTVOLTAGE;
	return (status & active) != 0;
}

PWRMGNT_BATTERY_STATE PmnPM1300::BatteryState()
{
	return PWRMGNT_BATTERY_UNKNOWN;
}

bool PmnPM1300::Battery()
{
	return BatteryState() == PWRMGNT_BATTERY_PRESENT;
}

void PmnPM1300::IrqHandler()
{
	uint8_t events;
	uint8_t active;
	size_t i;

	if (Interface() == NULL || !InterruptEnabled())
		return;

	for (i = 0; i < sizeof(s_Npm1300IrqGroups) / sizeof(s_Npm1300IrqGroups[0]); i++)
	{
		if (!ReadReg8(s_Npm1300IrqGroups[i].EventReg, events))
			continue;
		active = events & s_Npm1300IrqGroups[i].Mask;
		if (active == 0)
			continue;
		if (!WriteReg8(s_Npm1300IrqGroups[i].EventClrReg, active))
			continue;

		if (i == 2 && (active & NPM1300_IRQ_VBUSIN0_MASK) != 0 && vInputCurrentLimit != 0)
			WriteReg8(NPM1300_VBUSIN_TASKUPDATE_ILIMSW_REG,
				NPM1300_VBUSIN_TASKUPDATE_ILIMSW_SELVBUSILIM0);

		if (vpEvtHandler == NULL)
			continue;

		switch (i)
		{
		case 0:
			vpEvtHandler(this, PWRMGNT_EVT_OVER_HEAT);
			break;
		case 1:
			vpEvtHandler(this, PWRMGNT_EVT_CHARGE_FULL);
			break;
		case 2:
			vpEvtHandler(this, PWRMGNT_EVT_CHARGE_DETECTED);
			break;
		case 3:
			vpEvtHandler(this, PWRMGNT_EVT_OVER_HEAT);
			break;
		default:
			break;
		}
	}
}

bool PmnPM1300::SetGpioMode(size_t Index, uint8_t Mode)
{
	if (Index > 4 || Mode > 9 || Interface() == NULL)
		return false;
	if (Mode >= NPM1300_GPIOS_GPIOMODE_GPOIRQ &&
		!SetGpioPull(Index, false, false))
		return false;
	return WriteReg8((uint16_t)(NPM1300_GPIOS_GPIOMODE_BASE_REG + Index), Mode);
}

bool PmnPM1300::SetBuckGpioControl(uint16_t Reg, size_t Index,
	int GpioIndex, bool Invert)
{
	uint8_t shift;
	uint8_t mask;
	uint8_t value;

	if (Index > 1 || GpioIndex < -1 || GpioIndex > 4 || Interface() == NULL)
		return false;

	shift = Index == 0 ? 0 : 3;
	mask = (uint8_t)(7U << shift);
	value = GpioIndex < 0 ? 0 : (uint8_t)((GpioIndex + 1) << shift);
	if (Index == 0)
	{
		mask |= (1U << 6);
		if (GpioIndex >= 0 && Invert)
			value |= (1U << 6);
	}
	else
	{
		mask |= (1U << 7);
		if (GpioIndex >= 0 && Invert)
			value |= (1U << 7);
	}

	return UpdateReg8(Reg, mask, value);
}

bool PmnPM1300::SetBuckEnableGpio(size_t Index, int GpioIndex, bool Invert)
{
	return SetBuckGpioControl(NPM1300_BUCK_BUCKENCTRL_REG, Index,
		GpioIndex, Invert);
}

bool PmnPM1300::SetBuckRetentionGpio(size_t Index, int GpioIndex, bool Invert)
{
	return SetBuckGpioControl(NPM1300_BUCK_BUCKVRETCTRL_REG, Index,
		GpioIndex, Invert);
}

bool PmnPM1300::SetBuckPwmGpio(size_t Index, int GpioIndex, bool Invert)
{
	return SetBuckGpioControl(NPM1300_BUCK_BUCKPWMCTRL_REG, Index,
		GpioIndex, Invert);
}

bool PmnPM1300::SetBuckPwm(size_t Index, bool Enable)
{
	uint16_t reg;
	uint8_t pfmMask;

	if (Index > 1 || Interface() == NULL)
		return false;

	if (Enable)
	{
		pfmMask = (uint8_t)(1U << Index);
		if (!UpdateReg8(NPM1300_BUCK_BUCKCTRL0_REG, pfmMask, 0))
			return false;
	}

	reg = (uint16_t)(NPM1300_BUCK_BUCK1PWM_SET_REG + 2 * Index + (Enable ? 0 : 1));
	return WriteReg8(reg, 1);
}

bool PmnPM1300::SetBuckPfm(size_t Index, bool Enable)
{
	uint8_t mask;

	if (Index > 1 || Interface() == NULL)
		return false;
	if (Enable && !SetBuckPwm(Index, false))
		return false;

	mask = (uint8_t)(1U << Index);
	return UpdateReg8(NPM1300_BUCK_BUCKCTRL0_REG, mask, Enable ? mask : 0);
}

bool PmnPM1300::SetBuckRetention(size_t Index, int32_t mVolt)
{
	uint16_t reg;

	if (Index > 1 || mVolt < 1000 || mVolt > 3300 ||
		(mVolt - 1000) % 100 != 0 || Interface() == NULL)
		return false;
	reg = (uint16_t)(NPM1300_BUCK_BUCK1RET_VOUT_REG + 2 * Index);
	return WriteReg8(reg, (uint8_t)((mVolt - 1000) / 100));
}

bool PmnPM1300::SetBuckDischarge(size_t Index, bool Enable)
{
	uint8_t mask;

	if (Index > 1 || Interface() == NULL)
		return false;
	mask = Index == 0 ? NPM1300_BUCK_BUCKCTRL0_BUCK1ENPULLDOWN_ENABLE :
		NPM1300_BUCK_BUCKCTRL0_BUCK2ENPULLDOWN_ENABLE;
	return UpdateReg8(NPM1300_BUCK_BUCKCTRL0_REG, mask, Enable ? mask : 0);
}

bool PmnPM1300::ReadBuckStatus(size_t Index, uint8_t &VoltageCode, uint8_t &Status)
{
	uint16_t reg;

	if (Index > 1 || Interface() == NULL)
		return false;
	reg = Index == 0 ? NPM1300_BUCK_BUCK1VOUT_STATUS_REG :
		NPM1300_BUCK_BUCK2VOUT_STATUS_REG;
	if (!ReadReg8(reg, VoltageCode))
		return false;
	return ReadReg8(NPM1300_BUCK_BUCKSTATUS_REG, Status);
}

bool PmnPM1300::ReadStatus(uint8_t &Charger, uint8_t &ChargerErrors,
	uint8_t &Vbus, uint8_t &Ntc, uint8_t &ResetCause)
{
	const uint16_t regs[] = {
		NPM1300_BCHARGER_BCHG_CHARGE_STATUS_REG,
		NPM1300_BCHARGER_BCHG_ERR_REASON_REG,
		NPM1300_VBUSIN_VBUSIN_STATUS_REG,
		NPM1300_BCHARGER_NTC_STATUS_REG,
		NPM1300_ERRLOG_RSTCAUSE_REG
	};
	uint8_t values[5];
	size_t i;

	if (Interface() == NULL)
		return false;
	for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
	{
		if (!ReadReg8(regs[i], values[i]))
			return false;
	}

	Charger = values[0];
	ChargerErrors = values[1];
	Vbus = values[2];
	Ntc = values[3];
	ResetCause = values[4];
	return true;
}

bool PmnPM1300::ReadChargerError(uint8_t &Reason, uint8_t &Sensor)
{
	uint8_t reason;
	uint8_t sensor;

	if (!ReadReg8(NPM1300_BCHARGER_BCHG_ERR_REASON_REG, reason))
		return false;
	if (!ReadReg8(NPM1300_BCHARGER_BCHG_ERR_SENSOR_REG, sensor))
		return false;

	Reason = reason;
	Sensor = sensor;
	return true;
}

bool PmnPM1300::ClearErrorLog()
{
	return WriteReg8(NPM1300_ERRLOG_TASKCLRERRLOG_REG, 1);
}
