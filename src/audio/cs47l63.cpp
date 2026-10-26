/**-------------------------------------------------------------------------
@file	cs47l63.cpp

@brief	Cirrus Logic CS47L63 control-interface driver

@author	Nguyen Hoan Hoang
@date	Oct. 24, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "audio/cs47l63.h"
#include "iopinctrl.h"
#include "idelay.h"

// CS47L63 DS1249F2 register addresses. The values below are the Audio DK
// 48-kHz ASP1 configuration, with separate microphone and headphone routing.
#define CS47L63_REG_SYSCLK				0x1404U
#define CS47L63_REG_ASYNCCLK			0x1460U
#define CS47L63_REG_SAMPLE_RATE1		0x1420U
#define CS47L63_REG_SAMPLE_RATE2		0x1424U
#define CS47L63_REG_SAMPLE_RATE3		0x1428U
#define CS47L63_REG_SAMPLE_RATE4		0x142CU
#define CS47L63_REG_FLL1_CTRL1			0x1C00U
#define CS47L63_REG_FLL1_CTRL2			0x1C04U
#define CS47L63_REG_FLL1_CTRL3			0x1C08U
#define CS47L63_REG_FLL1_GPIO_CLOCK		0x1CA0U
#define CS47L63_REG_GPIO1				0x0C08U
#define CS47L63_REG_LDO2				0x2408U
#define CS47L63_REG_MICBIAS1			0x2410U
#define CS47L63_REG_MICBIAS5			0x2418U
#define CS47L63_REG_INPUT_ENABLE		0x4000U
#define CS47L63_REG_INPUT_VU			0x4014U
#define CS47L63_REG_INPUT1				0x4020U
#define CS47L63_REG_IN1L_VOLUME			0x4028U
#define CS47L63_REG_IN1R_VOLUME			0x4048U
#define CS47L63_REG_INPUT2				0x4060U
#define CS47L63_REG_IN2L_SOURCE			0x4064U
#define CS47L63_REG_IN2L_VOLUME			0x4068U
#define CS47L63_REG_IN2R_SOURCE			0x4084U
#define CS47L63_REG_IN2R_VOLUME			0x4088U
#define CS47L63_REG_OUT_ENABLE			0x4804U
#define CS47L63_REG_OUT_VOLUME			0x4818U
#define CS47L63_REG_ASP1_ENABLE			0x6000U
#define CS47L63_REG_ASP1_CTRL2			0x6008U
#define CS47L63_REG_ASP1_CTRL3			0x600CU
#define CS47L63_REG_ASP1_TX_DATA		0x6030U
#define CS47L63_REG_ASP1_RX_DATA		0x6040U
#define CS47L63_REG_OUT1L_INPUT1		0x8100U
#define CS47L63_REG_OUT1L_INPUT2		0x8104U
#define CS47L63_REG_ASP1TX1_INPUT		0x8200U
#define CS47L63_REG_ASP1TX2_INPUT		0x8210U
#define CS47L63_REG_IRQ_EINT1			0x18010U
#define CS47L63_REG_IRQ_EINT6			0x18024U


bool Cs47l63::PinValid(const IOPinCfg_t &Pin) const
{
	return Pin.PortNo >= 0 && Pin.PinNo >= 0;
}

bool Cs47l63::IrqActive() const
{
	if (!PinValid(vCfg.IrqPin))
	{
		return false;
	}

	return IOPinRead(vCfg.IrqPin.PortNo, vCfg.IrqPin.PinNo) == 0;
}

bool Cs47l63::WaitBoot()
{
	uint32_t count;
	uint32_t pending;

	// BOOT_DONE is latched in IRQ1_EINT_2. The external IRQ pin is
	// masked until the codec enables the corresponding interrupt source.
	// Waiting on that pin would time out on an otherwise healthy codec.
	count = vCfg.BootTimeoutMs;
	while (count > 0)
	{
		if (ReadReg(0x18014U, pending) && (pending & (1U << 3)) != 0)
		{
			return true;
		}
		msDelay(1);
		count--;
	}

	return false;
}

bool Cs47l63::Boot()
{
	uint32_t id;
	uint32_t revision;

	IOPinSet(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
	if (!WaitBoot())
	{
		return false;
	}

	if (!ReadReg(CS47L63_REG_DEVID, id) ||
		(id & 0x00FFFFFFU) != CS47L63_DEVICE_ID)
	{
		return false;
	}
	if (!ReadReg(CS47L63_REG_REVID, revision))
	{
		return false;
	}

	vRevision = uint8_t(revision);
	DeviceID((uint64_t(CS47L63_DEVICE_ID) << 8) | vRevision);
	vbEnabled = true;

	return true;
}

bool Cs47l63::Init(const Cs47l63Cfg_t &Cfg, DeviceIntrf *pIntrf)
{
	Valid(false);
	vbEnabled = false;
	vRevision = 0;
	vbAudioConfigured = false;
	vbAudioSaved = false;

	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_SPI ||
		!PinValid(Cfg.ResetPin) || !PinValid(Cfg.IrqPin) ||
		Cfg.BootTimeoutMs == 0)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vCfg = Cfg;

	IOPinClear(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
	IOPinConfig(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo,
				vCfg.ResetPin.PinOp, IOPINDIR_OUTPUT,
				vCfg.ResetPin.Res, vCfg.ResetPin.Type);
	IOPinConfig(vCfg.IrqPin.PortNo, vCfg.IrqPin.PinNo,
				vCfg.IrqPin.PinOp, IOPINDIR_INPUT,
				vCfg.IrqPin.Res, vCfg.IrqPin.Type);
	// Hold reset long enough for warm-reset and cold power-up cases.
	msDelay(2);

	if (!Boot())
	{
		IOPinClear(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
		return false;
	}

	Valid(true);

	return true;
}

bool Cs47l63::Enable()
{
	if (!Device::Valid())
	{
		return false;
	}
	if (vbEnabled)
	{
		return true;
	}

	if (!Boot())
	{
		return false;
	}
	if (vbAudioSaved && !ConfigureAudio(vAudioCfg))
	{
		Valid(false);
		return false;
	}
	return true;
}

void Cs47l63::Disable()
{
	if (!Device::Valid())
	{
		return;
	}

	StopAudio();
	IOPinClear(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
	vbEnabled = false;
}

void Cs47l63::Reset()
{
	bool result;

	if (!Device::Valid())
	{
		return;
	}

	IOPinClear(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
	msDelay(2);
	result = Boot();
	if (result && vbAudioSaved)
	{
		result = ConfigureAudio(vAudioCfg);
	}
	if (!result)
	{
		Valid(false);
		vbEnabled = false;
	}
}

void Cs47l63::PowerOff()
{
	StopAudio();
	vbAudioSaved = false;
	if (PinValid(vCfg.ResetPin))
	{
		IOPinClear(vCfg.ResetPin.PortNo, vCfg.ResetPin.PinNo);
	}

	vbEnabled = false;
	Valid(false);
}

bool Cs47l63::ReadBlock(uint32_t Reg, uint8_t *pData, size_t DataLen)
{
	uint8_t command[8];
	uint32_t address;
	int count;

	if (Interface() == nullptr || pData == nullptr || DataLen == 0 ||
		(DataLen & 3U) != 0 || DataLen > 0x7FFFFFFFU)
	{
		return false;
	}

	address = (Reg & CS47L63_SPI_ADDR_MASK) | CS47L63_SPI_READ;
	command[0] = uint8_t(address >> 24);
	command[1] = uint8_t(address >> 16);
	command[2] = uint8_t(address >> 8);
	command[3] = uint8_t(address);
	command[4] = 0;
	command[5] = 0;
	command[6] = 0;
	command[7] = 0;

	count = Interface()->Read(DeviceAddress(), command, sizeof(command),
		pData, int(DataLen));

	return count == int(DataLen);
}

bool Cs47l63::WriteBlock(uint32_t Reg, const uint8_t *pData, size_t DataLen)
{
	uint8_t command[8];
	uint32_t address;
	int count;

	if (Interface() == nullptr || pData == nullptr || DataLen == 0 ||
		(DataLen & 3U) != 0 || DataLen > 0x7FFFFFFFU)
	{
		return false;
	}

	address = Reg & CS47L63_SPI_ADDR_MASK;
	command[0] = uint8_t(address >> 24);
	command[1] = uint8_t(address >> 16);
	command[2] = uint8_t(address >> 8);
	command[3] = uint8_t(address);
	command[4] = 0;
	command[5] = 0;
	command[6] = 0;
	command[7] = 0;

	count = Interface()->Write(DeviceAddress(), command, sizeof(command),
		pData, int(DataLen));

	return count == int(DataLen);
}

bool Cs47l63::ReadReg(uint32_t Reg, uint32_t &Value)
{
	uint8_t data[4];

	if (!ReadBlock(Reg, data, sizeof(data)))
	{
		return false;
	}

	Value = (uint32_t(data[0]) << 24) |
			(uint32_t(data[1]) << 16) |
			(uint32_t(data[2]) << 8) |
			uint32_t(data[3]);

	return true;
}

bool Cs47l63::WriteReg(uint32_t Reg, uint32_t Value)
{
	uint8_t data[4];

	data[0] = uint8_t(Value >> 24);
	data[1] = uint8_t(Value >> 16);
	data[2] = uint8_t(Value >> 8);
	data[3] = uint8_t(Value);

	return WriteBlock(Reg, data, sizeof(data));
}

bool Cs47l63::UpdateReg(uint32_t Reg, uint32_t Mask, uint32_t Value)
{
	uint32_t current;

	if (!ReadReg(Reg, current))
	{
		return false;
	}

	current = (current & ~Mask) | (Value & Mask);

	return WriteReg(Reg, current);
}

bool Cs47l63::Apply(const Cs47l63RegVal_t *pConfig, size_t Count)
{
	size_t index;

	if (!Device::Valid() || !vbEnabled ||
		(pConfig == nullptr && Count != 0))
	{
		return false;
	}

	for (index = 0; index < Count; index++)
	{
		if (!WriteReg(pConfig[index].Reg, pConfig[index].Value))
		{
			return false;
		}
		if (pConfig[index].DelayMs != 0)
		{
			msDelay(pConfig[index].DelayMs);
		}
	}

	return true;
}


bool Cs47l63::StopAudio()
{
	bool ok;

	if (!Device::Valid() || !vbEnabled)
	{
		vbAudioConfigured = false;
		return false;
	}
	if (!vbAudioConfigured)
	{
		return true;
	}

	ok = WriteReg(CS47L63_REG_OUT_VOLUME, 0x380U);
	ok = WriteReg(CS47L63_REG_OUT_ENABLE, 0) && ok;
	ok = WriteReg(CS47L63_REG_INPUT_ENABLE, 0) && ok;
	ok = WriteReg(CS47L63_REG_ASP1_ENABLE, 0) && ok;
	ok = WriteReg(CS47L63_REG_MICBIAS5, 0) && ok;
	ok = WriteReg(CS47L63_REG_MICBIAS1, 0) && ok;
	ok = WriteReg(CS47L63_REG_LDO2, 0) && ok;
	ok = WriteReg(CS47L63_REG_SYSCLK, 0x030CU) && ok;
	ok = WriteReg(CS47L63_REG_ASYNCCLK, 0x030CU) && ok;
	ok = WriteReg(CS47L63_REG_FLL1_CTRL1, 0) && ok;
	vbAudioConfigured = false;
	vbMuted = true;
	return ok;
}

bool Cs47l63::SelectInput(CS47L63_INPUT Input)
{
	bool ok;

	if (!Device::Valid() || !vbEnabled ||
		(Input != CS47L63_INPUT_OFF && Input != CS47L63_INPUT_PDM &&
		 Input != CS47L63_INPUT_LINE))
	{
		return false;
	}

	// Disconnect the old source before changing ADC/PDM configuration.
	ok = WriteReg(CS47L63_REG_INPUT_ENABLE, 0);
	ok = WriteReg(CS47L63_REG_ASP1TX1_INPUT, 0) && ok;
	ok = WriteReg(CS47L63_REG_ASP1TX2_INPUT, 0) && ok;
	if (!ok)
	{
		return false;
	}

	if (Input == CS47L63_INPUT_PDM)
	{
		ok = WriteReg(CS47L63_REG_LDO2, 0x0005U);
		ok = WriteReg(CS47L63_REG_MICBIAS1, 0x00ECU) && ok;
		ok = WriteReg(CS47L63_REG_MICBIAS5, 0x0272U) && ok;
		ok = WriteReg(CS47L63_REG_INPUT1, 0x00050021U) && ok;
		ok = WriteReg(CS47L63_REG_IN1L_VOLUME, 0x00800080U) && ok;
		ok = WriteReg(CS47L63_REG_IN1R_VOLUME, 0x00800080U) && ok;
		ok = WriteReg(CS47L63_REG_INPUT_ENABLE, 0x000FU) && ok;
		ok = WriteReg(CS47L63_REG_INPUT_VU, 0x20000000U) && ok;
		ok = WriteReg(CS47L63_REG_ASP1TX1_INPUT, 0x00800010U) && ok;
		ok = WriteReg(CS47L63_REG_ASP1TX2_INPUT, 0x00800011U) && ok;
	}
	else
	{
		ok = WriteReg(CS47L63_REG_MICBIAS5, 0);
		ok = WriteReg(CS47L63_REG_MICBIAS1, 0) && ok;
		ok = WriteReg(CS47L63_REG_LDO2, 0) && ok;
		if (Input == CS47L63_INPUT_LINE)
		{
			ok = WriteReg(CS47L63_REG_INPUT2, 0x00050020U) && ok;
			ok = WriteReg(CS47L63_REG_IN2L_SOURCE, 0x10000000U) && ok;
			ok = WriteReg(CS47L63_REG_IN2R_SOURCE, 0x10000000U) && ok;
			ok = WriteReg(CS47L63_REG_IN2L_VOLUME, 0x00800080U) && ok;
			ok = WriteReg(CS47L63_REG_IN2R_VOLUME, 0x00800080U) && ok;
			ok = WriteReg(CS47L63_REG_INPUT_ENABLE, 0x000FU) && ok;
			ok = WriteReg(CS47L63_REG_INPUT_VU, 0x20000000U) && ok;
			ok = WriteReg(CS47L63_REG_ASP1TX1_INPUT, 0x00800012U) && ok;
			ok = WriteReg(CS47L63_REG_ASP1TX2_INPUT, 0x00800013U) && ok;
		}
	}
	if (ok)
	{
		vAudioCfg.Input = Input;
	}
	return ok;
}

bool Cs47l63::HeadphoneGain(int MilliDb)
{
	uint32_t volume;

	if (!Device::Valid() || !vbEnabled ||
		MilliDb < -64000 || MilliDb > 31500 || MilliDb % 500 != 0)
	{
		return false;
	}
	volume = uint32_t(128 + MilliDb / 500);
	volume |= 0x200U | (vbMuted ? 0x100U : 0U);
	if (!WriteReg(CS47L63_REG_OUT_VOLUME, volume))
	{
		return false;
	}

	vAudioCfg.HeadphoneGainMilliDb = MilliDb;
	return true;
}

bool Cs47l63::HeadphoneMute(bool Mute)
{
	uint32_t volume;

	if (!Device::Valid() || !vbEnabled || !vbAudioConfigured ||
		!vAudioCfg.HeadphoneEnable)
	{
		return false;
	}
	volume = uint32_t(128 + vAudioCfg.HeadphoneGainMilliDb / 500);
	volume |= 0x200U | (Mute ? 0x100U : 0);
	if (!WriteReg(CS47L63_REG_OUT_VOLUME, volume))
	{
		return false;
	}
	vbMuted = Mute;
	return true;
}

bool Cs47l63::ConfigureAudio(const Cs47l63AudioCfg_t &Cfg)
{
	uint32_t readback;
	bool ok;

	if (!Device::Valid() || !vbEnabled ||
		Cfg.SampleRate != 48000 ||
		(Cfg.Input != CS47L63_INPUT_OFF &&
		 Cfg.Input != CS47L63_INPUT_PDM &&
		 Cfg.Input != CS47L63_INPUT_LINE) ||
		(Cfg.Input == CS47L63_INPUT_OFF && !Cfg.HeadphoneEnable) ||
		Cfg.HeadphoneGainMilliDb < -64000 ||
		Cfg.HeadphoneGainMilliDb > 31500 ||
		Cfg.HeadphoneGainMilliDb % 500 != 0)
	{
		return false;
	}

	if (vbAudioConfigured && !StopAudio())
	{
		return false;
	}
	vbMuted = true;
	vAudioCfg = Cfg;
	vbAudioConfigured = false;

	// ASP1_BCLK = 48 kHz x 32 bits per stereo frame = 1.536 MHz.
	// FLL1 configuration follows the Cirrus/Nordic Audio DK 48-kHz profile.
	static const Cs47l63RegVal_t clocks[] = {
		{ CS47L63_REG_SAMPLE_RATE1, 0x0003U, 0 },
		{ CS47L63_REG_SAMPLE_RATE2, 0, 0 },
		{ CS47L63_REG_SAMPLE_RATE3, 0, 0 },
		{ CS47L63_REG_SAMPLE_RATE4, 0, 0 },
		{ CS47L63_REG_FLL1_CTRL2, 0x88200008U, 0 },
		{ CS47L63_REG_FLL1_CTRL3, 0x00010000U, 0 },
		{ CS47L63_REG_FLL1_GPIO_CLOCK, 0x0005U, 0 },
		{ CS47L63_REG_FLL1_CTRL1, 0, 1 },
		{ CS47L63_REG_FLL1_CTRL1, 1, 2 },
		{ CS47L63_REG_SYSCLK, 0x034CU, 0 },
		{ CS47L63_REG_ASYNCCLK, 0x034CU, 0 },
	};
	static const Cs47l63RegVal_t asp[] = {
		{ CS47L63_REG_GPIO1 + 0x00U, 0x61000000U, 0 },
		{ CS47L63_REG_GPIO1 + 0x04U, 0xE1000000U, 0 },
		{ CS47L63_REG_GPIO1 + 0x08U, 0xE1000000U, 0 },
		{ CS47L63_REG_GPIO1 + 0x0CU, 0xE1000000U, 0 },
		{ CS47L63_REG_GPIO1 + 0x10U, 0x61000001U, 0 },
		{ CS47L63_REG_ASP1_CTRL2, 0x10100200U, 0 },
		{ CS47L63_REG_ASP1_CTRL3, 0x00000000U, 0 },
		{ CS47L63_REG_ASP1_TX_DATA, 0x0010U, 0 },
		{ CS47L63_REG_ASP1_RX_DATA, 0x0010U, 0 },
		{ CS47L63_REG_ASP1_ENABLE, 0x30003U, 0 }
	};

	ok = Apply(clocks, sizeof(clocks) / sizeof(clocks[0]));
	ok = ok && Apply(asp, sizeof(asp) / sizeof(asp[0]));
	ok = ok && SelectInput(Cfg.Input);
	ok = ok && WriteReg(CS47L63_REG_OUT_ENABLE, 0);
	if (Cfg.HeadphoneEnable)
	{
		// The mono codec mixes one I2S RX slot. Do not sum both slots at 0 dB.
		ok = ok && WriteReg(CS47L63_REG_OUT1L_INPUT1, 0x00800020U);
		ok = ok && WriteReg(CS47L63_REG_OUT1L_INPUT2, 0);
		ok = ok && HeadphoneGain(Cfg.HeadphoneGainMilliDb);
		ok = ok && WriteReg(CS47L63_REG_OUT_ENABLE, 0x0002U);
	}
	else
	{
		ok = ok && WriteReg(CS47L63_REG_OUT1L_INPUT1, 0);
		ok = ok && WriteReg(CS47L63_REG_OUT1L_INPUT2, 0);
	}
	if (ok)
	{
		ok = ReadReg(CS47L63_REG_SAMPLE_RATE1, readback) &&
			readback == 3U;
	}
	if (ok)
	{
		ok = ReadReg(CS47L63_REG_ASP1_CTRL2, readback) &&
			readback == 0x10100200U;
	}
	if (ok)
	{
		ok = ReadReg(CS47L63_REG_ASP1_ENABLE, readback) &&
			readback == 0x30003U;
	}
	if (!ok)
	{
		vbAudioConfigured = true;
		StopAudio();
		return false;
	}
	vbAudioConfigured = true;
	if (Cfg.HeadphoneEnable && !HeadphoneMute(false))
	{
		StopAudio();
		return false;
	}
	vbAudioSaved = true;
	return true;
}

bool Cs47l63::ReadInterrupts(uint32_t &Faults, uint32_t &FllEvents)
{
	if (!Device::Valid() || !vbEnabled)
	{
		return false;
	}
	if (!ReadReg(CS47L63_REG_IRQ_EINT1, Faults) ||
		!ReadReg(CS47L63_REG_IRQ_EINT6, FllEvents))
	{
		return false;
	}
	return true;
}
