/**-------------------------------------------------------------------------
@file	cs47l63.cpp
@brief	Cirrus Logic CS47L63 control-interface driver
@author	Hoang Nguyen Hoan
@date	Oct. 24, 2026
@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
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
#include "coredev/iopinctrl.h"
#include "idelay.h"

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

	count = vCfg.BootTimeoutMs;
	while (count > 0)
	{
		if (IrqActive())
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

	return Boot();
}

void Cs47l63::Disable()
{
	if (!Device::Valid())
	{
		return;
	}

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
	if (!result)
	{
		Valid(false);
		vbEnabled = false;
	}
}

void Cs47l63::PowerOff()
{
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
