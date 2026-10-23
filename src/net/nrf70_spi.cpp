/**-------------------------------------------------------------------------
@file nrf70_spi.cpp
@brief nRF70 SPI transport over IOsonata DeviceIntrf.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "net/nrf70_spi.h"

bool Nrf70Spi::Init(DeviceIntrf * const pSpi, uint32_t ChipSelect, uint8_t SlaveLatency)
{
	if (pSpi == nullptr || pSpi->Type() != DEVINTRF_TYPE_SPI || SlaveLatency > 2)
		return false;
	vpSpi = pSpi;
	vSelect = ChipSelect;
	vLatency = SlaveLatency;
	vErrors = 0;
	return true;
}

bool Nrf70Spi::ReadStatus(uint8_t Opcode, uint8_t &Value)
{
	uint8_t command[1];
	uint8_t result[5];

	if (vpSpi == nullptr)
		return false;
	command[0] = Opcode;
	// Nordic returns the register byte immediately after the opcode;
	// subsequent clocks are required by some SPI controllers.
	if (vpSpi->Read(vSelect, command, 1, result, sizeof(result)) != sizeof(result))
	{
		vErrors++;
		return false;
	}
	Value = result[0];
	return true;
}

bool Nrf70Spi::WriteStatus(uint8_t Opcode, uint8_t Value)
{
	uint8_t command[2];

	if (vpSpi == nullptr)
		return false;
	command[0] = Opcode;
	command[1] = Value;
	if (vpSpi->Tx(vSelect, command, sizeof(command)) != sizeof(command))
	{
		vErrors++;
		return false;
	}
	return true;
}

bool Nrf70Spi::Write(uint32_t Address, const void *pData, uint32_t Length)
{
	uint8_t cmd[4];

	if (vpSpi == nullptr || pData == nullptr || Length == 0 ||
		Address > 0x7FFFFF || Length - 1 > 0x7FFFFF - Address)
		return false;
	cmd[0] = 0x02;
	cmd[1] = (uint8_t)((Address >> 16) | 0x80);
	cmd[2] = (uint8_t)(Address >> 8);
	cmd[3] = (uint8_t)Address;
	// DeviceIntrf::Write sends the command and payload under one CS.
	if (vpSpi->Write(vSelect, cmd, sizeof(cmd),
		(const uint8_t *)pData, (int)Length) != (int)Length)
	{
		vErrors++;
		return false;
	}
	return true;
}

bool Nrf70Spi::ReadBlock(uint32_t Address, void *pData, uint32_t Length,
	uint32_t Discard)
{
	uint8_t cmd[13] = { 0 };
	int cmdLen;

	if (vpSpi == nullptr || pData == nullptr || Length == 0 ||
		Address > 0x7FFFFF || Length > 0x7FFFFF - Address ||
		Discard > 8 || Length > 0x7FFFFFFF)
		return false;

	cmd[0] = 0x0B;
	cmd[1] = (uint8_t)(Address >> 16);
	cmd[2] = (uint8_t)(Address >> 8);
	cmd[3] = (uint8_t)Address;
	cmdLen = 5 + (int)Discard;

	// The complete header, dummy byte and latency clocks are issued
	// in the command phase of one DeviceIntrf::Read transaction.
	// CS stays asserted until the following receive phase completes.
	if (vpSpi->Read(vSelect, cmd, cmdLen,
		(uint8_t *)pData, (int)Length) != (int)Length)
	{
		vErrors++;
		return false;
	}
	return true;
}

bool Nrf70Spi::Read(uint32_t Address, void *pData, uint32_t Length)
{
	return ReadBlock(Address, pData, Length, 0);
}

bool Nrf70Spi::ReadHighLatency(uint32_t Address, void *pData, uint32_t Length)
{
	return ReadBlock(Address, pData, Length, 4 * vLatency);
}

bool Nrf70Spi::ReadRegister32(uint32_t Address, uint32_t &Value)
{
	uint8_t data[4];
	if (!ReadHighLatency(Address, data, sizeof(data)))
	{
		return false;
	}
	Value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
	return true;
}

bool Nrf70Spi::WriteRegister32(uint32_t Address, uint32_t Value)
{
	uint8_t data[4];
	data[0] = (uint8_t)Value;
	data[1] = (uint8_t)(Value >> 8);
	data[2] = (uint8_t)(Value >> 16);
	data[3] = (uint8_t)(Value >> 24);
	return Write(Address, data, sizeof(data));
}

extern "C" unsigned int Nrf70OsalSpiRead32(void *priv, unsigned long addr)
{
	Nrf70Spi *spi = (Nrf70Spi *)priv;
	uint32_t value = 0;
	if (spi == nullptr || addr > 0x7FFFFF)
		return 0;
	(void)spi->ReadRegister32((uint32_t)addr, value);
	return value;
}

extern "C" void Nrf70OsalSpiWrite32(void *priv, unsigned long addr, unsigned int value)
{
	Nrf70Spi *spi = (Nrf70Spi *)priv;
	if (spi != nullptr && addr <= 0x7FFFFF)
		(void)spi->WriteRegister32((uint32_t)addr, value);
}

extern "C" void Nrf70OsalSpiCopyFrom(void *priv, void *dest,
	unsigned long addr, size_t count)
{
	Nrf70Spi *spi = (Nrf70Spi *)priv;
	if (spi == nullptr || dest == nullptr || addr > 0x7FFFFF ||
		count > 0x7FFFFF || count == 0)
		return;
	(void)spi->ReadHighLatency((uint32_t)addr, dest, (uint32_t)count);
}

extern "C" void Nrf70OsalSpiCopyTo(void *priv, unsigned long addr,
	const void *src, size_t count)
{
	Nrf70Spi *spi = (Nrf70Spi *)priv;
	if (spi == nullptr || src == nullptr || addr > 0x7FFFFF ||
		count > 0x7FFFFF || count == 0)
		return;
	(void)spi->Write((uint32_t)addr, src, (uint32_t)count);
}
