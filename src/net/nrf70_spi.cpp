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
		return false;
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
	return vpSpi->Tx(vSelect, command, sizeof(command)) == sizeof(command);
}

bool Nrf70Spi::Write(uint32_t Address, const void *pData, uint32_t Length)
{
	uint8_t cmd[4];

	if (vpSpi == nullptr || pData == nullptr || Length == 0 ||
		Address > 0x7FFFFF || Length > 0x7FFFFF - Address)
		return false;
	cmd[0] = 0x02;
	cmd[1] = (uint8_t)((Address >> 16) | 0x80);
	cmd[2] = (uint8_t)(Address >> 8);
	cmd[3] = (uint8_t)Address;
	// DeviceIntrf::Write sends the command and payload under one CS.
	return vpSpi->Write(vSelect, cmd, sizeof(cmd),
		(const uint8_t *)pData, (int)Length) == (int)Length;
}

bool Nrf70Spi::ReadBlock(uint32_t Address, void *pData, uint32_t Length,
	uint32_t Discard)
{
	uint8_t cmd[5];

	if (vpSpi == nullptr || pData == nullptr || Length == 0 ||
		Address > 0x7FFFFF || Length > 0x7FFFFF - Address ||
		Discard > 8)
		return false;
	cmd[0] = 0x0B;
	cmd[1] = (uint8_t)(Address >> 16);
	cmd[2] = (uint8_t)(Address >> 8);
	cmd[3] = (uint8_t)Address;
	cmd[4] = 0;
	// High-latency reads include four dummy bytes per slave latency step.
	// The interface must support an atomic read transaction with dummy clocks.
	if (Discard == 0)
		return vpSpi->Read(vSelect, cmd, sizeof(cmd),
			(uint8_t *)pData, (int)Length) == (int)Length;
	// A separate transaction would deassert CS. Do not issue one here.
	// The host port must supply a streaming SPI transfer for that case.
	return false;
}

bool Nrf70Spi::Read(uint32_t Address, void *pData, uint32_t Length)
{
	return ReadBlock(Address, pData, Length, 0);
}

bool Nrf70Spi::ReadHighLatency(uint32_t Address, void *pData, uint32_t Length)
{
	return ReadBlock(Address, pData, Length, 4 * vLatency);
}
