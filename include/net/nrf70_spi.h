/**-------------------------------------------------------------------------
@file nrf70_spi.h
@brief Low-level nRF7002 SPI command transport using IOsonata DeviceIntrf.
@license MIT License
Copyright (c) 2026 I-SYST inc.

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
-------------------------------------------------------------------------*/
#ifndef __NRF70_SPI_H__
#define __NRF70_SPI_H__
#include <stdint.h>
#include <stddef.h>
#include "device_intrf.h"

/*
 * SPI commands follow Nordic's nRF70 bus protocol, not serial NOR flash.
 * No power sequencing or firmware loading is performed here.
 */
class Nrf70Spi
{
public:
	Nrf70Spi() : vpSpi(nullptr), vSelect(0), vLatency(0), vErrors(0)
	{
	}
	bool Init(DeviceIntrf * const pSpi, uint32_t ChipSelect, uint8_t SlaveLatency = 0);
	bool ReadStatus(uint8_t Opcode, uint8_t &Value);
	bool WriteStatus(uint8_t Opcode, uint8_t Value);
	bool Read(uint32_t Address, void *pData, uint32_t Length);
	bool Write(uint32_t Address, const void *pData, uint32_t Length);
	bool ReadHighLatency(uint32_t Address, void *pData, uint32_t Length);
	bool ReadRegister32(uint32_t Address, uint32_t &Value);
	bool WriteRegister32(uint32_t Address, uint32_t Value);
	uint32_t Errors() const
	{
		return vErrors;
	}

private:
	bool ReadBlock(uint32_t Address, void *pData, uint32_t Length, uint32_t Discard);
	DeviceIntrf *vpSpi;
	uint32_t vSelect;
	uint8_t vLatency;
	uint32_t vErrors;
};
#ifdef __cplusplus
extern "C" {
#endif
/* Matches the SPI members of Nordic nrf_wifi_osal_ops. Pass Nrf70Spi* as priv.
 * The callbacks cannot report an error, so use Nrf70Spi::Errors() for
 * diagnostics and fail host initialization on its first increment. */
unsigned int Nrf70OsalSpiRead32(void *priv, unsigned long addr);
void Nrf70OsalSpiWrite32(void *priv, unsigned long addr, unsigned int value);
void Nrf70OsalSpiCopyFrom(void *priv, void *dest, unsigned long addr, size_t count);
void Nrf70OsalSpiCopyTo(void *priv, unsigned long addr, const void *src, size_t count);
#ifdef __cplusplus
}
#endif
#endif
