/**-------------------------------------------------------------------------
@file nrf70_spi.h
@brief Low-level nRF7002 SPI command transport using IOsonata DeviceIntrf.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __NRF70_SPI_H__
#define __NRF70_SPI_H__
#include <stdint.h>
#include "device_intrf.h"

/*
 * SPI commands follow Nordic's nRF70 bus protocol, not serial NOR flash.
 * No power sequencing or firmware loading is performed here.
 */
class Nrf70Spi {
public:
	Nrf70Spi() : vpSpi(nullptr), vSelect(0), vLatency(0) {}
	bool Init(DeviceIntrf * const pSpi, uint32_t ChipSelect, uint8_t SlaveLatency = 0);
	bool ReadStatus(uint8_t Opcode, uint8_t &Value);
	bool WriteStatus(uint8_t Opcode, uint8_t Value);
	bool Read(uint32_t Address, void *pData, uint32_t Length);
	bool Write(uint32_t Address, const void *pData, uint32_t Length);
	bool ReadHighLatency(uint32_t Address, void *pData, uint32_t Length);

private:
	bool ReadBlock(uint32_t Address, void *pData, uint32_t Length, uint32_t Discard);
	DeviceIntrf *vpSpi;
	uint32_t vSelect;
	uint8_t vLatency;
};
#endif
