/**-------------------------------------------------------------------------
@file seep_24cw160.h
@brief Microchip 24CW160 (16-Kbit) serial EEPROM configuration.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#ifndef __SEEP_24CW160_H__
#define __SEEP_24CW160_H__

#include "storage/seep.h"

/*
 * 24CW160 uses a fixed two-byte word address. Unlike legacy 24C16 parts,
 * its upper address bits are NOT encoded into the I2C device address.
 * Part suffix 0 selects client address 0x50; suffixes 1..7 select 0x51..0x57.
 *
 * Register access to its software protection/HAR configuration is deliberately
 * not exposed here: those registers can permanently affect EEPROM access.
 */
#define SEEP_24CW160_BASE_ADDR       0x50U
#define SEEP_24CW160_SIZE           2048U
#define SEEP_24CW160_PAGE_SIZE      32U
#define SEEP_24CW160_ADDRESS_BYTES  2U
#define SEEP_24CW160_WRITE_DELAY_MS 5U

/**
 * Configure a 24CW160 (without changing any persistent protection registers).
 * ClientAddress is the 7-bit I2C client address, 0x50 through 0x57.
 * Use the board's confirmed client address; standard 24CW160 is 0x50.
 */
static inline SeepCfg_t Seep24cw160Config(uint8_t ClientAddress)
{
	SeepCfg_t cfg = {};

	cfg.DevAddr = ClientAddress;
	cfg.AddrLen = SEEP_24CW160_ADDRESS_BYTES;
	cfg.PageSize = SEEP_24CW160_PAGE_SIZE;
	cfg.Size = SEEP_24CW160_SIZE;
	cfg.WrDelay = SEEP_24CW160_WRITE_DELAY_MS;
	cfg.WrProtPin.PortNo = -1;
	cfg.WrProtPin.PinNo = -1;
	cfg.pInitCB = NULL;
	cfg.pWaitCB = NULL;
	return cfg;
}

static inline bool Seep24cw160Init(Seep &Eeprom,
	DeviceIntrf * const pI2c, uint8_t ClientAddress = SEEP_24CW160_BASE_ADDR)
{
	if (pI2c == NULL || ClientAddress < 0x50 || ClientAddress > 0x57)
		return false;

	SeepCfg_t cfg = Seep24cw160Config(ClientAddress);

	return Eeprom.Init(cfg, pI2c);
}
#endif
