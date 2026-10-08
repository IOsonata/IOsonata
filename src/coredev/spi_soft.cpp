/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc. All rights reserved.
 * Portable GPIO SPI master using IOsonata GPIO, delay and DeviceIntrf hooks.
 */
#include <limits.h>
#include "coredev/spi_soft.h"
#include "iopinctrl.h"
#include "idelay.h"

static SPISoftDev_t *SoftDev(DevIntrf_t *pIntrf)
{
	return static_cast<SPISoftDev_t *>(pIntrf->pDevData);
}

static bool Present(const IOPinCfg_t &Pin)
{
	return Pin.PortNo >= 0 && Pin.PinNo >= 0;
}

static void Output(const IOPinCfg_t &Pin, bool High)
{
	if (High)
		IOPinSet(Pin.PortNo, Pin.PinNo);
	else
		IOPinClear(Pin.PortNo, Pin.PinNo);
}

static uint32_t HalfPeriod(uint32_t Rate)
{
	return Rate ? 500000U / Rate + (500000U % Rate != 0U) : 0U;
}

static uint32_t GetRate(DevIntrf_t *pIntrf)
{
	return SoftDev(pIntrf)->Spi.Cfg.Rate;
}

static uint32_t SetRate(DevIntrf_t *pIntrf, uint32_t Rate)
{
	SPISoftDev_t *dev = SoftDev(pIntrf);
	if (!Rate || dev->Spi.CurDevCs >= 0)
		return 0;
	dev->HalfPeriodUs = HalfPeriod(Rate);
	dev->Spi.Cfg.Rate = 500000U / dev->HalfPeriodUs;
	return dev->Spi.Cfg.Rate;
}

static void Configure(SPISoftDev_t *dev)
{
	const SPICfg_t &cfg = dev->Spi.Cfg;
	for (int i = 0; i < cfg.NbIOPins; ++i)
	{
		if (i >= SPI_CS_IOPIN_IDX && cfg.ChipSel == SPICSEL_MAN)
			continue; // Application owns manual CS, including configuration.
		const IOPinCfg_t &pin = cfg.pIOPinMap[i];
		if (!Present(pin))
			continue;
		const bool input = i == SPI_MISO_IOPIN_IDX;
		if (!input)
			Output(pin, i >= SPI_CS_IOPIN_IDX ||
				(i == SPI_SCK_IOPIN_IDX && cfg.ClkPol == SPICLKPOL_LOW));
		IOPinConfig(pin.PortNo, pin.PinNo, IOPINOP_GPIO,
			input ? IOPINDIR_INPUT : IOPINDIR_OUTPUT, pin.Res, pin.Type);
	}
	dev->Enabled = true;
}

static bool Start(DevIntrf_t *pIntrf, uint32_t DevCs)
{
	SPISoftDev_t *dev = SoftDev(pIntrf);
	SPIDev_t &spi = dev->Spi;
	if (!dev->Enabled || DevCs > INT_MAX ||
		(spi.Cfg.ChipSel == SPICSEL_AUTO &&
		 DevCs >= static_cast<uint32_t>(spi.Cfg.NbIOPins - SPI_CS_IOPIN_IDX)))
		return false;
	// Generic Read calls the direction-change hook while already selected.
	if (spi.CurDevCs >= 0)
		return spi.CurDevCs == static_cast<int>(DevCs);
	spi.CurDevCs = static_cast<int>(DevCs);
	spi.FirstRdData = -1;
	Output(spi.Cfg.pIOPinMap[SPI_SCK_IOPIN_IDX], spi.Cfg.ClkPol == SPICLKPOL_LOW);
	if (spi.Cfg.ChipSel == SPICSEL_AUTO)
		Output(spi.Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + DevCs], false);
	usDelay(dev->HalfPeriodUs);
	return true;
}

static void Stop(DevIntrf_t *pIntrf)
{
	SPISoftDev_t *dev = SoftDev(pIntrf);
	SPIDev_t &spi = dev->Spi;
	if (spi.CurDevCs < 0)
		return;
	usDelay(dev->HalfPeriodUs);
	if (spi.Cfg.ChipSel == SPICSEL_AUTO)
		Output(spi.Cfg.pIOPinMap[SPI_CS_IOPIN_IDX + spi.CurDevCs], true);
	spi.CurDevCs = -1;
	pIntrf->bTxReady = true;
}

static int TransferData(DevIntrf_t *pIntrf, const uint8_t *pTx, uint8_t *pRx, int Length)
{
	SPISoftDev_t *dev = SoftDev(pIntrf);
	SPIDev_t &spi = dev->Spi;
	const SPICfg_t &cfg = spi.Cfg;
	const int step = cfg.DataSize > 8 ? 2 : 1;
	const bool shared = cfg.Phy == SPIPHY_3WIRE;
	const IOPinCfg_t &sck = cfg.pIOPinMap[SPI_SCK_IOPIN_IDX];
	const IOPinCfg_t &mosi = cfg.pIOPinMap[SPI_MOSI_IOPIN_IDX];
	const IOPinCfg_t &miso = cfg.pIOPinMap[shared ? SPI_MOSI_IOPIN_IDX : SPI_MISO_IOPIN_IDX];
	if (!dev->Enabled || spi.CurDevCs < 0 || Length <= 0 || Length % step ||
		(!pTx && !pRx) || (pTx && !Present(mosi)) || (pRx && !Present(miso)) ||
		(shared && pTx && pRx))
		return 0;
	if (shared)
		IOPinSetDir(mosi.PortNo, mosi.PinNo, pRx ? IOPINDIR_INPUT : IOPINDIR_OUTPUT);
	const bool idle = cfg.ClkPol == SPICLKPOL_LOW;
	const bool phase = cfg.DataPhase == SPIDATAPHASE_SECOND_CLK;
	const bool drive = Present(mosi) && !(shared && pRx);
	pIntrf->bTxReady = false;
	for (int offset = 0; offset < Length; offset += step)
	{
		uint16_t tx = pTx ? pTx[offset] : cfg.DummyByte;
		if (step == 2)
			tx |= static_cast<uint16_t>(pTx ? pTx[offset + 1] : cfg.DummyByte) << 8;
		uint16_t rx = 0;
		for (unsigned bit = 0; bit < cfg.DataSize; ++bit)
		{
			const unsigned shift = cfg.BitOrder == SPIDATABIT_MSB ? cfg.DataSize - bit - 1 : bit;
			if (phase)
				Output(sck, !idle);
			if (drive)
				Output(mosi, (tx & (1U << shift)) != 0);
			usDelay(dev->HalfPeriodUs);
			// Keep the sampling edge and GPIO read together. Delays and the
			// rest of the transfer leave interrupts in the caller's state.
			const auto state = DisableInterrupt();
			Output(sck, phase ? idle : !idle);
			if (Present(miso) && (!shared || pRx))
				rx |= static_cast<uint16_t>(IOPinRead(miso.PortNo, miso.PinNo) != 0) << shift;
			EnableInterrupt(state);
			usDelay(dev->HalfPeriodUs);
			if (!phase)
				Output(sck, idle);
		}
		if (spi.FirstRdData < 0)
			spi.FirstRdData = rx;
		if (pRx)
		{
			pRx[offset] = static_cast<uint8_t>(rx);
			if (step == 2)
				pRx[offset + 1] = static_cast<uint8_t>(rx >> 8);
		}
	}
	pIntrf->bTxReady = true;
	return Length;
}

static int Tx(DevIntrf_t *pIntrf, const uint8_t *pData, int Length)
{
	return pData ? TransferData(pIntrf, pData, nullptr, Length) : 0;
}
static int Rx(DevIntrf_t *pIntrf, uint8_t *pData, int Length)
{
	return pData ? TransferData(pIntrf, nullptr, pData, Length) : 0;
}
static void Enable(DevIntrf_t *pIntrf) { Configure(SoftDev(pIntrf)); }
static void Disable(DevIntrf_t *pIntrf)
{
	if (SoftDev(pIntrf)->Spi.CurDevCs >= 0)
		DeviceIntrfStopTx(pIntrf);
	SoftDev(pIntrf)->Enabled = false;
}
static void Reset(DevIntrf_t *pIntrf)
{
	if (SoftDev(pIntrf)->Spi.CurDevCs >= 0)
		DeviceIntrfStopTx(pIntrf);
	if (SoftDev(pIntrf)->Enabled)
		Configure(SoftDev(pIntrf));
}
static void PowerOff(DevIntrf_t *pIntrf)
{
	Disable(pIntrf);
	const SPICfg_t &cfg = SoftDev(pIntrf)->Spi.Cfg;
	for (int i = 0; i < cfg.NbIOPins; ++i)
		if (Present(cfg.pIOPinMap[i]) && (i < SPI_CS_IOPIN_IDX || cfg.ChipSel == SPICSEL_AUTO))
			IOPinDisable(cfg.pIOPinMap[i].PortNo, cfg.pIOPinMap[i].PinNo);
	pIntrf->EnCnt = 0;
}
static void *GetHandle(DevIntrf_t *pIntrf) { return &SoftDev(pIntrf)->Spi; }

bool SPISoftInit(SPISoftDev_t * const pDev, const SPICfg_t *pCfg)
{
	if (!pDev || !pCfg || pCfg->Mode != SPIMODE_MASTER ||
		(pCfg->Phy != SPIPHY_NORMAL && pCfg->Phy != SPIPHY_3WIRE) ||
		pCfg->bDmaEn || pCfg->bIntEn || !pCfg->Rate || pCfg->MaxRetry < 0 ||
		pCfg->DataSize < 4 || pCfg->DataSize > 16 ||
		(pCfg->BitOrder != SPIDATABIT_MSB && pCfg->BitOrder != SPIDATABIT_LSB) ||
		(pCfg->ClkPol != SPICLKPOL_HIGH && pCfg->ClkPol != SPICLKPOL_LOW) ||
		(pCfg->DataPhase != SPIDATAPHASE_FIRST_CLK && pCfg->DataPhase != SPIDATAPHASE_SECOND_CLK) ||
		(pCfg->ChipSel != SPICSEL_AUTO && pCfg->ChipSel != SPICSEL_MAN) ||
		!pCfg->pIOPinMap || pCfg->NbIOPins < 3 ||
		(pCfg->ChipSel == SPICSEL_AUTO && pCfg->NbIOPins < 4))
		return false;
	const IOPinCfg_t *pins = pCfg->pIOPinMap;
	if (!Present(pins[0]) || (!Present(pins[1]) && !Present(pins[2])) ||
		(pCfg->Phy == SPIPHY_3WIRE && (Present(pins[1]) || !Present(pins[2]))))
		return false;
	for (int i = 0; i < pCfg->NbIOPins; ++i)
	{
		if (i >= SPI_CS_IOPIN_IDX && pCfg->ChipSel == SPICSEL_MAN)
			continue;
		if (pins[i].PortNo == -1 && pins[i].PinNo == -1 && (i == 1 || i == 2))
			continue;
		if (!Present(pins[i]) || pins[i].PinOp != IOPINOP_GPIO)
			return false;
		for (int j = 0; j < i; ++j)
			if (Present(pins[j]) && pins[i].PortNo == pins[j].PortNo && pins[i].PinNo == pins[j].PinNo)
				return false;
	}
	pDev->Spi.Cfg = *pCfg;
	pDev->Spi.CurDevCs = -1;
	pDev->Spi.FirstRdData = -1;
	DevIntrf_t *intrf = &pDev->Spi.DevIntrf;
	intrf->pDevData = pDev;
	intrf->Type = DEVINTRF_TYPE_SPI;
	intrf->IntPrio = pCfg->IntPrio;
	intrf->EvtCB = pCfg->EvtCB;
	intrf->MaxRetry = pCfg->MaxRetry;
	intrf->EnCnt = 1;
	intrf->bDma = intrf->bIntEn = false;
	intrf->bTxReady = true;
	intrf->bNoStop = false;
	intrf->Disable = Disable;
	intrf->Enable = Enable;
	intrf->GetRate = GetRate;
	intrf->SetRate = SetRate;
	intrf->StartRx = intrf->StartTx = Start;
	intrf->StopRx = intrf->StopTx = Stop;
	intrf->RxData = Rx;
	intrf->TxData = intrf->TxSrData = Tx;
	intrf->Reset = Reset;
	intrf->PowerOff = PowerOff;
	intrf->GetHandle = GetHandle;
	atomic_flag_clear(&intrf->bBusy);
	SetRate(intrf, pCfg->Rate);
	Configure(pDev);
	return true;
}

int SPISoftTransfer(SPISoftDev_t * const pDev, uint32_t DevCs,
					const uint8_t *pTx, uint8_t *pRx, int Length)
{
	if (!pDev || pDev->Spi.DevIntrf.pDevData != pDev || Length <= 0 ||
		(!pTx && !pRx) || (pDev->Spi.Cfg.Phy == SPIPHY_3WIRE && pTx && pRx) ||
		(pDev->Spi.Cfg.DataSize > 8 && (Length & 1)))
		return 0;
	DevIntrf_t *intrf = &pDev->Spi.DevIntrf;
	if (!DeviceIntrfStartTx(intrf, DevCs))
		return 0;
	int count = TransferData(intrf, pTx, pRx, Length);
	DeviceIntrfStopTx(intrf);
	return count;
}
