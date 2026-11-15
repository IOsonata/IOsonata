// SAMD11 SPI mode/lifecycle tests. Register flags are supplied explicitly;
// this is not a SERCOM timing or interrupt-priority simulation.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#define _UINT32_(x) ((uint32_t)(x))
#define _UINT16_(x) ((uint32_t)(x))
#define _UINT8_(x) ((uint32_t)(x))
#define __I volatile
#define __O volatile
#define __IO volatile
#include "component/sercom.h"
#include "component/dmac.h"

#define SAMD_SERCOM_CNT 3
#define SAMD_GCLK_MAIN 0
#define SAMD_DMAC_COUNT_MAX 65535
#define SAMD_DMAC_TRIG_SERCOM_RX(n) (1 + 2 * (n))
#define SAMD_DMAC_TRIG_SERCOM_TX(n) (2 + 2 * (n))
typedef int IRQn_Type;
typedef void (*SamdDmacHandler_t)(int, uint32_t, void *);

static sercom_registers_t s_Regs[3];
static IrqHandler_t s_Handler;
static DevIntrf_t *s_Intrf;
static bool s_Channels[2];
static SamdDmacHandler_t s_DmaHandler[2];
static void *s_DmaCtx[2];
static unsigned s_Opens, s_Closes, s_Stops, s_Starts, s_Events, s_Arms;
static bool s_IrqEnabled, s_Clock, s_FailTx;
static int s_EventLen;
static uint32_t s_DmaLeft[2];
static uint8_t s_SlaveTx[] = {0x31, 0x32};
static uint8_t s_SlaveRx[2];
static const IOPinCfg_t s_MasterPins[] = {
	{0, 1}, {0, 3}, {0, 0}, {0, 2}
};
static const IOPinCfg_t s_SlavePins[] = {
	{0, 1}, {0, 0}, {0, 3}, {0, 2}
};

static void NVIC_DisableIRQ(int);
static void NVIC_EnableIRQ(int);
static void NVIC_ClearPendingIRQ(int);
static void NVIC_SetPriority(int, uint32_t);
static int SamdSercomIrq(int DevNo);
static uint32_t SamdIrqPrio(int Prio);
static bool SamdSercomOwned(int, const DevIntrf_t *);
static bool SamdSercomClockEnable(int);
static void SamdSercomClockDisable(int);
static sercom_registers_t *SamdSercomGetReg(int DevNo);
static int SamdSercomPad(int, const IOPinCfg_t *pPin);
static uint32_t SystemPeriphClockGet(int);
static void IOPinSet(int, int);
static void IOPinClear(int, int);
static int IOPinRead(int, int);
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE);
void IOPinDisable(int, int);
void SharedIntrfSetIrqHandler(int, DevIntrf_t *pIntrf, IrqHandler_t Handler);
void DeviceIntrfTxComplete(DevIntrf_t * const pDev);
static int SamdDmacOpen(int, SamdDmacHandler_t Handler, void *pCtx, int);
static void SamdDmacClose(int Chan);
static uint32_t SamdDmacStop(int Chan);
static bool SamdDmacBusy(int);
static void SamdDmacStart(int Chan, const volatile void *, bool, volatile void *, bool, uint32_t);

#include "spi_driver.h"

static void NVIC_DisableIRQ(int)
{
	s_IrqEnabled = false;
}
static void NVIC_EnableIRQ(int)
{
	s_IrqEnabled = true;
}
static void NVIC_ClearPendingIRQ(int)
{
}
static void NVIC_SetPriority(int, uint32_t)
{
}
static int SamdSercomIrq(int DevNo)
{
	return DevNo;
}
static uint32_t SamdIrqPrio(int Prio)
{
	return Prio > 3 ? 3 : Prio;
}
static bool SamdSercomOwned(int, const DevIntrf_t *)
{
	return false;
}
static bool SamdSercomClockEnable(int)
{
	s_Clock = true;
	return true;
}
static void SamdSercomClockDisable(int)
{
	s_Clock = false;
}
static sercom_registers_t *SamdSercomGetReg(int DevNo)
{
	return &s_Regs[DevNo];
}
static int SamdSercomPad(int, const IOPinCfg_t *pPin)
{
	return pPin->PinNo;
}
static uint32_t SystemPeriphClockGet(int)
{
	return 48000000;
}
static void IOPinSet(int, int)
{
}
static void IOPinClear(int, int)
{
}
static int IOPinRead(int, int)
{
	return 1;
}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}
void IOPinDisable(int, int)
{
}
void SharedIntrfSetIrqHandler(int, DevIntrf_t *pIntrf, IrqHandler_t Handler)
{
	s_Intrf = pIntrf;
	s_Handler = Handler;
}
void DeviceIntrfTxComplete(DevIntrf_t * const pDev)
{
	pDev->StopTx(pDev);
	pDev->bTxReady = true;
	atomic_flag_clear(&pDev->bBusy);
}
static int SamdDmacOpen(int, SamdDmacHandler_t Handler, void *pCtx, int)
{
	for (int i = 0; i < 2; i++)
	{
		if (!s_Channels[i] && !(i == 1 && s_FailTx))
		{
			s_Channels[i] = true;
			s_DmaHandler[i] = Handler;
			s_DmaCtx[i] = pCtx;
			s_Opens++;
			return i;
		}
	}
	return -1;
}
static void SamdDmacClose(int Chan)
{
	if (Chan >= 0)
	{
		assert(s_Channels[Chan]);
		s_Channels[Chan] = false;
		s_Closes++;
	}
}
static uint32_t SamdDmacStop(int Chan)
{
	assert(Chan >= 0 && s_Channels[Chan]);
	s_Stops++;
	return s_DmaLeft[Chan];
}
static bool SamdDmacBusy(int)
{
	return false;
}
static void SamdDmacStart(int Chan, const volatile void *, bool, volatile void *, bool, uint32_t)
{
	assert(s_Channels[Chan]);
	s_Starts++;
}



static int Event(DevIntrf_t * const pIntrf, DEVINTRF_EVT EventId, uint8_t *, int Len)
{
	if (EventId == DEVINTRF_EVT_STATECHG)
	{
		s_Arms++;
		SPIDev_t *spi = (SPIDev_t *)pIntrf->GetHandle(pIntrf);
		spi->pTxData[0] = s_SlaveTx;
		spi->TxDataLen[0] = sizeof(s_SlaveTx);
		spi->pRxBuff[0] = s_SlaveRx;
		spi->RxBuffLen[0] = sizeof(s_SlaveRx);
	}
	else if (EventId == DEVINTRF_EVT_COMPLETED)
	{
		s_Events++;
		s_EventLen = Len;
	}
	return 0;
}

int main()
{
	SPIDev_t spi = {};
	SPICfg_t cfg = {
		.DevNo = 0, .Phy = SPIPHY_NORMAL, .Mode = SPIMODE_MASTER,
		.pIOPinMap = s_MasterPins, .NbIOPins = 4, .Rate = 1000000,
		.DataSize = 8, .ChipSel = SPICSEL_AUTO, .IntPrio = 3,
		.DummyByte = 0xff, .EvtCB = Event
	};
	uint8_t data[] = {1, 2, 3};
	auto &reg = s_Regs[0].SPIM;

	// Every master mode, including reinitialization from DMA to non-DMA.
	for (int mode : {3, 0, 2, 1})
	{
		cfg.bDmaEn = (mode & 2) != 0;
		cfg.bIntEn = (mode & 1) != 0;
		assert(SPIInit(&spi, &cfg));
		assert(s_Intrf == &spi.DevIntrf);
		assert(spi.DevIntrf.TxData == (cfg.bDmaEn ? SamdSpiTxDataDma : SamdSpiTxData));
		assert(spi.DevIntrf.RxData == (cfg.bDmaEn ? SamdSpiRxDataDma : SamdSpiRxData));
		assert((s_Handler != nullptr) == cfg.bIntEn);
		assert(s_IrqEnabled == cfg.bIntEn);
		assert(s_Channels[0] == cfg.bDmaEn && s_Channels[1] == cfg.bDmaEn);
		assert(spi.DevIntrf.StartTx(&spi.DevIntrf, 0));
		reg.SERCOM_INTFLAG = cfg.bIntEn || cfg.bDmaEn ? 0 :
			SERCOM_SPIM_INTFLAG_DRE_Msk | SERCOM_SPIM_INTFLAG_RXC_Msk;
		unsigned events = s_Events;
		int result = spi.DevIntrf.TxData(&spi.DevIntrf, data, sizeof(data));
		assert(result == (cfg.bIntEn ? -1 : 3));
		if (cfg.bIntEn)
		{
			if (cfg.bDmaEn)
			{
				s_DmaHandler[0](0, DMAC_CHINTFLAG_TCMPL_Msk, s_DmaCtx[0]);
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					reg.SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_RXC_Msk;
					s_Handler(0, &spi.DevIntrf);
				}
			}
			assert(s_Events == events + 1 && s_EventLen == 3);
		}
		else
		{
			assert(s_Events == events);
			spi.DevIntrf.StopTx(&spi.DevIntrf);
		}
		if (cfg.bIntEn && cfg.bDmaEn)
		{
			// DMA must ignore RXC; SERCOM handles only transfer errors.
			reg.SERCOM_INTFLAG = 0;
			assert(spi.DevIntrf.StartTx(&spi.DevIntrf, 0));
			assert(spi.DevIntrf.TxData(&spi.DevIntrf, data, sizeof(data)) == -1);
			events = s_Events;
			reg.SERCOM_DATA = 0xa5;
			reg.SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_RXC_Msk;
			s_Handler(0, &spi.DevIntrf);
			assert(s_Events == events && reg.SERCOM_DATA == 0xa5);
			s_DmaLeft[0] = 1;
			reg.SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_ERROR_Msk;
			s_Handler(0, &spi.DevIntrf);
			assert(s_Events == events + 1 && s_EventLen == 2);
			assert(spi.DevIntrf.bTxReady);
			// A late channel completion cannot report the same transfer twice.
			s_DmaHandler[0](0, DMAC_CHINTFLAG_TCMPL_Msk, s_DmaCtx[0]);
			s_Handler(0, &spi.DevIntrf);
			assert(s_Events == events + 1);
			s_DmaLeft[0] = 0;
		}

		// A read's command phase remains synchronous even with interrupts.
		spi.DevIntrf.bNoStop = true;
		reg.SERCOM_INTFLAG = cfg.bDmaEn ? 0 :
			SERCOM_SPIM_INTFLAG_DRE_Msk | SERCOM_SPIM_INTFLAG_RXC_Msk;
		events = s_Events;
		assert(spi.DevIntrf.TxData(&spi.DevIntrf, data, sizeof(data)) == 3);
		assert(s_Events == events);
		spi.DevIntrf.bNoStop = false;
		uint8_t received[3] = {};
		reg.SERCOM_INTFLAG = cfg.bIntEn || cfg.bDmaEn ? 0 :
			SERCOM_SPIM_INTFLAG_DRE_Msk | SERCOM_SPIM_INTFLAG_RXC_Msk;
		events = s_Events;
		assert(spi.DevIntrf.RxData(&spi.DevIntrf, received, sizeof(received)) ==
			(cfg.bIntEn ? -1 : 3));
		if (cfg.bIntEn)
		{
			if (cfg.bDmaEn)
			{
				s_DmaHandler[0](0, DMAC_CHINTFLAG_TCMPL_Msk, s_DmaCtx[0]);
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					reg.SERCOM_DATA = data[i];
					reg.SERCOM_INTFLAG = SERCOM_SPIM_INTFLAG_RXC_Msk;
					s_Handler(0, &spi.DevIntrf);
				}
				assert(memcmp(received, data, sizeof(data)) == 0);
			}
			assert(s_Events == events + 1 && s_EventLen == 3);
		}
		else
		{
			assert(s_Events == events);
		}

		unsigned stops = s_Stops;
		spi.DevIntrf.Disable(&spi.DevIntrf);
		assert(!s_Clock);
		spi.DevIntrf.Enable(&spi.DevIntrf);
		assert(s_Clock);
		spi.DevIntrf.Reset(&spi.DevIntrf);
		assert(s_Stops == stops + (cfg.bDmaEn ? 4 : 0));
	}

	// PowerOff releases both channels and rejects transfers until Init.
	cfg.bDmaEn = true;
	assert(SPIInit(&spi, &cfg));
	spi.DevIntrf.PowerOff(&spi.DevIntrf);
	assert(!s_Clock && !s_Channels[0] && !s_Channels[1]);
	assert(spi.DevIntrf.TxData(&spi.DevIntrf, data, sizeof(data)) == 0);
	assert(spi.DevIntrf.RxData(&spi.DevIntrf, data, sizeof(data)) == 0);
	assert(SPIInit(&spi, &cfg));
	spi.DevIntrf.PowerOff(&spi.DevIntrf);

	// Hardware reset timeout also releases channels after acquisition.
	reg.SERCOM_SYNCBUSY = SERCOM_SPIM_SYNCBUSY_SWRST_Msk;
	assert(!SPIInit(&spi, &cfg));
	assert(!s_Channels[0] && !s_Channels[1] && !s_Clock);
	reg.SERCOM_SYNCBUSY = 0;

	// Failed acquisition returns the first channel; polling can follow.
	s_FailTx = true;
	assert(!SPIInit(&spi, &cfg));
	assert(!s_Channels[0] && !s_Channels[1] && s_Intrf == nullptr);
	s_FailTx = false;
	cfg.bDmaEn = false;
	cfg.bIntEn = false;
	assert(SPIInit(&spi, &cfg));

	// Slave init, reset and enable each arm the next frame; master I/O fails.
	cfg.Mode = SPIMODE_SLAVE;
	cfg.pIOPinMap = s_SlavePins;
	assert(SPIInit(&spi, &cfg));
	assert(s_Handler != nullptr && s_IrqEnabled && s_Arms == 1);
	assert(reg.SERCOM_DATA == s_SlaveTx[0]);
	assert(spi.DevIntrf.TxData(&spi.DevIntrf, data, sizeof(data)) == 0);
	spi.DevIntrf.Reset(&spi.DevIntrf);
	spi.DevIntrf.Disable(&spi.DevIntrf);
	spi.DevIntrf.Enable(&spi.DevIntrf);
	assert(s_Arms == 3);
	spi.DevIntrf.PowerOff(&spi.DevIntrf);
	assert(s_Opens == s_Closes);
	assert(s_Starts == 14);
	puts("SAMD SPI master modes, completion, lifecycle and slave arming: PASS");
}
