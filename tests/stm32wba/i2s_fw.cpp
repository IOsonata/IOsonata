// Production I2S transport with modeled DMA transfers and SAI registers.
#include <stdint.h>
#include <stddef.h>
#include "stm32wbaxx.h"
#include "coredev/i2s.h"
#include "gpdma_stm32wba.h"
#include "sai_stm32wba.h"

#define TEST_FRAMES 32

typedef struct __Test_Dma
{
	bool Used;
	bool Irq;
	bool Running;
	bool ToPeriph;
	unsigned Width;
	uint32_t Flags;
	uint8_t *pMem;
	uint32_t Count;
	WbaGpdmaHandler_t Handler;
	void *pCtx;
} TestDma_t;

static TestDma_t s_Dma[2];
static int s_AllocationLimit = 2;
static unsigned s_Checks;
static bool s_RequireBothRunning;
static unsigned s_Events[DEVINTRF_EVT_COMPLETED + 1];
static volatile uint32_t *const s_Io = (volatile uint32_t *)0x500FF000;
alignas(4) static uint8_t s_RxMem[CFIFO_TOTAL_MEMSIZE(64, 6)];
alignas(4) static uint8_t s_TxMem[CFIFO_TOTAL_MEMSIZE(64, 6)];
static const IOPinCfg_t s_Pins[] = {
	{ IOPORTA, 7, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ IOPORTC, 4, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ IOPORTC, 5, IOPINOP_FUNC13, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ IOPORTD, 5, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ IOPORTE, 2, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
};

extern "C" uint8_t *__real_CFifoPeek(hCFifo_t Fifo);
extern "C" uint8_t *__real_CFifoResv(hCFifo_t Fifo);
extern "C" uint8_t *__real_CFifoPeekMultiple(hCFifo_t Fifo, int *pCount);
extern "C" uint8_t *__real_CFifoResvMultiple(hCFifo_t Fifo, int *pCount);

static void Check(bool Ok)
{
	s_Io[1] = ++s_Checks | (Ok ? 0 : 0x80000000U);
}

static void CheckDmaRunning()
{
	if (s_RequireBothRunning)
	{
		Check(s_Dma[0].Running && s_Dma[1].Running);
	}
}

extern "C" uint8_t *__wrap_CFifoPeek(hCFifo_t Fifo)
{
	CheckDmaRunning();
	return __real_CFifoPeek(Fifo);
}

extern "C" uint8_t *__wrap_CFifoResv(hCFifo_t Fifo)
{
	CheckDmaRunning();
	return __real_CFifoResv(Fifo);
}

extern "C" uint8_t *__wrap_CFifoPeekMultiple(hCFifo_t Fifo, int *pCount)
{
	CheckDmaRunning();
	return __real_CFifoPeekMultiple(Fifo, pCount);
}

extern "C" uint8_t *__wrap_CFifoResvMultiple(hCFifo_t Fifo, int *pCount)
{
	CheckDmaRunning();
	return __real_CFifoResvMultiple(Fifo, pCount);
}

extern "C" void *memset(void *pDest, int Value, size_t Count)
{
	volatile uint8_t *dest = (volatile uint8_t *)pDest;

	while (Count--)
	{
		*dest++ = Value;
	}
	return pDest;
}

extern "C" void *memcpy(void *pDest, const void *pSource, size_t Count)
{
	volatile uint8_t *dest = (volatile uint8_t *)pDest;
	const volatile uint8_t *source = (const volatile uint8_t *)pSource;

	while (Count--)
	{
		*dest++ = *source++;
	}
	return pDest;
}

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

extern "C" int WbaGpdmaAlloc(WbaGpdmaHandler_t Handler, void *pCtx, int Priority, bool Irq)
{
	Check(Priority == 6);
	for (int ch = 0; ch < s_AllocationLimit; ch++)
	{
		if (!s_Dma[ch].Used)
		{
			s_Dma[ch].Used = true;
			s_Dma[ch].Handler = Handler;
			s_Dma[ch].pCtx = pCtx;
			s_Dma[ch].Irq = Irq;
			return ch;
		}
	}
	return -1;
}

extern "C" void WbaGpdmaStop(int Ch)
{
	Check(Ch >= 0 && Ch < 2 && s_Dma[Ch].Used);
	s_Dma[Ch].Running = false;
	s_Dma[Ch].Flags = 0;
}

extern "C" void WbaGpdmaFree(int Ch)
{
	if (Ch < 0)
	{
		return;
	}
	WbaGpdmaStop(Ch);
	s_Dma[Ch].Used = false;
}

extern "C" void WbaGpdmaSetup(int Ch, uint32_t Request, bool ToPeriph, volatile void *pPeriph, int Width, bool Irq)
{
	Check(Ch >= 0 && Ch < 2 && s_Dma[Ch].Used && s_Dma[Ch].Irq == Irq);
	Check(ToPeriph ? Request == 17 && pPeriph == &SAI1_Block_A->DR : Request == 18 && pPeriph == &SAI1_Block_B->DR);
	s_Dma[Ch].ToPeriph = ToPeriph;
	s_Dma[Ch].Width = 1U << Width;
}

extern "C" void WbaGpdmaStart(int Ch, const void *pMem, uint32_t Count)
{
	Check(Ch >= 0 && Ch < 2 && s_Dma[Ch].Used && !(uintptr_t(pMem) & 3U));
	Check(Count > 0 && Count <= 256 && Count % s_Dma[Ch].Width == 0);
	s_Dma[Ch].pMem = (uint8_t *)pMem;
	s_Dma[Ch].Count = Count;
	s_Dma[Ch].Flags = 0;
	s_Dma[Ch].Running = true;
}

extern "C" uint32_t WbaGpdmaFlags(int Ch)
{
	Check(Ch >= 0 && Ch < 2);
	return s_Dma[Ch].Flags;
}

extern "C" void WbaGpdmaClear(int Ch)
{
	Check(Ch >= 0 && Ch < 2);
	s_Dma[Ch].Flags = 0;
}

static int Event(DevIntrf_t *, DEVINTRF_EVT Evt, uint8_t *, int)
{
	s_Events[Evt]++;
	return 0;
}

static void Complete(int Ch, bool Error = false)
{
	Check(s_Dma[Ch].Running);
	s_Dma[Ch].Flags = Error ? WBA_GPDMA_FLAG_ERR : WBA_GPDMA_FLAG_TC;
	s_Dma[Ch].Running = false;
	if (s_Dma[Ch].Irq)
	{
		s_Dma[Ch].Handler(Ch, s_Dma[Ch].pCtx);
	}
}

static void Capture()
{
	Check(s_Dma[0].Running && !s_Dma[0].ToPeriph);
	for (unsigned sample = 0; sample < s_Dma[0].Count / s_Dma[0].Width; sample++)
	{
		uint32_t value = (sample & 1) ? 0x0080FFFE : 0x007F0001;

		memcpy(s_Dma[0].pMem + sample * s_Dma[0].Width, &value, s_Dma[0].Width);
	}
	Complete(0);
}

static void DummyHandler()
{
}

extern "C" void SAI1_IRQHandler();

static void I2sChecks()
{
	I2SDev_t dev = {};
	I2SDev_t other = {};
	I2SCfg_t cfg = {
		.DevNo = 0,
		.pIOPinMap = s_Pins,
		.NbIOPins = 5,
		.Mode = I2SMODE_MASTER,
		.Format = I2SFMT_NORMAL,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Chan = I2SCHAN_STEREO,
		.Freq = 16000,
		.MaxRetry = 0,
		.bDmaEn = true,
		.bIntEn = true,
		.IntPrio = 6,
		.EvtCB = Event,
		.pRxFifoMem = s_RxMem,
		.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(64, 4),
		.pTxFifoMem = s_TxMem,
		.TxFifoMemSize = CFIFO_TOTAL_MEMSIZE(64, 4)
	};
	uint8_t data[256];

	IOPinCfg_t pins[5];

	memcpy(pins, s_Pins, sizeof(pins));
	cfg.pIOPinMap = pins;
	pins[0].PinNo = 0;
	Check(!I2SInit(&dev, &cfg));
	pins[0] = s_Pins[0];
	cfg.pRxFifoMem = s_TxMem;
	Check(!I2SInit(&dev, &cfg));
	cfg.pRxFifoMem = s_RxMem;
	Check(!I2SInit(nullptr, &cfg));
	Check(!I2SInit(&dev, nullptr));
	cfg.DevNo = 1;
	Check(!I2SInit(&dev, &cfg));
	cfg.DevNo = 0;
	cfg.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(1, 4);
	Check(!I2SInit(&dev, &cfg));
	cfg.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(64, 4);
	s_AllocationLimit = 1;
	Check(!I2SInit(&dev, &cfg));
	Check(!s_Dma[0].Used && !s_Dma[1].Used);
	s_AllocationLimit = 2;
	s_Io[0] = 1;
	Check(!I2SInit(&dev, &cfg));
	Check(!s_Dma[0].Used && !s_Dma[1].Used);
	s_Io[0] = 0;
	Check(I2SInit(&dev, &cfg));
	Check(dev.Freq == 15625 && dev.MClkFreq == 4000000);
	Check(!I2SInit(&other, &cfg));
	Check(!WbaSaiClaim(&other, DummyHandler));
	Check(SAI1_Block_A->FRCR == (31U | (15U << 8) | SAI_xFRCR_FSDEF | SAI_xFRCR_FSOFF));
	Check((SAI1_Block_A->CR1 & (SAI_xCR1_MCKEN | SAI_xCR1_SAIEN)) == (SAI_xCR1_MCKEN | SAI_xCR1_SAIEN));
	Check((SAI1_Block_B->CR1 & (SAI_xCR1_MODE | SAI_xCR1_SYNCEN)) == (SAI_xCR1_MODE | SAI_xCR1_SYNCEN_0));
	// Both DMA completions must be rearmed before any FIFO processing.
	s_Dma[0].Flags = WBA_GPDMA_FLAG_TC;
	s_Dma[1].Flags = WBA_GPDMA_FLAG_TC;
	s_Dma[0].Running = false;
	s_Dma[1].Running = false;
	s_RequireBothRunning = true;
	s_Dma[0].Handler(0, s_Dma[0].pCtx);
	s_RequireBothRunning = false;
	DeviceIntrfReset(&dev.DevIntrf);
	memset(s_Events, 0, sizeof(s_Events));
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 4) == 0);
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 3) == 0);
	Check(CFifoUsed(dev.hRxFifo) == 0);
	Capture();
	Check(CFifoUsed(dev.hRxFifo) == 32 && s_Events[DEVINTRF_EVT_RX_DATA] == 1);
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 128) == 128);
	for (unsigned frame = 0; frame < 32; frame++)
	{
		Check(data[frame * 4] == 1 && data[frame * 4 + 1] == 0 && data[frame * 4 + 2] == 0xFE && data[frame * 4 + 3] == 0xFF);
	}
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 128) == 128);
	Check(CFifoUsed(dev.hTxFifo) == 32);
	Complete(1);
	Check(CFifoUsed(dev.hTxFifo) == 0 && s_Dma[1].Count == 128);
	// The already prepared silent block runs before the new data block.
	for (unsigned i = 0; i < 128; i++)
	{
		Check(s_Dma[1].pMem[i] == 0);
	}
	Complete(1);
	for (unsigned i = 0; i < 128; i++)
	{
		Check(s_Dma[1].pMem[i] == data[i]);
	}
	Complete(1);
	Check(s_Events[DEVINTRF_EVT_TX_READY] == 1 && s_Events[DEVINTRF_EVT_TX_FIFO_EMPTY] == 1);
	Capture();
	Capture();
	Capture();
	Check(s_Events[DEVINTRF_EVT_RX_FIFO_FULL] == 1);
	Check(!s_Dma[0].Running && !s_Dma[1].Running);
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 4) == 0);
	DeviceIntrfReset(&dev.DevIntrf);
	Check(CFifoUsed(dev.hRxFifo) == 0 && s_Dma[0].Running);
	Complete(0, true);
	Check(s_Events[DEVINTRF_EVT_STATECHG] == 1 && !s_Dma[1].Running);
	DeviceIntrfReset(&dev.DevIntrf);
	s_Io[0] = 2;
	SAI1_IRQHandler();
	Check(s_Events[DEVINTRF_EVT_STATECHG] == 2);
	s_Io[0] = 0;
	DeviceIntrfReset(&dev.DevIntrf);
	Check(DeviceIntrfSetRate(&dev.DevIntrf, UINT32_MAX) == 62500);
	Check(DeviceIntrfSetRate(&dev.DevIntrf, 1) == 992);
	Check(DeviceIntrfSetRate(&dev.DevIntrf, 0) == 15625);
	DeviceIntrfEnable(&dev.DevIntrf);
	DeviceIntrfDisable(&dev.DevIntrf);
	Check(s_Dma[0].Running);
	DeviceIntrfDisable(&dev.DevIntrf);
	Check(!s_Dma[0].Running);
	DeviceIntrfEnable(&dev.DevIntrf);
	Check(s_Dma[0].Running);
	s_Io[0] = 3;
	DeviceIntrfDisable(&dev.DevIntrf);
	s_Io[0] = 0;
	DeviceIntrfEnable(&dev.DevIntrf);
	Check(s_Dma[0].Running);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	Check(!s_Dma[0].Used && !s_Dma[1].Used && !(RCC->APB2ENR & RCC_APB2ENR_SAI1EN));

	// Packed widths and channel slot selection, including 24-bit sign bits.
	for (unsigned width = 1; width <= 3; width++)
	{
		for (unsigned channel = 0; channel < 3; channel++)
		{
			cfg.SampleWidth = I2SSWIDTH(width * 8);
			cfg.Chan = I2SCHAN(channel);
			cfg.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(64, width * (channel ? 1 : 2));
			cfg.TxFifoMemSize = cfg.RxFifoMemSize;
			cfg.Format = I2SFMT(channel);
			cfg.bIntEn = channel != 1;
			Check(I2SInit(&dev, &cfg));
			unsigned expectedSlots = channel == 0 ? 3 : channel == 1 ? 1
																	 : 2;

			Check((SAI1_Block_B->SLOTR >> SAI_xSLOTR_SLOTEN_Pos) == expectedSlots);
			Capture();
			unsigned count = 32 * width * (channel ? 1 : 2);

			Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, count) == int(count));
			for (unsigned sample = 0; sample < count / width; sample++)
			{
				uint32_t value = (sample & 1) ? 0x0080FFFE : 0x007F0001;

				for (unsigned byte = 0; byte < width; byte++)
				{
					Check(data[sample * width + byte] == uint8_t(value >> (8 * byte)));
				}
			}
			Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, count) == int(count));
			Complete(1);
			if (!cfg.bIntEn)
			{
				DeviceIntrfRx(&dev.DevIntrf, 0, data, width * (channel ? 1 : 2));
			}
			Check(CFifoUsed(dev.hTxFifo) == 0);
			Complete(1);
			if (!cfg.bIntEn)
			{
				DeviceIntrfRx(&dev.DevIntrf, 0, data, width * (channel ? 1 : 2));
			}
			for (unsigned sample = 0; sample < count / width; sample++)
			{
				for (unsigned byte = 0; byte < s_Dma[1].Width; byte++)
				{
					Check(s_Dma[1].pMem[sample * s_Dma[1].Width + byte] == (byte < width ? data[sample * width + byte] : 0));
				}
			}
			DeviceIntrfPowerOff(&dev.DevIntrf);
		}
	}
	// Non-power-of-two FIFOs split a DMA block at the ring boundary.
	cfg.SampleWidth = I2SSWIDTH_16BITS;
	cfg.Chan = I2SCHAN_STEREO;
	cfg.Format = I2SFMT_NORMAL;
	cfg.bIntEn = true;
	cfg.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(40, 4);
	cfg.TxFifoMemSize = cfg.RxFifoMemSize;
	Check(I2SInit(&dev, &cfg));
	Capture();
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 96) == 96);
	Capture();
	Check(CFifoUsed(dev.hRxFifo) == 40);
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 160) == 160);
	for (unsigned frame = 0; frame < 40; frame++)
	{
		Check(data[frame * 4] == 1 && data[frame * 4 + 1] == 0 && data[frame * 4 + 2] == 0xFE && data[frame * 4 + 3] == 0xFF);
	}
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 128) == 128);
	Complete(1);
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 128) == 128);
	Complete(1);
	for (unsigned block = 0; block < 2; block++)
	{
		for (unsigned i = 0; i < 128; i++)
		{
			Check(s_Dma[1].pMem[i] == data[i]);
		}
		Complete(1);
	}
	Check(CFifoUsed(dev.hTxFifo) == 0);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	for (unsigned sample = 0; sample < 32; sample++)
	{
		uint32_t value = (sample & 1) ? 0x0080FFFE : 0x007F0001;

		for (unsigned byte = 0; byte < 3; byte++)
		{
			data[sample * 3 + byte] = value >> (8 * byte);
		}
	}
	// Restore the packed 24-bit playback case used below.
	cfg.SampleWidth = I2SSWIDTH_24BITS;
	cfg.Chan = I2SCHAN_RIGHT;
	cfg.Format = I2SFMT_ALIGN_RIGHT;
	cfg.RxFifoMemSize = CFIFO_TOTAL_MEMSIZE(64, 3);
	cfg.TxFifoMemSize = cfg.RxFifoMemSize;
	// Playback uses transmit only and allocates its TX channel independently.
	cfg.pIOPinMap = pins;
	pins[2].PortNo = -1;
	cfg.pRxFifoMem = nullptr;
	cfg.bIntEn = true;
	Check(I2SInit(&dev, &cfg));
	Check(s_Dma[0].Used && s_Dma[0].ToPeriph && !s_Dma[1].Used);
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 96) == 96);
	Complete(0);
	Check(CFifoUsed(dev.hTxFifo) == 0);
	Complete(0);
	Check(s_Dma[0].pMem[0] == 1 && s_Dma[0].pMem[2] == 0x7F && s_Dma[0].pMem[3] == 0);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	pins[2] = s_Pins[2];
	cfg.pRxFifoMem = s_RxMem;
	// The board microphone uses receive only, with no SD_A output pin.
	cfg.pIOPinMap = pins;
	pins[3].PortNo = -1;
	cfg.pTxFifoMem = nullptr;
	cfg.bIntEn = true;
	Check(I2SInit(&dev, &cfg));
	Check(s_Dma[0].Used && !s_Dma[1].Used);
	Capture();
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 32 * 3) == 32 * 3);
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 3) == 0);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	pins[3] = s_Pins[3];
	cfg.pIOPinMap = s_Pins;
	// No FIFO: bounded synchronous polling, real DeviceIntrf start/stop.
	cfg.bDmaEn = false;
	cfg.bIntEn = false;
	cfg.pRxFifoMem = nullptr;
	cfg.pTxFifoMem = nullptr;
	cfg.SampleWidth = I2SSWIDTH_16BITS;
	cfg.Chan = I2SCHAN_LEFT;
	cfg.Mode = I2SMODE_SLAVE;
	cfg.Freq = 16000;
	Check(I2SInit(&dev, &cfg));
	Check(dev.Freq == 16000 && dev.MClkFreq == 0);
	s_Io[5] = 0x8001;
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 2) == 2 && data[0] == 1 && data[1] == 0x80);
	Check(DeviceIntrfTx(&dev.DevIntrf, 0, data, 2) == 2 && s_Io[7] == 0x8001);
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 2) == 0);
	Check(s_Events[DEVINTRF_EVT_RX_TIMEOUT] == 1);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	// Interrupt FIFO receive and safe publication of complete frames.
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = sizeof(s_RxMem);
	cfg.bIntEn = true;
	Check(I2SInit(&dev, &cfg));
	s_Io[5] = 0x7FFF;
	SAI1_IRQHandler();
	Check(DeviceIntrfRx(&dev.DevIntrf, 0, data, 2) == 2 && data[0] == 0xFF && data[1] == 0x7F);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	Check(WbaSaiClaim(&other, DummyHandler));
	WbaSaiRelease(&other);
	// Reuse a running HSE-derived SYSCLK without changing the PLL or CPU.
	cfg.Mode = I2SMODE_MASTER;
	cfg.Freq = 15625;
	RCC->CR |= RCC_CR_HSERDY | RCC_CR_PLL1RDY;
	RCC->CFGR1 = RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0;
	RCC->CFGR2 = RCC_CFGR2_HPRE;
	RCC->PLL1CFGR = RCC_PLL1CFGR_PLL1SRC_1 | RCC_PLL1CFGR_PLL1SRC_0 |
		(7U << RCC_PLL1CFGR_PLL1M_Pos) | RCC_PLL1CFGR_PLL1REN;
	RCC->PLL1DIVR = (99U << RCC_PLL1DIVR_PLL1N_Pos) | (3U << RCC_PLL1DIVR_PLL1R_Pos);
	RCC->CCIPR2 = RCC_CCIPR2_SAI1SEL_0;
	uint32_t pll = RCC->PLL1CFGR;
	uint32_t divr = RCC->PLL1DIVR;

	Check(I2SInit(&dev, &cfg));
	Check(dev.Freq == 15625 && dev.MClkFreq == 4000000);
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_1);
	Check((SAI1_Block_A->CR1 & SAI_xCR1_MCKDIV) == (25U << SAI_xCR1_MCKDIV_Pos));
	Check(RCC->PLL1CFGR == pll && RCC->PLL1DIVR == divr);
	Check(RCC->CFGR1 == (RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0));
	DeviceIntrfPowerOff(&dev.DevIntrf);
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_0);
	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1FRACEN;
	RCC->PLL1FRACR = 4096U << RCC_PLL1FRACR_PLL1FRACN_Pos;
	Check(I2SInit(&dev, &cfg));
	Check(dev.Freq == 15703 && dev.MClkFreq == 4020000);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	// Direct HSE with HSEPRE also uses SYSCLK before the AHB prescaler.
	RCC->CFGR1 = RCC_CFGR1_SWS_1;
	RCC->CR |= RCC_CR_HSEPRE;
	Check(I2SInit(&dev, &cfg));
	Check(dev.Freq == 15625 && dev.MClkFreq == 4000000);
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_1);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	// An HSI-derived PLL keeps the original independent HSI16 source.
	RCC->CFGR1 = RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0;
	RCC->PLL1CFGR = RCC_PLL1CFGR_PLL1SRC_1;
	Check(I2SInit(&dev, &cfg));
	Check((RCC->CCIPR2 & RCC_CCIPR2_SAI1SEL) == RCC_CCIPR2_SAI1SEL_2);
	DeviceIntrfPowerOff(&dev.DevIntrf);
	s_Io[2] = 1;
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *src = &_sidata;

	for (uint32_t *dest = &_sdata; dest < &_edata;)
	{
		*dest++ = *src++;
	}
	for (uint32_t *dest = &_sbss; dest < &_ebss;)
	{
		*dest++ = 0;
	}
	I2sChecks();
	for (;;)
	{
	}
}
