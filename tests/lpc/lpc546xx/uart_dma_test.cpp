// Host regression of the LPC546xx DMA channel driver and the Flexcomm USART
// TX DMA path, with register models.
//
// The DMA model reads the channel descriptor from the table at SRAMBASE and
// moves one value per step while the channel is enabled, triggered and its
// peripheral request is on. The request of a USART TX channel is on while
// the TX FIFO is not full and FIFOCFG DMATX is set. The end of a descriptor
// sets INTA when SETINTA is set, clears the trigger with CLRTRIG and calls
// the DMA0 vector when its interrupt is enabled and not masked.
//
// The USART model has the 16 entry TX FIFO, drained to a capture buffer one
// character per line step.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "LPC546xx.h"

// Flexcomm PSELID, the present bits are read only
struct Psel {
	uint32_t value = 0x00000070U;
	operator uint32_t() const { return value; }
	Psel &operator=(uint32_t v) { value = (value & ~0xFU) | (v & 0xFU); return *this; }
};

struct FcRegs {
	uint8_t Reserved[0xFF8];
	Psel PSELID;
	uint32_t PID;
};

// USART model
struct UsartModel;
struct UFifoWr {
	UsartModel *pM = nullptr;
	UFifoWr &operator=(uint32_t v);
};
struct UFifoStat {
	UsartModel *pM = nullptr;
	operator uint32_t() const;
	UFifoStat &operator=(uint32_t v);
};
struct UFifoRd {
	operator uint32_t() const { assert(false); return 0; }
};
struct UsartRegs {
	uint32_t CFG = 0, CTL = 0, STAT = 0, INTENSET = 0, INTENCLR = 0, BRG = 0, INTSTAT = 0, OSR = 0, ADDR = 0;
	uint32_t FIFOCFG = 0;
	UFifoStat FIFOSTAT;
	uint32_t FIFOTRIG = 0;
	struct Set {
		uint32_t value = 0;
		operator uint32_t() const { return value; }
		Set &operator=(uint32_t v) { value |= v; return *this; }
	} FIFOINTENSET;
	struct Clr {
		uint32_t *p = nullptr;
		Clr &operator=(uint32_t v) { *p &= ~v; return *this; }
	} FIFOINTENCLR;
	UFifoWr FIFOWR;
	UFifoRd FIFORD;
};
struct UsartModel {
	UsartRegs *pRegs = nullptr;
	uint8_t Fifo[16] = {};
	int Cnt = 0;
	bool bTxErr = false;
	std::vector<uint8_t> Line;
};

static UsartModel s_UsartM[10];
static UsartRegs s_Usart[10];

UFifoWr &UFifoWr::operator=(uint32_t v)
{
	if (pM->Cnt >= 16)
	{
		pM->bTxErr = true;
	}
	else
	{
		pM->Fifo[pM->Cnt++] = (uint8_t)v;
	}
	return *this;
}

UFifoStat::operator uint32_t() const
{
	return (pM->Cnt < 16 ? USART_FIFOSTAT_TXNOTFULL_MASK : 0U) | (pM->Cnt == 0 ? USART_FIFOSTAT_TXEMPTY_MASK : 0U) |
		   (pM->bTxErr ? USART_FIFOSTAT_TXERR_MASK : 0U);
}

UFifoStat &UFifoStat::operator=(uint32_t v)
{
	if (v & USART_FIFOSTAT_TXERR_MASK)
	{
		pM->bTxErr = false;
	}
	return *this;
}

// One character out of the TX FIFO onto the line
static bool LineStep(int No)
{
	UsartModel &m = s_UsartM[No];

	if (m.Cnt == 0 || (m.pRegs->CFG & USART_CFG_ENABLE_MASK) == 0)
	{
		return false;
	}
	m.Line.push_back(m.Fifo[0]);
	memmove(m.Fifo, &m.Fifo[1], --m.Cnt);

	return true;
}

#include "dma_model.h"
#include "dma_lpc546xx.h"

alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
static FcRegs s_Fc[10];
uint32_t g_IrqMasked = 0;

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { s_NvicPrio[n] = p; }
void NVIC_EnableIRQ(IRQn_Type n) { s_NvicEn |= 1UL << (n & 31); }
void NVIC_DisableIRQ(IRQn_Type n) { s_NvicEn &= ~(1UL << (n & 31)); }
void __DMB(void) {}
void __DSB(void) {}

#undef SYSCON
#undef FLEXCOMM0
#undef FLEXCOMM1
#undef FLEXCOMM2
#undef FLEXCOMM3
#undef FLEXCOMM4
#undef FLEXCOMM5
#undef FLEXCOMM6
#undef FLEXCOMM7
#undef FLEXCOMM8
#undef FLEXCOMM9
#undef USART0
#undef USART1
#undef USART2
#undef USART3
#undef USART4
#undef USART5
#undef USART6
#undef USART7
#undef USART8
#undef USART9
#undef DMA0
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define FLEXCOMM_Type FcRegs
#define FLEXCOMM0 (&s_Fc[0])
#define FLEXCOMM1 (&s_Fc[1])
#define FLEXCOMM2 (&s_Fc[2])
#define FLEXCOMM3 (&s_Fc[3])
#define FLEXCOMM4 (&s_Fc[4])
#define FLEXCOMM5 (&s_Fc[5])
#define FLEXCOMM6 (&s_Fc[6])
#define FLEXCOMM7 (&s_Fc[7])
#define FLEXCOMM8 (&s_Fc[8])
#define FLEXCOMM9 (&s_Fc[9])
#define USART_Type UsartRegs
#define USART0 (&s_Usart[0])
#define USART1 (&s_Usart[1])
#define USART2 (&s_Usart[2])
#define USART3 (&s_Usart[3])
#define USART4 (&s_Usart[4])
#define USART5 (&s_Usart[5])
#define USART6 (&s_Usart[6])
#define USART7 (&s_Usart[7])
#define USART8 (&s_Usart[8])
#define USART9 (&s_Usart[9])
#define DMA_Type DmaRegs
#define DMA0 (&s_DmaRegs)

// Request of a USART TX channel: TX FIFO not full with DMATX
static bool DmaReqOn(int Ch)
{
	for (int i = 0; i < 10; i++)
	{
		if (LPC546XX_DMA_CH_FC_TX(i) == Ch)
		{
			return (s_Usart[i].FIFOCFG & USART_FIFOCFG_DMATX_MASK) && s_UsartM[i].Cnt < 16;
		}
	}
	return false;
}

static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal)
{
	(void)Addr; (void)pVal;
	return false;
}

// Memory to the USART FIFOWR
static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val)
{
	for (int i = 0; i < 10; i++)
	{
		if (Addr == (uint32_t)(uintptr_t)&s_Usart[i].FIFOWR)
		{
			s_Usart[i].FIFOWR = Val;
			return true;
		}
	}
	return false;
}

#include "coredev/iopincfg.h"
#include "coredev/uart.h"

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo; (void)PinNo; (void)PinOp; (void)Dir; (void)Resistor; (void)Type;
}
void IOPinDisable(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }

#include "../../../src/cfifo.c"
#include "../../../src/coredev/shared_intrf.cpp"
#include "../../../ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/uart_lpc546xx.cpp"
#include "../../../src/device_intrf.cpp"

static void UsartModelInit(void)
{
	for (int i = 0; i < 10; i++)
	{
		s_Usart[i] = UsartRegs();
		s_UsartM[i] = UsartModel();
		s_UsartM[i].pRegs = &s_Usart[i];
		s_Usart[i].FIFOSTAT.pM = &s_UsartM[i];
		s_Usart[i].FIFOWR.pM = &s_UsartM[i];
		s_Usart[i].FIFOINTENCLR.p = &s_Usart[i].FIFOINTENSET.value;
	}
}

// All models at reset, with no DMA channel taken
static void ModelInit(void)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	DmaModelInit();
	UsartModelInit();
	s_NvicEn = 0;
	g_IrqMasked = 0;
}

// DMA and line until both are idle
static void Run(int No)
{
	for (int i = 0; i < 100000; i++)
	{
		bool d = DmaStep() > 0;
		bool l = LineStep(No);
		if (!d && !l)
		{
			return;
		}
	}
	assert(false);
}

static const IOPinCfg_t s_UartPins[] = {
	{0, 29, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{0, 30, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static int s_TxReadyCnt = 0;
static int s_TxReadyLen = -1;

static int UartEvt(UARTDev_t * const pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pDev; (void)pBuffer;
	if (EvtId == UART_EVT_TXREADY)
	{
		s_TxReadyCnt++;
		s_TxReadyLen = BufferLen;
	}
	return 0;
}

static uint8_t s_Pattern[3000];

static UARTCfg_t UartCfg(bool bInt, bool bDma, bool bBlocking, uint8_t *pTxMem, int TxMemSize)
{
	UARTCfg_t cfg = {};

	cfg.DevNo = 0;
	cfg.pIOPinMap = s_UartPins;
	cfg.NbIOPins = 2;
	cfg.Rate = 115200;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.bIntMode = bInt;
	cfg.IntPrio = 3;
	cfg.EvtCallback = UartEvt;
	cfg.bFifoBlocking = bBlocking;
	cfg.TxMemSize = TxMemSize;
	cfg.pTxMem = pTxMem;
	cfg.bDMAMode = bDma;
	cfg.Mode = UART_MODE_UART;
	cfg.Duplex = UART_DUPLEX_FULL;

	return cfg;
}

// Send Len bytes with UARTTx, running the model between calls the way time
// passes between them
static void Send(UARTDev_t *pDev, const uint8_t *p, int Len)
{
	int done = 0;

	for (int i = 0; i < 100000 && done < Len; i++)
	{
		int l = UARTTx(pDev, (uint8_t *)&p[done], Len - done);
		assert(l >= 0);
		done += l;
		DmaStep();
		LineStep(0);
	}
	assert(done == Len);
}

// Send Len bytes, each call no more than the CFIFO can take. A non-blocking
// CFIFO drops its oldest bytes when a put does not fit.
static void SendWait(UARTDev_t *pDev, const uint8_t *p, int Len)
{
	int done = 0;

	for (int i = 0; i < 100000 && done < Len; i++)
	{
		int l = CFifoAvail(pDev->hTxFifo);
		if (l > Len - done)
		{
			l = Len - done;
		}
		if (l > 0)
		{
			assert(UARTTx(pDev, (uint8_t *)&p[done], l) == l);
			done += l;
		}
		DmaStep();
		LineStep(0);
	}
	assert(done == Len);
}

static int s_HandlerCh[8];
static int s_HandlerCnt = 0;
static uint32_t s_HandlerFlags[8];

static void ChHandler(int Ch, void *pCtx)
{
	assert(pCtx == (void *)&s_HandlerCnt);
	s_HandlerFlags[s_HandlerCnt] = Lpc546xxDmaChFlags(Ch);
	s_HandlerCh[s_HandlerCnt++] = Ch;
}

static int s_Handler2Cnt = 0;

static void ChHandler2(int Ch, void *pCtx)
{
	(void)Ch; (void)pCtx;
	s_Handler2Cnt++;
}

static void DmaChannelTest(void)
{
	ModelInit();

	// Channel numbers of the Flexcomm requests
	assert(LPC546XX_DMA_CH_FC_RX(0) == 0 && LPC546XX_DMA_CH_FC_TX(0) == 1);
	assert(LPC546XX_DMA_CH_FC_RX(7) == 14 && LPC546XX_DMA_CH_FC_TX(7) == 15);
	assert(LPC546XX_DMA_CH_FC_RX(8) == 20 && LPC546XX_DMA_CH_FC_TX(8) == 21);
	assert(LPC546XX_DMA_CH_FC_RX(9) == 22 && LPC546XX_DMA_CH_FC_TX(9) == 23);
	assert(LPC546XX_DMA_NBCH == 30);

	assert(!Lpc546xxDmaChClaim(-1, ChHandler, &s_HandlerCnt, 2, true));
	assert(!Lpc546xxDmaChClaim(30, ChHandler, &s_HandlerCnt, 2, true));

	// First channel turns the controller on with the table at SRAMBASE
	assert(Lpc546xxDmaChClaim(3, ChHandler, &s_HandlerCnt, 4, true));
	assert(SYSCON->AHBCLKCTRLSET[0] == SYSCON_AHBCLKCTRL_DMA_MASK);
	assert(s_DmaRegs.CTRL == DMA_CTRL_ENABLE_MASK);
	assert(s_DmaRegs.SRAMBASE != 0 && (s_DmaRegs.SRAMBASE & 511U) == 0);
	assert(s_DmaM.IntEn == (1UL << 3) && (s_NvicEn & (1UL << DMA0_IRQn)) && s_NvicPrio[DMA0_IRQn] == 4);

	// Taken again with the same handler and context, not with another
	assert(Lpc546xxDmaChClaim(3, ChHandler, &s_HandlerCnt, 4, true));
	assert(!Lpc546xxDmaChClaim(3, ChHandler2, &s_HandlerCnt, 4, true));
	assert(!Lpc546xxDmaChClaim(3, ChHandler, nullptr, 4, true));

	// The vector priority is the most urgent one asked
	assert(Lpc546xxDmaChClaim(5, ChHandler, &s_HandlerCnt, 2, true));
	assert(s_NvicPrio[DMA0_IRQn] == 2);
	assert(Lpc546xxDmaChClaim(7, ChHandler2, nullptr, 1, false));
	assert(s_NvicPrio[DMA0_IRQn] == 2 && (s_DmaM.IntEn & (1UL << 7)) == 0);
	Lpc546xxDmaChFree(5);
	assert(s_NvicPrio[DMA0_IRQn] == 4 && (s_DmaM.IntEn & (1UL << 5)) == 0);

	// Start refused before setup, with a bad width or size, or misaligned
	static uint8_t mem[4096] __attribute__((aligned(4)));
	assert(!Lpc546xxDmaChStart(3, mem, 4));
	assert(Lpc546xxDmaChFlags(3) & LPC546XX_DMA_FLAG_ERR);
	Lpc546xxDmaChSetup(3, true, &s_Usart[1].FIFOWR, 3);
	assert(Lpc546xxDmaChFlags(3) & LPC546XX_DMA_FLAG_ERR);
	assert(!Lpc546xxDmaChStart(3, mem, 4));
	Lpc546xxDmaChSetup(3, true, (volatile void *)&s_DmaRegs.CTRL, 2);
	assert(Lpc546xxDmaChFlags(3) == 0);
	assert(!Lpc546xxDmaChStart(3, mem, 0));
	assert(!Lpc546xxDmaChStart(3, mem, 6));
	assert(!Lpc546xxDmaChStart(3, &mem[2], 4));
	assert(!Lpc546xxDmaChStart(3, mem, 4100));
	assert(!Lpc546xxDmaChStart(3, nullptr, 4));
	assert(s_DmaM.Enable == 0);

	// A word transfer of 1024 values is the longest. Descriptor end addresses
	// are those of the last value.
	assert(Lpc546xxDmaChStart(3, mem, 4096));
	Lpc546xxDmaChStop(3);
	assert(Lpc546xxDmaChStart(3, mem, 4092));
	const uint32_t *desc = (const uint32_t *)(uintptr_t)(s_DmaRegs.SRAMBASE + 3 * 16U);
	assert(desc[1] == (uint32_t)(uintptr_t)&mem[4088] && desc[2] == (uint32_t)(uintptr_t)&s_DmaRegs.CTRL);
	uint32_t x = s_DmaRegs.CHANNEL[3].XFERCFG.value;
	assert((x & DMA_CHANNEL_XFERCFG_WIDTH_MASK) == DMA_CHANNEL_XFERCFG_WIDTH(2));
	assert((x & DMA_CHANNEL_XFERCFG_SRCINC_MASK) == DMA_CHANNEL_XFERCFG_SRCINC(1));
	assert((x & DMA_CHANNEL_XFERCFG_DSTINC_MASK) == 0);
	assert(((x & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >> DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT) == 1022);
	assert(Lpc546xxDmaChRemain(3) == 1023 * 4);
	assert(s_DmaM.Enable == (1UL << 3) && (s_DmaM.Trig & (1UL << 3)));

	// Stop with the count left
	s_DmaRegs.CHANNEL[3].XFERCFG.value = (x & ~DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) | DMA_CHANNEL_XFERCFG_XFERCOUNT(9);
	assert(Lpc546xxDmaChStopRemain(3) == 40);
	assert(s_DmaM.Enable == 0 && (s_DmaM.Trig & (1UL << 3)) == 0 && Lpc546xxDmaChRemain(3) == 0);

	// Peripheral to memory without memory increment
	Lpc546xxDmaChSetup(3, false, (volatile void *)&s_DmaRegs.CTRL, 0);
	Lpc546xxDmaChMemInc(3, false);
	assert(Lpc546xxDmaChStart(3, &mem[1], 5));
	assert(desc[1] == (uint32_t)(uintptr_t)&s_DmaRegs.CTRL && desc[2] == (uint32_t)(uintptr_t)&mem[1]);
	x = s_DmaRegs.CHANNEL[3].XFERCFG.value;
	assert((x & (DMA_CHANNEL_XFERCFG_SRCINC_MASK | DMA_CHANNEL_XFERCFG_DSTINC_MASK)) == 0);
	Lpc546xxDmaChStop(3);

	// A 1024 value transfer not begun is not mistaken for a done one
	Lpc546xxDmaChSetup(3, true, (volatile void *)&s_DmaRegs.CTRL, 0);
	assert(Lpc546xxDmaChStart(3, mem, 1024));
	assert(Lpc546xxDmaChRemain(3) == 1024);
	s_DmaRegs.COMMON[0].INTA.value |= 1UL << 3;
	assert(Lpc546xxDmaChRemain(3) == 0);
	// Done then its flags cleared: no longer active, nothing left
	s_DmaRegs.COMMON[0].ACTIVE &= ~(1UL << 3);
	s_DmaM.Trig &= ~(1UL << 3);
	Lpc546xxDmaChClear(3);
	assert(Lpc546xxDmaChFlags(3) == 0 && Lpc546xxDmaChRemain(3) == 0);
	Lpc546xxDmaChStop(3);

	// The vector takes the flags of each channel, clears them, calls the
	// handlers, and leaves channels without interrupt alone
	assert(Lpc546xxDmaChClaim(5, ChHandler, &s_HandlerCnt, 2, true));
	s_HandlerCnt = 0;
	s_DmaRegs.COMMON[0].INTA.value |= (1UL << 3) | (1UL << 7);
	s_DmaRegs.COMMON[0].ERRINT.value |= 1UL << 5;
	DMA0_IRQHandler();
	assert(s_HandlerCnt == 2 && s_HandlerCh[0] == 3 && s_HandlerCh[1] == 5);
	assert(s_HandlerFlags[0] == LPC546XX_DMA_FLAG_DONE && s_HandlerFlags[1] == LPC546XX_DMA_FLAG_ERR);
	assert(s_DmaRegs.COMMON[0].INTA == (1UL << 7) && s_DmaRegs.COMMON[0].ERRINT == 0);
	assert(Lpc546xxDmaChFlags(3) == LPC546XX_DMA_FLAG_DONE && Lpc546xxDmaChFlags(5) == LPC546XX_DMA_FLAG_ERR);
	Lpc546xxDmaChClear(3);
	assert(Lpc546xxDmaChFlags(3) == 0);

	// An error of a channel without interrupt enters the vector all the same:
	// taken into its flags and cleared, its handler not called
	s_HandlerCnt = 0;
	s_Handler2Cnt = 0;
	s_DmaRegs.COMMON[0].ERRINT.value |= 1UL << 7;
	DmaIrq();
	assert(s_HandlerCnt == 0 && s_Handler2Cnt == 0);
	assert(s_DmaRegs.COMMON[0].ERRINT == 0 && s_DmaRegs.COMMON[0].INTA == 0);
	assert(Lpc546xxDmaChFlags(7) == (LPC546XX_DMA_FLAG_DONE | LPC546XX_DMA_FLAG_ERR));
	Lpc546xxDmaChClear(7);
	assert(Lpc546xxDmaChFlags(7) == 0);

	// The last channel freed turns the controller off
	Lpc546xxDmaChFree(3);
	Lpc546xxDmaChFree(5);
	assert(s_DmaRegs.CTRL == DMA_CTRL_ENABLE_MASK);
	Lpc546xxDmaChFree(7);
	assert(s_DmaRegs.CTRL == 0 && SYSCON->AHBCLKCTRLCLR[0] == SYSCON_AHBCLKCTRL_DMA_MASK);
	assert((s_NvicEn & (1UL << DMA0_IRQn)) == 0);
	Lpc546xxDmaChFree(7);

	// Free channels can be taken by another owner
	assert(Lpc546xxDmaChClaim(3, ChHandler2, nullptr, 1, false));
	Lpc546xxDmaChFree(3);
	printf("LPC546xx DMA channels: claim, setup, start checks, remain, stop, vector dispatch: PASS\n");
}

static void UartDmaTest(void)
{
	UARTDev_t uart;
	alignas(4) static uint8_t txmem[CFIFO_MEMSIZE(64)];
	int ch = LPC546XX_DMA_CH_FC_TX(0);

	for (size_t i = 0; i < sizeof(s_Pattern); i++)
	{
		s_Pattern[i] = (uint8_t)(i * 7 + (i >> 8));
	}

	// Interrupt mode, blocking CFIFO: blocks sent in place, the order kept
	// over CFIFO wraps
	ModelInit();
	memset((void *)&uart, 0, sizeof(uart));
	UARTCfg_t cfg = UartCfg(true, true, true, txmem, sizeof(txmem));
	assert(UARTInit(&uart, &cfg));
	assert(s_Usart[0].FIFOCFG & USART_FIFOCFG_DMATX_MASK);
	assert((s_Usart[0].FIFOINTENSET & USART_FIFOINTENSET_TXLVL_MASK) == 0);
	// DMA request while the TX FIFO is not full, by DMATX or by the level
	assert((s_Usart[0].FIFOTRIG & USART_FIFOTRIG_TXLVL_MASK) == USART_FIFOTRIG_TXLVL(15));
	assert(uart.DevIntrf.bDma && s_DmaM.IntEn == (1UL << ch) && s_NvicPrio[DMA0_IRQn] == 3);
	assert(s_DmaRegs.CHANNEL[ch].CFG == DMA_CHANNEL_CFG_PERIPHREQEN_MASK);

	s_TxReadyCnt = 0;
	assert(UARTTx(&uart, s_Pattern, 40) == 40);
	assert(uart.bTxReady == false && (s_DmaM.Trig & (1UL << ch)));
	const uint32_t *desc = (const uint32_t *)(uintptr_t)(s_DmaRegs.SRAMBASE + ch * 16U);
	// In place: the source is the CFIFO memory
	assert(desc[1] >= (uint32_t)(uintptr_t)txmem && desc[1] < (uint32_t)(uintptr_t)txmem + sizeof(txmem));
	Run(0);
	assert(uart.bTxReady && s_TxReadyCnt == 1 && s_TxReadyLen == CFifoAvail(uart.hTxFifo));
	assert(CFifoUsed(uart.hTxFifo) == 0);

	// Data queued while a block runs goes in the next block, wrapping the
	// CFIFO end
	assert(UARTTx(&uart, &s_Pattern[40], 20) == 20);
	DmaStep();
	assert(UARTTx(&uart, &s_Pattern[60], 30) == 30);
	Run(0);
	Send(&uart, &s_Pattern[90], 1500);
	Run(0);
	assert(s_UsartM[0].Line.size() == 1590 && memcmp(s_UsartM[0].Line.data(), s_Pattern, 1590) == 0);
	assert(!s_UsartM[0].bTxErr && uart.TxDropCnt == 0 && uart.bTxReady);

	// A DMA error drops the bytes left of the block, the next block goes out
	s_UsartM[0].Line.clear();
	assert(UARTTx(&uart, s_Pattern, 10) == 10);
	s_DmaM.ErrCh = ch;
	DmaStep();
	assert(UARTTx(&uart, &s_Pattern[10], 5) == 5);
	Run(0);
	assert(uart.TxDropCnt == 10 && s_UsartM[0].Line.size() == 5 && memcmp(s_UsartM[0].Line.data(), &s_Pattern[10], 5) == 0);
	assert(uart.bTxReady);

	// Disable during a block stops the channel, Enable restarts clean
	s_UsartM[0].Line.clear();
	assert(UARTTx(&uart, s_Pattern, 30) == 30);
	DmaStep();
	UARTDisable(&uart);
	assert((s_DmaM.Enable & (1UL << ch)) == 0 && (s_DmaM.Trig & (1UL << ch)) == 0);
	s_UsartM[0].Cnt = 0;
	UARTEnable(&uart);
	assert(uart.bTxReady && CFifoUsed(uart.hTxFifo) == 0);
	s_UsartM[0].Line.clear();
	Send(&uart, &s_Pattern[100], 200);
	Run(0);
	assert(s_UsartM[0].Line.size() == 200 && memcmp(s_UsartM[0].Line.data(), &s_Pattern[100], 200) == 0);

	// Interrupt mode, non-blocking CFIFO: copies of at most 64 bytes
	UsartModelInit();
	alignas(4) static uint8_t txmem2[CFIFO_MEMSIZE(128)];
	memset((void *)&uart, 0, sizeof(uart));
	cfg = UartCfg(true, true, false, txmem2, sizeof(txmem2));
	assert(UARTInit(&uart, &cfg));
	assert(UARTTx(&uart, s_Pattern, 100) == 100);
	desc = (const uint32_t *)(uintptr_t)(s_DmaRegs.SRAMBASE + ch * 16U);
	assert(((s_DmaRegs.CHANNEL[ch].XFERCFG.value & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >>
			DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT) == 63);
	assert(CFifoUsed(uart.hTxFifo) == 36);
	Run(0);
	SendWait(&uart, &s_Pattern[100], 900);
	Run(0);
	assert(s_UsartM[0].Line.size() == 1000 && memcmp(s_UsartM[0].Line.data(), s_Pattern, 1000) == 0);
	assert(uart.TxDropCnt == 0 && uart.bTxReady);

	// A put on the full CFIFO while a block runs drops the oldest CFIFO bytes,
	// not the copy the DMA sends
	s_UsartM[0].Line.clear();
	int cap = CFifoAvail(uart.hTxFifo);
	assert(UARTTx(&uart, s_Pattern, 100) == 100);
	assert(UARTTx(&uart, &s_Pattern[100], 200) == 200);
	assert(CFifoUsed(uart.hTxFifo) == cap);
	Run(0);
	assert((int)s_UsartM[0].Line.size() == 64 + cap);
	assert(memcmp(s_UsartM[0].Line.data(), s_Pattern, 64) == 0);
	assert(memcmp(&s_UsartM[0].Line[64], &s_Pattern[300 - cap], cap) == 0);

	// Polling with DMA: no interrupt, the next TxData handles the block done
	UsartModelInit();
	memset((void *)&uart, 0, sizeof(uart));
	cfg = UartCfg(false, true, true, txmem, sizeof(txmem));
	assert(UARTInit(&uart, &cfg));
	assert(s_DmaM.IntEn == 0 && (s_NvicEn & (1UL << DMA0_IRQn)) == 0);
	s_TxReadyCnt = 0;
	assert(UARTTx(&uart, s_Pattern, 50) == 50);
	Run(0);
	assert(uart.bTxReady == false && (Lpc546xxDmaChFlags(ch) & LPC546XX_DMA_FLAG_DONE));
	Send(&uart, &s_Pattern[50], 2000);
	Run(0);
	assert(UARTTx(&uart, &s_Pattern[2050], 1) == 1);
	Run(0);
	// RxData also handles the block done
	uint8_t rx[4];
	for (int i = 0; i < 4; i++)
	{
		assert(UARTRx(&uart, rx, sizeof(rx)) == 0);
		Run(0);
	}
	assert(CFifoUsed(uart.hTxFifo) == 0 && uart.bTxReady);
	assert(s_UsartM[0].Line.size() == 2051 && memcmp(s_UsartM[0].Line.data(), s_Pattern, 2051) == 0);
	assert(s_TxReadyCnt == 0 && uart.TxDropCnt == 0);

	// Back to the interrupt mode without DMA: the channel is given back and
	// the TX level interrupt drains the CFIFO
	cfg = UartCfg(true, false, true, txmem, sizeof(txmem));
	assert(UARTInit(&uart, &cfg));
	assert((s_Usart[0].FIFOCFG & USART_FIFOCFG_DMATX_MASK) == 0 && uart.DevIntrf.bDma == false);
	assert((s_Usart[0].FIFOTRIG & USART_FIFOTRIG_TXLVL_MASK) == USART_FIFOTRIG_TXLVL(8));
	assert(s_DmaRegs.CTRL == 0);
	assert(Lpc546xxDmaChClaim(ch, ChHandler2, nullptr, 1, false));

	// The TX channel taken by another owner, bDMAMode fails
	cfg = UartCfg(true, true, true, txmem, sizeof(txmem));
	assert(!UARTInit(&uart, &cfg));
	Lpc546xxDmaChFree(ch);
	assert(UARTInit(&uart, &cfg));

	printf("LPC546xx UART TX DMA: in place, copy, wrap, error, disable, polling, release: PASS\n");
}

int main(void)
{
	DmaChannelTest();
	UartDmaTest();

	return 0;
}
