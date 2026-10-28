// Host model of the LPC546xx DMA controller, shared by the LPC546xx host
// regressions.
//
// The model reads the channel descriptor from the table at SRAMBASE and moves
// one value per step while the channel is enabled, triggered by SWTRIG and
// its peripheral request is on. The end of a descriptor sets INTA when
// SETINTA is set, clears the trigger (CLRTRIG is required) and calls the DMA0
// vector when its interrupt is enabled and not masked. An error sets ERRINT,
// which asks for the vector whatever INTENSET. The vector is entered again
// while a flag is pending, as the NVIC does, and a vector that leaves a flag
// pending fails the test. A test sets ErrCh for an error on the next value of
// a channel. With Immediate, a channel moves its first value when it is
// triggered, if its request is on, as the controller does while the CPU goes
// on: a driver that changes a buffer after starting the DMA on it is seen.
//
// The test defines, before including the drivers:
//   static bool DmaReqOn(int Ch);                         request of a channel
//   static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal); peripheral read
//   static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val);  peripheral write
// The two last return false when Addr is memory.
#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>

struct DmaModel;
struct DmaSetReg {
	uint32_t *p = nullptr;
	operator uint32_t() const { return *p; }
	DmaSetReg &operator=(uint32_t v) { *p |= v; return *this; }
};
struct DmaClrReg {
	uint32_t *p = nullptr;
	DmaClrReg &operator=(uint32_t v) { *p &= ~v; return *this; }
};
struct DmaW1cReg {
	uint32_t value = 0;
	operator uint32_t() const { return value; }
	DmaW1cReg &operator=(uint32_t v) { value &= ~v; return *this; }
};
struct DmaAbortReg {
	DmaAbortReg &operator=(uint32_t v);
};
struct DmaXferCfgReg {
	int Ch = 0;
	uint32_t value = 0;
	operator uint32_t() const { return value; }
	DmaXferCfgReg &operator=(uint32_t v);
};
struct DmaCommon {
	DmaSetReg ENABLESET;
	DmaClrReg ENABLECLR;
	uint32_t ACTIVE = 0, BUSY = 0;
	DmaW1cReg ERRINT;
	DmaSetReg INTENSET;
	DmaClrReg INTENCLR;
	DmaW1cReg INTA, INTB;
	uint32_t SETVALID = 0, SETTRIG = 0;
	DmaAbortReg ABORT;
};
struct DmaChannel {
	uint32_t CFG = 0, CTLSTAT = 0;
	DmaXferCfgReg XFERCFG;
};
struct DmaRegs {
	uint32_t CTRL = 0, INTSTAT = 0, SRAMBASE = 0;
	DmaCommon COMMON[1];
	DmaChannel CHANNEL[30];
};
struct DmaModel {
	uint32_t Enable = 0, IntEn = 0;
	uint32_t Trig = 0;					// Triggered by SWTRIG
	int ErrCh = -1;						// Error on the next value of this channel
	int Values = 0;
	bool Immediate = false;				// First value moved at the trigger
};

static DmaRegs s_DmaRegs;
static DmaModel s_DmaM;
extern uint32_t g_IrqMasked;
static uint32_t s_NvicEn = 0;
[[maybe_unused]] static uint32_t s_NvicPrio[64];

static bool DmaReqOn(int Ch);
static bool DmaPeriphRead(uint32_t Addr, uint32_t *pVal);
static bool DmaPeriphWrite(uint32_t Addr, uint32_t Val);
extern "C" void DMA0_IRQHandler(void);
static int DmaChStep(int ch, bool bIrq);

inline DmaAbortReg &DmaAbortReg::operator=(uint32_t v)
{
	s_DmaRegs.COMMON[0].ACTIVE &= ~v;
	s_DmaM.Trig &= ~v;
	return *this;
}

inline DmaXferCfgReg &DmaXferCfgReg::operator=(uint32_t v)
{
	value = v & ~DMA_CHANNEL_XFERCFG_SWTRIG_MASK;
	if (v & DMA_CHANNEL_XFERCFG_SWTRIG_MASK)
	{
		// The descriptor must be valid and the channel enabled
		assert(v & DMA_CHANNEL_XFERCFG_CFGVALID_MASK);
		assert(s_DmaM.Enable & (1UL << Ch));
		s_DmaM.Trig |= 1UL << Ch;
		s_DmaRegs.COMMON[0].ACTIVE |= 1UL << Ch;
		if (s_DmaM.Immediate && (s_DmaRegs.CTRL & DMA_CTRL_ENABLE_MASK))
		{
			DmaChStep(Ch, false);
		}
	}
	return *this;
}

[[maybe_unused]] static void DmaModelInit(void)
{
	s_DmaRegs = DmaRegs();
	s_DmaM = DmaModel();
	DmaCommon &c = s_DmaRegs.COMMON[0];
	c.ENABLESET.p = &s_DmaM.Enable;
	c.ENABLECLR.p = &s_DmaM.Enable;
	c.INTENSET.p = &s_DmaM.IntEn;
	c.INTENCLR.p = &s_DmaM.IntEn;
	for (int i = 0; i < 30; i++)
	{
		s_DmaRegs.CHANNEL[i].XFERCFG.Ch = i;
	}
}

[[maybe_unused]] static bool DmaIrqPending(void)
{
	return (s_NvicEn & (1UL << DMA0_IRQn)) && g_IrqMasked == 0 &&
		   ((s_DmaRegs.COMMON[0].INTA & s_DmaM.IntEn) | s_DmaRegs.COMMON[0].ERRINT);
}

[[maybe_unused]] static void DmaIrq(void)
{
	for (int i = 0; DmaIrqPending(); i++)
	{
		assert(i < 4);
		DMA0_IRQHandler();
	}
}

// One value of a channel when it is ready. bIrq: call the vector at the end
// of a descriptor. Returns the number of values moved.
static int DmaChStep(int ch, bool bIrq)
{
	uint32_t bit = 1UL << ch;

	if ((s_DmaM.Enable & bit) == 0 || (s_DmaM.Trig & bit) == 0)
	{
		return 0;
	}
	assert(s_DmaRegs.CHANNEL[ch].CFG & DMA_CHANNEL_CFG_PERIPHREQEN_MASK);
	if (!DmaReqOn(ch))
	{
		return 0;
	}

	const uint32_t *desc = (const uint32_t *)(uintptr_t)(s_DmaRegs.SRAMBASE + ch * 16U);
	uint32_t cfg = s_DmaRegs.CHANNEL[ch].XFERCFG.value;
	uint32_t cnt = (cfg & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >> DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT;
	uint32_t w = 1U << ((cfg & DMA_CHANNEL_XFERCFG_WIDTH_MASK) >> DMA_CHANNEL_XFERCFG_WIDTH_SHIFT);
	uint32_t sinc = (cfg & DMA_CHANNEL_XFERCFG_SRCINC_MASK) >> DMA_CHANNEL_XFERCFG_SRCINC_SHIFT;
	uint32_t dinc = (cfg & DMA_CHANNEL_XFERCFG_DSTINC_MASK) >> DMA_CHANNEL_XFERCFG_DSTINC_SHIFT;

	assert(desc[3] == 0U);
	uint32_t src = desc[1] - cnt * w * sinc;
	uint32_t dst = desc[2] - cnt * w * dinc;

	if (s_DmaM.ErrCh == ch)
	{
		s_DmaM.ErrCh = -1;
		s_DmaRegs.COMMON[0].ERRINT.value |= bit;
		s_DmaM.Trig &= ~bit;
		s_DmaRegs.COMMON[0].ACTIVE &= ~bit;
		if (bIrq)
		{
			DmaIrq();
		}
		return 0;
	}

	uint32_t v = 0;

	if (!DmaPeriphRead(src, &v))
	{
		memcpy(&v, (const void *)(uintptr_t)src, w);
	}
	if (!DmaPeriphWrite(dst, v))
	{
		memcpy((void *)(uintptr_t)dst, &v, w);
	}
	s_DmaM.Values++;

	if (cnt == 0)
	{
		cnt = 0x3FF;
		if (cfg & DMA_CHANNEL_XFERCFG_SETINTA_MASK)
		{
			s_DmaRegs.COMMON[0].INTA.value |= bit;
		}
		assert(cfg & DMA_CHANNEL_XFERCFG_CLRTRIG_MASK);
		s_DmaM.Trig &= ~bit;
		s_DmaRegs.COMMON[0].ACTIVE &= ~bit;
	}
	else
	{
		cnt--;
	}
	s_DmaRegs.CHANNEL[ch].XFERCFG.value = (cfg & ~DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) |
										  DMA_CHANNEL_XFERCFG_XFERCOUNT(cnt);
	if (cnt == 0x3FF && bIrq)
	{
		DmaIrq();
	}

	return 1;
}

// One value on each channel ready. bIrq: call the vector at the end of a
// descriptor. Returns the number of values moved.
[[maybe_unused]] static int DmaStep(bool bIrq = true)
{
	int n = 0;

	if ((s_DmaRegs.CTRL & DMA_CTRL_ENABLE_MASK) == 0)
	{
		return 0;
	}
	assert((s_DmaRegs.SRAMBASE & 511U) == 0U);

	for (int ch = 0; ch < 30; ch++)
	{
		n += DmaChStep(ch, bIrq);
	}

	return n;
}

