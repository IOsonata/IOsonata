// Host regression of the LPC546xx Ethernet driver with a model of the ENET,
// its DMA channel 0 and an RMII PHY on MDIO.
//
// ENET model. The ENET registers are reset by the SYSCON reset and by the
// DMA software reset. The DMA software reset ends after a few microseconds
// when the PHY gives its reference clock and samples the PHY interface of
// ETHPHYSEL. An MDIO transfer (MB) ends on the next microsecond and reads or
// writes a register of the PHY at the address given, with the CR the test
// expects for the AHB clock.
//
// DMA model on the descriptors of the driver in host memory, the tail
// pointers compared for equality: the transmit DMA started (ST) or resumed by
// a tail write fetches the current descriptor until it reaches the tail or a
// descriptor it does not own (TBU), sends it, gives it back and sets TI (with
// IOC). The receive DMA stores a frame of the test for the MAC address or
// broadcast in the current descriptor when it owns it and it is not the tail,
// else it stops (RBU) until a tail write. The interrupt runs when enabled.
//
// PHY model: BMCR reset ends after a few reads, BMSR link latched low,
// register 31 the speed indication, the IDs of a LAN8720A.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "LPC546xx.h"

static int s_NvicEn = 0;
static int s_NvicPrio = -1;
static void Irq(void);
void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { assert(n == ETHERNET_IRQn); s_NvicPrio = (int)p; }
void NVIC_EnableIRQ(IRQn_Type n) { assert(n == ETHERNET_IRQn); s_NvicEn = 1; Irq(); }
void NVIC_DisableIRQ(IRQn_Type n) { assert(n == ETHERNET_IRQn); s_NvicEn = 0; }
void __DMB(void) {}
void __DSB(void) {}
void __NOP(void) {}

static uint32_t s_CoreClk = 220000000;
extern "C" uint32_t SystemCoreClockGet(void) { return s_CoreClk; }

#define OWN		0x80000000U
#define IOC		0x40000000U
#define FD		0x20000000U
#define LD		0x10000000U
#define BUF1V	0x01000000U

// ---- PHY ----

struct Phy {
	uint16_t Reg[32] = {};
	int Addr = 0;
	bool RefClk = true;			// 50 MHz reference clock on REF_CLK
	int ResetReads = 3;			// BMCR reads before the reset ends
	bool ResetStuck = false;
	bool Link = false;
	bool AnDone = true;			// Auto negotiation complete
	bool LinkLatched = false;	// Link went down since the last BMSR read
	uint16_t Mode = 6;			// Register 31 speed: 6 100 full
	int Bmcr8000 = 0, Bmcr1200 = 0;
};
static Phy s_Phy;

static uint16_t PhyRead(int Addr, int Reg)
{
	if (Addr != s_Phy.Addr)
	{
		return 0xFFFF;
	}
	switch (Reg)
	{
		case 0:
			if ((s_Phy.Reg[0] & 0x8000U) && !s_Phy.ResetStuck && --s_Phy.ResetReads <= 0)
			{
				s_Phy.Reg[0] = 0x3000;
			}
			return s_Phy.Reg[0];
		case 1:
		{
			uint16_t v = 0x7809 | (s_Phy.Link && !s_Phy.LinkLatched ? 0x0004U : 0U) | (s_Phy.AnDone ? 0x0020U : 0U);
			s_Phy.LinkLatched = false;
			return v;
		}
		case 31:
			return (uint16_t)(0x0040 | (s_Phy.Mode << 2));
		default:
			return s_Phy.Reg[Reg];
	}
}

static void PhyWrite(int Addr, int Reg, uint16_t v)
{
	if (Addr != s_Phy.Addr)
	{
		return;
	}
	if (Reg == 0)
	{
		s_Phy.Bmcr8000 += v == 0x8000;
		s_Phy.Bmcr1200 += v == 0x1200;
	}
	s_Phy.Reg[Reg] = v;
}

// ---- SYSCON: the ENET clock, reset and PHY selection ----

static bool s_EthClk, s_EthRst;
static uint32_t s_PhySel;
static int s_RstPulses;
static void EnetReset(void);

struct SysW {
	int Kind, Idx;
	SysW &operator=(uint32_t v)
	{
		if (Idx != 2 || (v & 0x100U) == 0)
		{
			return *this;
		}
		switch (Kind)
		{
			case 0: s_EthClk = true; break;
			case 1: s_EthClk = false; break;
			case 2: s_EthRst = true; EnetReset(); break;
			case 3: s_RstPulses += s_EthRst; s_EthRst = false; break;
		}
		return *this;
	}
};
struct PhySelR {
	operator uint32_t() const { return s_PhySel; }
	PhySelR &operator=(uint32_t v) { s_PhySel = v; return *this; }
};
struct SysRegs {
	SysW AHBCLKCTRLSET[3] = { { 0, 0 }, { 0, 1 }, { 0, 2 } };
	SysW AHBCLKCTRLCLR[3] = { { 1, 0 }, { 1, 1 }, { 1, 2 } };
	SysW PRESETCTRLSET[3] = { { 2, 0 }, { 2, 1 }, { 2, 2 } };
	SysW PRESETCTRLCLR[3] = { { 3, 0 }, { 3, 1 }, { 3, 2 } };
	PhySelR ETHPHYSEL;
};
static SysRegs s_SysRegs;

// ---- ENET ----

struct Desc { uint32_t W[4]; };

// Register values, cleared by a reset
struct EthState {
	uint32_t MacCfg, Filter, AddrLo, AddrHi, Tic, RxqCtrl, MacIntEn;
	uint32_t ChCtrl, TxCtrl, RxCtrl, TxList, RxList, TxTail, RxTail, TxLen, RxLen, IntEn, Stat;
	uint32_t TxQ, RxQ;
	int TxCur, RxCur;			// DMA descriptor index
	int TxFetched;				// Descriptor being sent, -1 none
	bool RxStopped;				// Receive DMA stopped on a descriptor (RBU)
	bool Rmii;					// PHY interface sampled by the DMA reset
	int Flushes;
};
static EthState s_E;

// Test conditions and results, kept through a reset
struct Model {
	bool MdioStuck = false;		// MDIO transfer never ends
	int MdioHangAt = -1;		// This MDIO transfer never ends
	uint32_t MdioPend = 0;		// MDIO transfer not ended
	int RxTails = 0;			// Receive tail writes
	bool TxHold = false;		// The descriptor fetched is not sent
	bool BusError = false;		// The next transmit descriptor fetch fails
	int SwrUs = 0;
	int MdioOps = 0;
	std::vector<std::vector<uint8_t>> Wire;	// Frames sent
	int Lost = 0;
	int Irqs = 0;
};
static Model s_M;
static uint64_t s_Us;

static void DmaRun(void);

static bool EnetOn(void)
{
	return s_EthClk && !s_EthRst;
}

struct CfgR {
	uint32_t EthState::*f;
	operator uint32_t() const { return s_E.*f; }
	CfgR &operator=(uint32_t v)
	{
		assert(EnetOn());
		s_E.*f = v;
		return *this;
	}
	CfgR &operator|=(uint32_t v) { return *this = (s_E.*f | v); }
	CfgR &operator&=(uint32_t v) { return *this = (s_E.*f & v); }
};
struct TxCtrlR {
	operator uint32_t() const { return s_E.TxCtrl; }
	TxCtrlR &operator=(uint32_t v)
	{
		assert(EnetOn());
		if ((s_E.TxCtrl & ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK) && (v & ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK) == 0)
		{
			s_E.TxFetched = -1;
			s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_TPS_MASK;
		}
		s_E.TxCtrl = v;
		DmaRun();
		return *this;
	}
	TxCtrlR &operator|=(uint32_t v) { return *this = (s_E.TxCtrl | v); }
	TxCtrlR &operator&=(uint32_t v) { return *this = (s_E.TxCtrl & v); }
};
struct RxCtrlR {
	operator uint32_t() const { return s_E.RxCtrl; }
	RxCtrlR &operator=(uint32_t v)
	{
		assert(EnetOn());
		if ((s_E.RxCtrl & ENET_DMA_CH_DMA_CHX_RX_CTRL_SR_MASK) && (v & ENET_DMA_CH_DMA_CHX_RX_CTRL_SR_MASK) == 0)
		{
			s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_RPS_MASK;
		}
		s_E.RxCtrl = v;
		s_E.RxStopped = false;
		return *this;
	}
	RxCtrlR &operator|=(uint32_t v) { return *this = (s_E.RxCtrl | v); }
	RxCtrlR &operator&=(uint32_t v) { return *this = (s_E.RxCtrl & v); }
};
// Descriptor list address: the DMA starts again from it
struct ListR {
	bool Tx;
	operator uint32_t() const { return Tx ? s_E.TxList : s_E.RxList; }
	ListR &operator=(uint32_t v)
	{
		assert(EnetOn() && (v & 15U) == 0);
		(Tx ? s_E.TxList : s_E.RxList) = v;
		(Tx ? s_E.TxCur : s_E.RxCur) = 0;
		return *this;
	}
};
// Tail pointer: a descriptor of the ring or one past the last
struct TailR {
	bool Tx;
	operator uint32_t() const { return Tx ? s_E.TxTail : s_E.RxTail; }
	TailR &operator=(uint32_t v)
	{
		uint32_t list = Tx ? s_E.TxList : s_E.RxList;
		uint32_t len = Tx ? s_E.TxLen : s_E.RxLen;

		assert(EnetOn() && v >= list && v <= list + 16U * (len + 1U) && ((v - list) & 15U) == 0);
		if (Tx)
		{
			s_E.TxTail = v;
			DmaRun();
		}
		else
		{
			s_E.RxTail = v;
			s_E.RxStopped = false;
			s_M.RxTails++;
		}
		return *this;
	}
};
struct StatR {
	operator uint32_t() const { return s_E.Stat; }
	StatR &operator=(uint32_t v) { s_E.Stat &= ~v; return *this; }
};
struct TxQR {
	operator uint32_t() const { return s_E.TxQ; }
	TxQR &operator=(uint32_t v)
	{
		assert(EnetOn());
		if (v & ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_FTQ_MASK)
		{
			// The flush ends at once
			s_E.Flushes++;
			v &= ~ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_FTQ_MASK;
		}
		s_E.TxQ = v;
		return *this;
	}
	TxQR &operator|=(uint32_t v) { return *this = (s_E.TxQ | v); }
};
struct EnetCh {
	CfgR DMA_CHX_CTRL = { &EthState::ChCtrl };
	TxCtrlR DMA_CHX_TX_CTRL;
	RxCtrlR DMA_CHX_RX_CTRL;
	ListR DMA_CHX_TXDESC_LIST_ADDR = { true };
	ListR DMA_CHX_RXDESC_LIST_ADDR = { false };
	TailR DMA_CHX_TXDESC_TAIL_PTR = { true };
	TailR DMA_CHX_RXDESC_TAIL_PTR = { false };
	CfgR DMA_CHX_TXDESC_RING_LENGTH = { &EthState::TxLen };
	CfgR DMA_CHX_RXDESC_RING_LENGTH = { &EthState::RxLen };
	CfgR DMA_CHX_INT_EN = { &EthState::IntEn };
	StatR DMA_CHX_STAT;
};
struct MtlQ {
	TxQR MTL_TXQX_OP_MODE;
	CfgR MTL_RXQX_OP_MODE = { &EthState::RxQ };
};
struct EnetRegs {
	CfgR MAC_CONFIG = { &EthState::MacCfg };
	CfgR MAC_FRAME_FILTER = { &EthState::Filter };
	CfgR MAC_ADDR_LOW = { &EthState::AddrLo };
	CfgR MAC_ADDR_HIGH = { &EthState::AddrHi };
	CfgR MAC_1US_TIC_COUNTR = { &EthState::Tic };
	CfgR MAC_RXQ_CTRL[1] = { { &EthState::RxqCtrl } };
	CfgR MAC_INTR_EN = { &EthState::MacIntEn };
	volatile uint32_t MAC_MDIO_ADDR;
	volatile uint32_t MAC_MDIO_DATA;
	volatile uint32_t DMA_MODE;
	MtlQ MTL_QUEUE[1];
	EnetCh DMA_CH[1];
};
static EnetRegs s_Enet;

static void EnetReset(void)
{
	s_E = EthState();
	s_E.TxFetched = -1;
	s_E.Filter = 0x80000000U;	// Receive all, the driver clears it
	s_Enet.MAC_MDIO_ADDR = 0;
	s_Enet.MAC_MDIO_DATA = 0;
	s_Enet.DMA_MODE = 0;
	s_M.SwrUs = 0;
	s_M.MdioPend = 0;
}

// MDC 2.5 MHz or less from the AHB clock
static uint32_t ExpectCr(uint32_t Clk)
{
	if (Clk < 35000000)
	{
		return 2;
	}
	if (Clk < 60000000)
	{
		return 3;
	}
	if (Clk < 100000000)
	{
		return 0;
	}
	if (Clk < 150000000)
	{
		return 1;
	}
	return Clk < 250000000 ? 4 : 5;
}

// One microsecond of the ENET: the DMA reset and an MDIO transfer
static void Tick(void)
{
	if ((s_Enet.DMA_MODE & ENET_DMA_MODE_SWR_MASK) && EnetOn() && s_Phy.RefClk && ++s_M.SwrUs >= 3)
	{
		EnetReset();
		s_E.Rmii = (s_PhySel & SYSCON_ETHPHYSEL_PHY_SEL_MASK) != 0;
	}
	uint32_t v = s_Enet.MAC_MDIO_ADDR;
	// No new transfer written before the previous one ends
	assert(s_M.MdioPend == 0 || v == s_M.MdioPend);
	if ((v & ENET_MAC_MDIO_ADDR_MB_MASK) && EnetOn() && (s_M.MdioStuck || s_M.MdioOps == s_M.MdioHangAt))
	{
		s_M.MdioPend = v;
	}
	else if ((v & ENET_MAC_MDIO_ADDR_MB_MASK) && EnetOn())
	{
		s_M.MdioPend = 0;
		int pa = (int)((v & ENET_MAC_MDIO_ADDR_PA_MASK) >> ENET_MAC_MDIO_ADDR_PA_SHIFT);
		int rda = (int)((v & ENET_MAC_MDIO_ADDR_RDA_MASK) >> ENET_MAC_MDIO_ADDR_RDA_SHIFT);
		uint32_t moc = (v & ENET_MAC_MDIO_ADDR_MOC_MASK) >> ENET_MAC_MDIO_ADDR_MOC_SHIFT;

		// Clause 22, the clock divider of the AHB clock
		assert((v & 2U) == 0 && ((v & ENET_MAC_MDIO_ADDR_CR_MASK) >> ENET_MAC_MDIO_ADDR_CR_SHIFT) == ExpectCr(s_CoreClk));
		assert(s_E.Rmii);
		if (moc == 1)
		{
			PhyWrite(pa, rda, (uint16_t)s_Enet.MAC_MDIO_DATA);
		}
		else
		{
			assert(moc == 3);
			s_Enet.MAC_MDIO_DATA = PhyRead(pa, rda);
		}
		s_Enet.MAC_MDIO_ADDR = v & ~ENET_MAC_MDIO_ADDR_MB_MASK;
		s_M.MdioOps++;
	}
}

void ModelDelayUs(uint32_t Us)
{
	for (uint32_t i = 0; i < Us; i++)
	{
		s_Us++;
		Tick();
	}
}

extern "C" void ETHERNET_IRQHandler(void);
static int s_InIrq = 0;

static void Irq(void)
{
	uint32_t en = s_E.IntEn;
	uint32_t st = s_E.Stat & en;
	bool pend = ((st & (ENET_DMA_CH_DMA_CHX_STAT_TI_MASK | ENET_DMA_CH_DMA_CHX_STAT_RI_MASK)) &&
				 (en & ENET_DMA_CH_DMA_CHX_INT_EN_NIE_MASK)) ||
				((st & ENET_DMA_CH_DMA_CHX_STAT_FBE_MASK) && (en & ENET_DMA_CH_DMA_CHX_INT_EN_AIE_MASK));

	if (s_NvicEn && !s_InIrq && pend)
	{
		s_InIrq = 1;
		s_M.Irqs++;
		ETHERNET_IRQHandler();
		s_InIrq = 0;
	}
}

static uint32_t TxAddr(int i) { return s_E.TxList + 16U * (uint32_t)i; }
static uint32_t RxAddr(int i) { return s_E.RxList + 16U * (uint32_t)i; }

// Host addresses of the descriptors and buffers below 4 GB (-no-pie, static
// memory)
static void DmaRun(void)
{
	if (!EnetOn() || !(s_E.MacCfg & ENET_MAC_CONFIG_TE_MASK) || !(s_E.TxCtrl & ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK))
	{
		return;
	}
	assert(s_E.TxLen == 3 && s_E.ChCtrl == 0);
	for (;;)
	{
		if (s_E.TxFetched < 0)
		{
			if (TxAddr(s_E.TxCur) == s_E.TxTail)
			{
				break;
			}
			Desc *d = (Desc *)(uintptr_t)TxAddr(s_E.TxCur);
			if ((d->W[3] & OWN) == 0)
			{
				s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_TBU_MASK;
				break;
			}
			if (s_M.BusError)
			{
				s_M.BusError = false;
				s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_FBE_MASK | ENET_DMA_CH_DMA_CHX_STAT_AIS_MASK;
				s_E.TxCtrl &= ~ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK;
				break;
			}
			s_E.TxFetched = s_E.TxCur;
		}
		if (s_M.TxHold)
		{
			break;
		}
		Desc *d = (Desc *)(uintptr_t)TxAddr(s_E.TxFetched);
		// One frame per descriptor: first and last, buffer 1 only, CRC and
		// padding added
		assert((d->W[3] & (FD | LD)) == (FD | LD) && d->W[1] == 0 && (d->W[2] & 0x3FFF0000U) == 0);
		assert((d->W[3] & 0x0C030000U) == 0);
		uint32_t len = d->W[3] & 0x7FFFU;
		assert(len == (d->W[2] & 0x3FFFU));
		const uint8_t *p = (const uint8_t *)(uintptr_t)d->W[0];
		s_M.Wire.push_back(std::vector<uint8_t>(p, p + len));
		d->W[0] = 0xAAAA;
		d->W[1] = 0xBBBB;
		d->W[3] = FD | LD;
		if (d->W[2] & 0x80000000U)
		{
			s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_TI_MASK | ENET_DMA_CH_DMA_CHX_STAT_NIS_MASK;
		}
		s_E.TxCur = (s_E.TxCur + 1) % 4;
		s_E.TxFetched = -1;
	}
	Irq();
}

// A frame from the network, without its FCS. Flags for the write back.
static bool Receive(const std::vector<uint8_t> &Frame, uint32_t Flags = 0)
{
	if (!EnetOn() || !s_Phy.Link || !(s_E.MacCfg & ENET_MAC_CONFIG_RE_MASK) ||
		!(s_E.RxCtrl & ENET_DMA_CH_DMA_CHX_RX_CTRL_SR_MASK))
	{
		s_M.Lost++;
		return false;
	}
	assert(s_E.RxLen == 3);
	bool bcast = memcmp(Frame.data(), "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0;
	uint8_t own[6] = { (uint8_t)s_E.AddrLo, (uint8_t)(s_E.AddrLo >> 8), (uint8_t)(s_E.AddrLo >> 16),
					   (uint8_t)(s_E.AddrLo >> 24), (uint8_t)s_E.AddrHi, (uint8_t)(s_E.AddrHi >> 8) };
	if (!bcast && memcmp(Frame.data(), own, 6) != 0 && (s_E.Filter & 0x80000001U) == 0)
	{
		return false;
	}
	Desc *d = (Desc *)(uintptr_t)RxAddr(s_E.RxCur);
	if (s_E.RxStopped || RxAddr(s_E.RxCur) == s_E.RxTail || (d->W[3] & OWN) == 0)
	{
		s_E.RxStopped = true;
		s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_RBU_MASK | ENET_DMA_CH_DMA_CHX_STAT_AIS_MASK;
		s_M.Lost++;
		return false;
	}
	// Read format: buffer 1 valid, the other words zero, its size from RBSZ
	assert((d->W[3] & BUF1V) && (d->W[3] & 0x3EFFFFFFU) == 0 && d->W[1] == 0 && d->W[2] == 0);
	uint32_t rbsz = (s_E.RxCtrl & ENET_DMA_CH_DMA_CHX_RX_CTRL_RBSZ_MASK) >> 1;
	assert(rbsz == 1536 && Frame.size() <= rbsz);
	memcpy((void *)(uintptr_t)d->W[0], Frame.data(), Frame.size());
	bool ioc = (d->W[3] & IOC) != 0;
	d->W[0] = 0xDEAD;
	d->W[1] = 0xBEEF;
	d->W[2] = 0x1234;
	d->W[3] = FD | LD | Flags | (uint32_t)Frame.size();
	if (ioc)
	{
		s_E.Stat |= ENET_DMA_CH_DMA_CHX_STAT_RI_MASK | ENET_DMA_CH_DMA_CHX_STAT_NIS_MASK;
	}
	s_E.RxCur = (s_E.RxCur + 1) % 4;
	Irq();
	return true;
}

alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];
alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];

// The pin driver uses its SYSCON registers in memory, the Ethernet driver the
// SYSCON model
#undef SYSCON
#undef IOCON
#undef GPIO
#undef PINT
#undef INPUTMUX
#undef ENET
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#undef SYSCON
#define SYSCON (&s_SysRegs)
#define ENET (&s_Enet)
#define LPC546XX_ETH_POLL_US 50
#include "../../../ARM/NXP/LPC546xx/src/ethernet_lpc546xx.cpp"

// ---- Tests ----

// LPCXpresso54628 RMII pins (NXP SDK enet examples)
static const IOPinCfg_t s_Pins[] = {
	{4, 14, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// REF_CLK
	{4, 16, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MDIO
	{4, 15, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// MDC
	{4, 10, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CRS_DV
	{4, 11, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RXD0
	{4, 12, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RXD1
	{4, 13, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX_EN
	{4, 8, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TXD0
	{0, 17, IOPINOP_FUNC7, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TXD1
};

static const uint8_t s_Mac[6] = { 0x02, 0x00, 0x00, 0x54, 0x62, 0x80 };
static const uint8_t s_Bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static int s_Evt[16];

static int EthEvt(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Len)
{
	(void)pDev; (void)pData; (void)Len;
	s_Evt[Evt & 15]++;
	return 0;
}

static int EthEvt2(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Len)
{
	return EthEvt(pDev, Evt, pData, Len);
}

static EthernetCfg_t Cfg(bool bInt = true)
{
	EthernetCfg_t c;

	memset(&c, 0, sizeof(c));
	c.DevNo = 0;
	c.pIOPinMap = s_Pins;
	c.NbIOPins = 9;
	memcpy(c.MacAddr, s_Mac, 6);
	c.PhyAddr = 0;
	c.IntPrio = 6;
	c.bIntEn = bInt;
	c.EvtCB = EthEvt;
	return c;
}

static void Fresh(void)
{
	s_M = Model();
	s_EthClk = s_EthRst = false;
	EnetReset();
	// Values the driver must set again
	s_E.MacCfg = s_E.IntEn = s_E.ChCtrl = 0xFFFFFFFFU;
	s_Phy = Phy();
	s_Phy.Reg[2] = 0x0007;
	s_Phy.Reg[3] = 0xC0F1;
	s_PhySel = 0;
	s_RstPulses = 0;
	s_NvicEn = 0;
	s_NvicPrio = -1;
	s_CoreClk = 220000000;
	memset(s_Evt, 0, sizeof(s_Evt));
	memset(s_IoconMem, 0xFF, sizeof(s_IoconMem));
}

static std::vector<uint8_t> Frame(const uint8_t *pDst, int Len, uint8_t Seed)
{
	std::vector<uint8_t> f(Len);

	for (int i = 0; i < Len; i++)
	{
		f[i] = (uint8_t)(Seed + i * 7);
	}
	memcpy(f.data(), pDst, Len < 6 ? Len : 6);
	if (Len >= 12)
	{
		memcpy(f.data() + 6, s_Mac, 6);
	}
	return f;
}

static uint32_t Iocon(const IOPinCfg_t &p)
{
	return ((IOCON_Type *)s_IoconMem)->PIO[p.PortNo][p.PinNo];
}

static void TestInitChecks()
{
	Ethernet eth;
	EthernetDev_t *d = eth;
	EthernetCfg_t c;

	Fresh();
	assert(!EthernetInit(NULL, &(c = Cfg())) && !EthernetInit(d, NULL));
	c = Cfg(); c.DevNo = 1; assert(!eth.Init(c));
	c = Cfg(); c.DevNo = -1; assert(!eth.Init(c));
	c = Cfg(); c.pIOPinMap = NULL; assert(!eth.Init(c));
	c = Cfg(); c.NbIOPins = 8; assert(!eth.Init(c));
	c = Cfg(); c.NbIOPins = 10; assert(!eth.Init(c));
	c = Cfg(); c.PhyAddr = -1; assert(!eth.Init(c));
	c = Cfg(); c.PhyAddr = 32; assert(!eth.Init(c));
	c = Cfg(); c.IntPrio = -1; assert(!eth.Init(c));
	c = Cfg(); c.IntPrio = 8; assert(!eth.Init(c));
	c = Cfg(); c.MacAddr[0] = 0x01; assert(!eth.Init(c));
	c = Cfg(); c.MacAddr[0] = 0x03; assert(!eth.Init(c));
	c = Cfg(); memset(c.MacAddr, 0, 6); assert(!eth.Init(c));
	s_CoreClk = 19999999; c = Cfg(); assert(!eth.Init(c));
	s_CoreClk = 300000001; assert(!eth.Init(c));
	assert(!s_EthClk && s_RstPulses == 0 && s_Phy.Bmcr8000 == 0 && s_M.MdioOps == 0);

	// No reference clock: the DMA reset does not end, closed
	Fresh();
	s_Phy.RefClk = false;
	uint64_t t0 = s_Us;
	assert(!eth.Init(Cfg()) && !s_EthClk && !s_EthRst && s_RstPulses == 2 && s_M.MdioOps == 0);
	assert(s_Us - t0 >= 50 && s_Us - t0 < 100 && !s_NvicEn);
	for (auto &p : s_Pins)
	{
		assert(Iocon(p) == 0);
	}
	// No PHY at the address, another PHY, MDIO stuck, a PHY reset that does
	// not end: closed, the clock off
	Fresh();
	s_Phy.Addr = 1;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr8000 == 0);
	Fresh();
	s_Phy.Reg[3] = 0x0161;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr8000 == 0);
	Fresh();
	s_Phy.Reg[3] = 0xC101;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr8000 == 0);
	Fresh();
	s_Phy.Reg[2] = 0x0022;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr8000 == 0);
	Fresh();
	s_M.MdioStuck = true;
	t0 = s_Us;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Us - t0 < 200);
	Fresh();
	s_Phy.ResetStuck = true;
	t0 = s_Us;
	assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr8000 == 1 && s_Phy.Bmcr1200 == 0 && s_Us - t0 >= 400000);
	// An MDIO transfer that does not end at each step: ID1, ID2, PHY reset,
	// reset poll, auto negotiation
	Fresh();
	assert(eth.Init(Cfg()));
	int ops = s_M.MdioOps;
	EthernetClose(d);
	for (int k = 0; k < ops; k++)
	{
		Fresh();
		s_M.MdioHangAt = k;
		assert(!eth.Init(Cfg()) && !s_EthClk && s_Phy.Bmcr1200 == 0 && s_Phy.Bmcr8000 == (k >= 3));
	}
	// The LAN8742A, the address 31, priority 0 and 7, a MAC address of one
	// bit, the AHB clock limits
	Fresh();
	s_Phy.Reg[3] = 0xC131;
	assert(eth.Init(Cfg()));
	EthernetClose(d);
	assert(!s_EthClk);
	Fresh();
	s_Phy.Addr = 31;
	c = Cfg(); c.PhyAddr = 31; c.IntPrio = 7;
	assert(eth.Init(c) && s_NvicPrio == 7);
	EthernetClose(d);
	Fresh();
	c = Cfg(); c.IntPrio = 0; memset(c.MacAddr, 0, 6); c.MacAddr[5] = 1;
	assert(eth.Init(c) && s_NvicPrio == 0 && s_E.AddrHi == 0x0100U && s_E.AddrLo == 0);
	EthernetClose(d);
	static const uint32_t clk[] = { 20000000, 300000000 };
	for (uint32_t k : clk)
	{
		Fresh();
		s_CoreClk = k;
		assert(eth.Init(Cfg()) && s_E.Tic == k / 1000000 - 1);
		EthernetClose(d);
	}
	printf("init checks: PASS\n");
}

static void TestInit()
{
	Ethernet eth;
	EthernetDev_t *d = eth;

	Fresh();
	s_Phy.Addr = 5;
	EthernetCfg_t c = Cfg();
	c.PhyAddr = 5;
	memset((void *)d, 0xA5, sizeof(*d));
	d->DevIntrf.bTxReady = false;
	d->DevIntrf.bDma = false;
	d->DevIntrf.bIntEn = false;
	assert(eth.Init(c));
	// RMII sampled by the DMA reset, the ENET clocked and out of reset
	assert(s_E.Rmii && s_EthClk && !s_EthRst && s_RstPulses == 1);
	// Pins in their function
	for (auto &p : s_Pins)
	{
		assert((Iocon(p) & IOCON_PIO_FUNC_MASK) == (uint32_t)(p.PinOp - 1));
	}
	// PHY reset then auto negotiation
	assert(s_Phy.Bmcr8000 == 1 && s_Phy.Bmcr1200 == 1 && s_Phy.Reg[0] == 0x1200);
	// DMA, MTL and MAC
	assert(s_E.ChCtrl == 0 && s_E.TxCtrl == (1U << 16) && s_E.RxCtrl == ((1U << 16) | (1536U << 1)));
	assert(s_E.TxList == (uint32_t)(uintptr_t)s_EthMem.Tx && s_E.RxList == (uint32_t)(uintptr_t)s_EthMem.Rx);
	assert(s_E.TxLen == 3 && s_E.RxLen == 3);
	assert(s_E.TxTail == (uint32_t)(uintptr_t)s_EthMem.Tx && s_E.RxTail == (uint32_t)(uintptr_t)(s_EthMem.Rx + 4));
	assert(s_E.TxQ == (0x2U | (2U << 2) | (7U << 16)) && s_E.RxQ == (0x20U | (7U << 20)));
	assert(s_E.AddrLo == 0x54000002U && s_E.AddrHi == 0x8062U && s_E.Filter == 0);
	assert(s_E.MacCfg == (ENET_MAC_CONFIG_PS_MASK | ENET_MAC_CONFIG_ACS_MASK | ENET_MAC_CONFIG_CST_MASK));
	assert(s_E.RxqCtrl == 1 && s_E.MacIntEn == 0 && s_E.Tic == 219 && s_E.IntEn == 0);
	assert(s_NvicPrio == 6 && s_NvicEn == 0);
	for (int i = 0; i < 4; i++)
	{
		assert(s_EthMem.Rx[i].Word[0] == (uint32_t)(uintptr_t)s_EthMem.RxData[i]);
		assert(s_EthMem.Rx[i].Word[3] == 0xC1000000U && s_EthMem.Tx[i].Word[3] == 0);
	}
	assert(memcmp(d->MacAddr, s_Mac, 6) == 0 && d->DevIntrf.Type == DEVINTRF_TYPE_ETH);
	assert(d->DevIntrf.GetHandle(&d->DevIntrf) == d && d->DevIntrf.pDevData == d);
	assert(d->DevIntrf.EnCnt == 1 && d->DevIntrf.bIntEn && d->DevIntrf.IntPrio == 6 && d->DevIntrf.EvtCB == EthEvt);
	assert(d->DevIntrf.MaxRetry == 0 && d->DevIntrf.bDma && d->DevIntrf.bTxReady && !d->DevIntrf.bNoStop);
	assert(d->DevIntrf.StopRx == EthNoOp && d->DevIntrf.StopTx == EthNoOp && d->DevIntrf.TxSrData == EthTx);
	assert(!atomic_flag_test_and_set(&d->DevIntrf.bBusy));
	atomic_flag_clear(&d->DevIntrf.bBusy);
	// No link yet: no rate, no transmit
	uint8_t f[64];
	memset(f, 0xFF, sizeof(f));
	assert(eth.Rate() == 0 && eth.Rate(100000000) == 0 && eth.Tx(0, f, 64) == 0 && s_M.Wire.empty());
	// Another device is refused while this one is set up
	Ethernet other;
	assert(!other.Init(Cfg()));
	assert(((EthernetDev_t *)other)->DevIntrf.GetHandle == NULL);
	// Init again: closed then set up
	int pulses = s_RstPulses;
	assert(eth.Init(c) && s_RstPulses == pulses + 2);
	EthernetClose(d);
	assert(!s_EthClk && d->DevIntrf.GetHandle(&d->DevIntrf) == NULL);
	for (auto &p : s_Pins)
	{
		assert(Iocon(p) == 0);
	}
	s_Phy.Addr = 0;
	assert(other.Init(Cfg()));
	s_Phy.Link = true;
	other.PollLink();
	assert(other.Rate() == 100000000);
	// Calls on a device not set up, the other one running
	DevIntrf_t *di = &d->DevIntrf;
	assert(di->GetRate(di) == 0 && di->SetRate(di, 10000000) == 0 && !di->StartRx(di, 0) && !di->StartTx(di, 0));
	assert(di->GetHandle(di) == NULL && di->TxData(di, f, 64) == 0 && di->RxData(di, f, 64) == 0);
	pulses = s_RstPulses;
	di->Disable(di);
	assert(other.Rate() == 100000000);
	EthernetClose(d);
	assert(s_EthClk && other.Rate() == 100000000);
	di->Reset(di);
	di->PowerOff(di);
	assert(s_EthClk && s_RstPulses == pulses && other.Rate() == 100000000);
	other.Disable();
	di->Enable(di);
	assert(other.Rate() == 0);
	other.Enable();
	assert(other.Rate() == 100000000 && other.Rate(10000000) == 100000000);
	other.PowerOff();
	assert(!s_EthClk);
	// Nothing set up: no access to the ENET
	s_Enet.MAC_MDIO_ADDR = 0;
	EthernetPollLink(NULL);
	EthernetPollLink(d);
	EthernetClose(NULL);
	assert(s_Enet.MAC_MDIO_ADDR == 0);
	// MDIO clock from the AHB clock
	static const struct { uint32_t Clk, Cr; } cr[] = {
		{ 20000000, 2 }, { 34999999, 2 }, { 35000000, 3 }, { 59999999, 3 }, { 60000000, 0 }, { 99999999, 0 },
		{ 100000000, 1 }, { 149999999, 1 }, { 150000000, 4 }, { 249999999, 4 }, { 250000000, 5 }, { 300000000, 5 },
	};
	for (auto &e : cr)
	{
		assert(EthMdioClock(e.Clk) == (e.Cr << 8));
	}
	printf("init: PASS\n");
}

static void TestLink()
{
	Ethernet eth;

	Fresh();
	assert(eth.Init(Cfg()));
	// Down: nothing changes, no event
	eth.PollLink();
	assert(eth.Rate() == 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 0 && (s_E.MacCfg & 3U) == 0);
	// Up 100 full
	s_Phy.Link = true;
	s_Phy.Mode = 6;
	eth.PollLink();
	assert(eth.Rate() == 100000000 && s_Evt[DEVINTRF_EVT_STATECHG] == 1);
	uint32_t m = ENET_MAC_CONFIG_FES_MASK | ENET_MAC_CONFIG_DM_MASK | ENET_MAC_CONFIG_RE_MASK | ENET_MAC_CONFIG_TE_MASK;
	assert((s_E.MacCfg & m) == m && (s_E.MacCfg & ENET_MAC_CONFIG_PS_MASK));
	assert((s_E.TxCtrl & 1U) && (s_E.RxCtrl & 1U) && s_E.Flushes == 1 && s_NvicEn);
	assert(s_E.IntEn == 0xD041U);
	// The same: no event
	eth.PollLink();
	assert(s_Evt[DEVINTRF_EVT_STATECHG] == 1 && s_E.Flushes == 1);
	// 10 half, 100 half, 10 full
	static const struct { uint16_t Mode; uint32_t Rate; bool Fes, Dm; } md[] = {
		{ 1, 10000000, false, false }, { 2, 100000000, true, false }, { 5, 10000000, false, true },
	};
	int n = 1;
	for (auto &e : md)
	{
		s_Phy.Mode = e.Mode;
		eth.PollLink();
		assert(eth.Rate() == e.Rate && s_Evt[DEVINTRF_EVT_STATECHG] == ++n);
		assert(((s_E.MacCfg & ENET_MAC_CONFIG_FES_MASK) != 0) == e.Fes && ((s_E.MacCfg & ENET_MAC_CONFIG_DM_MASK) != 0) == e.Dm);
		assert((s_E.MacCfg & 3U) == 3U && (s_E.TxCtrl & 1U) && (s_E.RxCtrl & 1U));
		assert((s_E.Stat & (ENET_DMA_CH_DMA_CHX_STAT_TPS_MASK | ENET_DMA_CH_DMA_CHX_STAT_RPS_MASK)) == 0);
	}
	// Speed the same, duplex changed
	s_Phy.Mode = 1;
	eth.PollLink();
	assert(eth.Rate() == 10000000 && (s_E.MacCfg & ENET_MAC_CONFIG_DM_MASK) == 0 && s_Evt[DEVINTRF_EVT_STATECHG] == ++n);
	// Modes the PHY does not give: down
	static const uint16_t bad[] = { 0, 3, 4, 7 };
	for (uint16_t b : bad)
	{
		s_Phy.Mode = 6;
		eth.PollLink();
		assert(eth.Rate() == 100000000);
		n = s_Evt[DEVINTRF_EVT_STATECHG];
		s_Phy.Mode = b;
		eth.PollLink();
		assert(eth.Rate() == 0 && (s_E.MacCfg & (m | 3U)) == 0 && (s_E.TxCtrl & 1U) == 0 && (s_E.RxCtrl & 1U) == 0);
		assert(!s_NvicEn && s_E.IntEn == 0 && s_Evt[DEVINTRF_EVT_STATECHG] == n + 1);
	}
	// Link lost since the last read: the second read gives the state
	s_Phy.Mode = 6;
	s_Phy.LinkLatched = true;
	eth.PollLink();
	assert(eth.Rate() == 100000000);
	// Link up, auto negotiation not complete; link down
	s_Phy.AnDone = false;
	eth.PollLink();
	assert(eth.Rate() == 0);
	s_Phy.AnDone = true;
	eth.PollLink();
	assert(eth.Rate() == 100000000);
	s_Phy.Link = false;
	eth.PollLink();
	assert(eth.Rate() == 0);
	// MDIO transfer that does not end: down, then up when it ends
	s_Phy.Link = true;
	eth.PollLink();
	assert(eth.Rate() == 100000000);
	s_M.MdioStuck = true;
	eth.PollLink();
	assert(eth.Rate() == 0);
	s_M.MdioStuck = false;
	eth.PollLink();
	assert(eth.Rate() == 100000000);
	// The register 31 read does not end: down; the next poll waits for it
	s_M.MdioHangAt = s_M.MdioOps + 2;
	eth.PollLink();
	assert(eth.Rate() == 0 && s_M.MdioPend != 0);
	s_M.MdioHangAt = -1;
	eth.PollLink();
	assert(eth.Rate() == 100000000 && s_M.MdioPend == 0);
	// Disabled: the link is followed, nothing started
	eth.Disable();
	s_Phy.Mode = 2;
	eth.PollLink();
	assert(eth.Rate() == 0 && (s_E.MacCfg & 3U) == 0 && (s_E.MacCfg & ENET_MAC_CONFIG_FES_MASK) && !s_NvicEn);
	eth.Enable();
	assert(eth.Rate() == 100000000 && (s_E.MacCfg & 3U) == 3U && (s_E.MacCfg & ENET_MAC_CONFIG_DM_MASK) == 0);
	// No event callback
	EthernetDev_t *d = eth;
	d->DevIntrf.EvtCB = NULL;
	s_Phy.Mode = 6;
	n = s_Evt[DEVINTRF_EVT_STATECHG];
	eth.PollLink();
	assert(eth.Rate() == 100000000 && s_Evt[DEVINTRF_EVT_STATECHG] == n);
	// Without interrupts the DMA interrupts stay off
	EthernetClose(eth);
	Fresh();
	assert(eth.Init(Cfg(false)) && eth.Rate() == 0);
	s_Phy.Link = true;
	eth.PollLink();
	assert(eth.Rate() == 100000000 && s_E.IntEn == 0 && !s_NvicEn && s_EthMem.Rx[0].Word[3] == 0x81000000U);
	// An object not set up, a NULL device
	int ops = s_M.MdioOps;
	EthernetPollLink(NULL);
	Ethernet none;
	none.PollLink();
	assert(s_M.MdioOps == ops);
	EthernetClose(eth);
	printf("link: PASS\n");
}

static void TestTxRx()
{
	for (int intr = 0; intr < 2; intr++)
	{
		Ethernet eth;
		EthernetDev_t *d = eth;
		DevIntrf_t *di = &d->DevIntrf;

		Fresh();
		assert(eth.Init(Cfg(intr != 0)));
		s_Phy.Link = true;
		eth.PollLink();

		// Transmit: lengths 14 to 1514, refused out of them
		uint8_t buf[1600];
		memset(buf, 0x55, sizeof(buf));
		static const int lens[] = { 14, 60, 61, 1000, 1514 };
		for (int len : lens)
		{
			std::vector<uint8_t> f = Frame(s_Bcast, len, (uint8_t)len);
			assert(eth.Tx(0, f.data(), len) == len);
			assert(s_M.Wire.back() == f);
		}
		assert(s_M.Wire.size() == 5);
		assert(eth.Tx(0, buf, 13) == 0 && eth.Tx(0, buf, 1515) == 0 && di->TxData(di, NULL, 60) == 0);
		assert(s_Evt[DEVINTRF_EVT_TX_READY] == (intr ? 5 : 0));
		// Buffer length and interrupt on completion
		static const uint32_t last[] = { 1514, 60, 61, 1000 };
		for (int i = 0; i < 4; i++)
		{
			assert(s_EthMem.Tx[i].Word[2] == ((intr ? 0x80000000U : 0U) | last[i]));
		}

		// The DMA holds the descriptor it fetched: 4 frames queued, the 5th
		// refused, then all sent in order
		s_M.TxHold = true;
		for (int i = 0; i < 4; i++)
		{
			std::vector<uint8_t> f = Frame(s_Mac, 100, (uint8_t)i);
			assert(eth.Tx(0, f.data(), 100) == 100);
		}
		assert(eth.Tx(0, buf, 100) == 0 && s_M.Wire.size() == 5);
		s_M.TxHold = false;
		DmaRun();
		assert(s_M.Wire.size() == 9);
		for (int i = 0; i < 4; i++)
		{
			assert(s_M.Wire[5 + i] == Frame(s_Mac, 100, (uint8_t)i));
		}
		assert(eth.Tx(0, buf, 100) == 100 && s_M.Wire.size() == 10);
		assert(di->TxSrData(di, buf, 99) == 99 && s_M.Wire.size() == 11 && s_M.Wire.back().size() == 99);
		// Many frames, the tail one past the last descriptor on each lap
		for (int i = 0; i < 20; i++)
		{
			std::vector<uint8_t> f = Frame(s_Bcast, 60 + i, (uint8_t)i);
			assert(eth.Tx(0, f.data(), 60 + i) == 60 + i && s_M.Wire.back() == f);
		}

		// Receive: own address and broadcast in order, another address not
		// taken
		std::vector<uint8_t> a = Frame(s_Mac, 60, 1), b = Frame(s_Bcast, 1514, 2);
		std::vector<uint8_t> o = Frame((const uint8_t *)"\x02\x00\x00\x54\x62\x81", 60, 3);
		assert(Receive(a) && Receive(b) && !Receive(o));
		assert(eth.Rx(0, buf, sizeof(buf)) == 60 && memcmp(buf, a.data(), 60) == 0);
		assert(eth.Rx(0, buf, sizeof(buf)) == 1514 && memcmp(buf, b.data(), 1514) == 0);
		int tails = s_M.RxTails;
		assert(eth.Rx(0, buf, sizeof(buf)) == 0 && s_M.RxTails == tails);
		assert(s_Evt[DEVINTRF_EVT_RX_DATA] == (intr ? 2 : 0));
		// The 4 descriptors filled: the 5th frame lost, the DMA stopped until
		// a descriptor is given back
		for (int i = 0; i < 5; i++)
		{
			Receive(Frame(s_Mac, 80 + i, (uint8_t)(10 + i)));
		}
		assert(s_M.Lost == 1 && s_E.RxStopped);
		for (int i = 0; i < 4; i++)
		{
			assert(eth.Rx(0, buf, sizeof(buf)) == 80 + i && memcmp(buf, Frame(s_Mac, 80 + i, (uint8_t)(10 + i)).data(), 80 + i) == 0);
			assert(!s_E.RxStopped);
		}
		assert(eth.Rx(0, buf, sizeof(buf)) == 0);
		// Many frames through the ring
		for (int i = 0; i < 50; i++)
		{
			std::vector<uint8_t> f = Frame(s_Mac, 14 + i * 29, (uint8_t)i);
			assert(Receive(f));
			int n = eth.Rx(0, buf, sizeof(buf));
			assert(n == (int)f.size() && memcmp(buf, f.data(), n) == 0);
		}
		// A frame in error, a context descriptor, a frame not whole (no FD or
		// no LD): dropped, the next one returned
		static const uint32_t errs[] = { 0x8000U, 0x40000000U };
		for (uint32_t e : errs)
		{
			assert(Receive(Frame(s_Mac, 100, 20), e));
			assert(Receive(Frame(s_Mac, 100, 21)));
			assert(eth.Rx(0, buf, sizeof(buf)) == 100 && memcmp(buf, Frame(s_Mac, 100, 21).data(), 100) == 0);
		}
		static const uint32_t part[] = { FD, LD };
		for (uint32_t p : part)
		{
			Desc *dd = (Desc *)(uintptr_t)RxAddr(s_E.RxCur);
			assert(Receive(Frame(s_Mac, 100, 22)));
			dd->W[3] &= ~p;
			assert(Receive(Frame(s_Mac, 100, 23)));
			assert(eth.Rx(0, buf, sizeof(buf)) == 100 && memcmp(buf, Frame(s_Mac, 100, 23).data(), 100) == 0);
		}
		// Lengths out of 14 to 1514, longer than the buffer of the call:
		// dropped
		assert(Receive(Frame(s_Mac, 13, 24)) && eth.Rx(0, buf, sizeof(buf)) == 0);
		assert(Receive(Frame(s_Mac, 1515, 25)) && eth.Rx(0, buf, sizeof(buf)) == 0);
		assert(Receive(Frame(s_Mac, 200, 26)) && eth.Rx(0, buf, 199) == 0);
		assert(Receive(Frame(s_Mac, 200, 27)) && eth.Rx(0, buf, 200) == 200 && buf[199] == Frame(s_Mac, 200, 27)[199]);
		// No buffer or not the owner: the frame stays
		assert(Receive(Frame(s_Mac, 60, 28)));
		Ethernet none;
		DevIntrf_t *ni = none;
		assert(di->RxData(di, NULL, 100) == 0 && di->RxData(ni, buf, sizeof(buf)) == 0);
		assert(di->TxData(ni, buf, 60) == 0);
		assert(eth.Rx(0, buf, sizeof(buf)) == 60 && buf[20] == Frame(s_Mac, 60, 28)[20]);
		// The descriptors given back: buffer address and owner
		for (int i = 0; i < 4; i++)
		{
			assert(s_EthMem.Rx[i].Word[0] == (uint32_t)(uintptr_t)s_EthMem.RxData[i]);
			assert(s_EthMem.Rx[i].Word[1] == 0 && s_EthMem.Rx[i].Word[2] == 0);
			assert(s_EthMem.Rx[i].Word[3] == (intr ? 0xC1000000U : 0x81000000U));
		}

		// Disable: stopped, nothing sent or received; Enable: started again
		eth.Disable();
		assert((s_E.MacCfg & 3U) == 0 && !s_NvicEn && eth.Rate() == 0 && eth.Tx(0, buf, 60) == 0);
		assert(!eth.StartRx(0) && !eth.StartTx(0) && !Receive(a) && eth.Rx(0, buf, sizeof(buf)) == 0);
		eth.Enable();
		assert((s_E.MacCfg & 3U) == 3U && eth.Rate() == 100000000 && eth.Tx(0, a.data(), 60) == 60);
		assert(s_M.Wire.back() == a);
		assert(eth.StartRx(0));
		eth.StopRx();
		assert(eth.StartTx(0));
		eth.StopTx();
		assert(Receive(b) && eth.Rx(0, buf, sizeof(buf)) == 1514);

		if (intr)
		{
			// Fatal bus error: stopped, event, Reset sets up again
			size_t sent = s_M.Wire.size();
			int ops = s_M.MdioOps;
			s_M.BusError = true;
			assert(eth.Tx(0, a.data(), 60) == 60);
			assert(eth.Rate() == 0 && s_Evt[DEVINTRF_EVT_STATECHG] == 2 && s_E.IntEn == 0);
			assert(eth.Tx(0, a.data(), 60) == 0 && !eth.StartRx(0) && s_M.Wire.size() == sent);
			eth.PollLink();
			assert(eth.Rate() == 0 && s_M.MdioOps == ops && s_Evt[DEVINTRF_EVT_STATECHG] == 2);
			eth.Disable();
			eth.Enable();
			assert(eth.Rate() == 0 && s_E.IntEn == 0);
			di->EvtCB = EthEvt2;
			eth.Reset();
			assert(eth.Rate() == 0 && s_EthClk && di->EvtCB == EthEvt2);
			di->EvtCB = EthEvt;
			eth.PollLink();
			assert(eth.Rate() == 100000000 && eth.Tx(0, a.data(), 60) == 60 && s_M.Wire.size() == sent + 1);
			// Interrupt with nothing to report
			int irqs = s_M.Irqs, ev = s_Evt[DEVINTRF_EVT_RX_DATA] + s_Evt[DEVINTRF_EVT_TX_READY];
			s_E.Stat = ENET_DMA_CH_DMA_CHX_STAT_TBU_MASK;
			ETHERNET_IRQHandler();
			assert(s_E.Stat == 0 && s_M.Irqs == irqs && s_Evt[DEVINTRF_EVT_RX_DATA] + s_Evt[DEVINTRF_EVT_TX_READY] == ev);
			// No event callback
			di->EvtCB = NULL;
			assert(eth.Tx(0, a.data(), 60) == 60 && Receive(a) && eth.Rx(0, buf, sizeof(buf)) == 60);
			assert(s_Evt[DEVINTRF_EVT_RX_DATA] + s_Evt[DEVINTRF_EVT_TX_READY] == ev);
			di->EvtCB = EthEvt;
		}
		// Close: stopped, reset, clock off, pins disabled
		int pulses = s_RstPulses;
		EthernetClose(d);
		assert(!s_EthClk && s_RstPulses == pulses + 1 && !s_NvicEn && eth.Rate() == 0);
		assert(eth.Tx(0, a.data(), 60) == 0);
		for (auto &p : s_Pins)
		{
			assert(Iocon(p) == 0);
		}
		EthernetClose(d);
		assert(s_RstPulses == pulses + 1);
		// An interrupt after close: cleared, no event
		int ev = s_Evt[DEVINTRF_EVT_RX_DATA];
		s_E.Stat = ENET_DMA_CH_DMA_CHX_STAT_RI_MASK;
		ETHERNET_IRQHandler();
		assert(s_E.Stat == 0 && s_Evt[DEVINTRF_EVT_RX_DATA] == ev);
	}
	printf("transmit and receive: PASS\n");
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	TestInitChecks();
	TestInit();
	TestLink();
	TestTxRx();
	printf("ethernet: PASS\n");

	return 0;
}
