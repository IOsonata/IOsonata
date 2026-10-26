/**-------------------------------------------------------------------------
@file	ra4m1_host_regs.h

@brief	Host test register access for the RA4M1 drivers. C++ only.

		The drivers access the Renesas peripheral structures directly
		(R_SYSTEM->SCKSCR, R_ICU->IELSR[n], ...). This header replaces the
		peripheral pointers used by system_ra4m1.c, interrupt_ra4m1.cpp,
		iopincfg_ra4m1.c, iopinctrl.h and timer_ra4m1.cpp with register
		objects. Each read or write of such an object calls the model
		through RA4M1_RD8/16/32 and RA4M1_WR8/16/32 with the register
		address, so the address based models check every access.

		Register addresses are the R7FA4M1AB.h base plus offsetof() of the
		vendor member. The RA4M1_xxx address list below is written from
		R01UH0887EJ0110 and checked against the vendor header at compile
		time.

		Taking the address of a register (&R_SYSTEM->OSCSF, &PmnPFS) gives a
		pointer to the model storage through HostRegPtr(), which the model
		defines when the driver needs it. Accesses through that pointer do
		not call the model.

		Usage, in this order :
			declare the model rd8, rd16, rd32, wr8, wr16, wr32 functions and
			define RA4M1_RD8 ... RA4M1_WR32 to them
			#include "ra4m1xxx.h"
			#include "ra4m1_host_regs.h"
			#include the driver sources

		Not a production header.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#ifndef RA4M1_HOST_REGS_H
#define RA4M1_HOST_REGS_H

#ifndef __cplusplus
#error "ra4m1_host_regs.h needs C++"
#endif

#include <stddef.h>
#include <stdint.h>

#ifndef __RA4M1XXX_H__
#error "Include ra4m1xxx.h first"
#endif
#ifdef __IOPINCTRL_H__
#error "Include ra4m1_host_regs.h before iopinctrl.h"
#endif

// Register addresses, R01UH0887EJ0110 chapters 8, 10, 11, 13, 19, 22, 23, 44, 45

// System, clock generation and low power modes
#define RA4M1_SCKDIVCR		0x4001E020UL
#define RA4M1_SCKSCR		0x4001E026UL
#define RA4M1_PLLCR			0x4001E02AUL
#define RA4M1_PLLCCR2		0x4001E02BUL	// Byte wide
#define RA4M1_MEMWAIT		0x4001E031UL
#define RA4M1_MOSCCR		0x4001E032UL
#define RA4M1_HOCOCR		0x4001E036UL
#define RA4M1_HOCOCR2		0x4001E037UL
#define RA4M1_MOCOCR		0x4001E038UL
#define RA4M1_OSCSF			0x4001E03CUL
#define RA4M1_OSTDCR		0x4001E040UL
#define RA4M1_OSTDSR		0x4001E041UL
#define RA4M1_HOCOUTCR		0x4001E062UL
#define RA4M1_USBCKCR		0x4001E0D0UL
#define RA4M1_OPCCR			0x4001E0A0UL
#define RA4M1_MOSCWTCR		0x4001E0A2UL
#define RA4M1_HOCOWTCR		0x4001E0A5UL
#define RA4M1_SOPCCR		0x4001E0AAUL
#define RA4M1_PRCR			0x4001E3FEUL
#define RA4M1_MOMCR			0x4001E413UL
#define RA4M1_VBTCR1		0x4001E41FUL
#define RA4M1_SOSCCR		0x4001E480UL
#define RA4M1_SOMCR			0x4001E481UL
#define RA4M1_LOCOCR		0x4001E490UL
#define RA4M1_VBTSR			0x4001E4B1UL
#define RA4M1_VBTICTLR		0x4001E4BBUL
#define RA4M1_VBTOCTLR		0x4001E4BCUL

// Flash cache, USBFS
#define RA4M1_FCACHEE		0x4001C100UL
#define RA4M1_FCACHEIV		0x4001C104UL
#define RA4M1_USB_SYSCFG	0x40090000UL

// ICU
#define RA4M1_IRQCR(n)		(0x40006000UL + (n))
#define RA4M1_WUPEN			0x400061A0UL
#define RA4M1_DELSR(n)		(0x40006280UL + 4UL * (n))
#define RA4M1_IELSR(n)		(0x40006300UL + 4UL * (n))

// I/O ports
#define RA4M1_PCNTR1(p)		(0x40040000UL + 0x20UL * (p))
#define RA4M1_PCNTR2(p)		(0x40040004UL + 0x20UL * (p))
#define RA4M1_PCNTR3(p)		(0x40040008UL + 0x20UL * (p))
#define RA4M1_PCNTR4(p)		(0x4004000CUL + 0x20UL * (p))
#define RA4M1_PFS(p, n)		(0x40040800UL + 0x40UL * (p) + 4UL * (n))
#define RA4M1_PWPR			0x40040D03UL

// Module stop, AGT, GPT
#define RA4M1_TIMER_MSTPCRD	0x40047008UL
#define RA4M1_AGT_BASE(n)	(0x40084000UL + 0x100UL * (n))
#define RA4M1_GPT_BASE(n)	(0x40078000UL + 0x100UL * (n))

// AGT register offsets. Data registers are halfwords, control registers bytes.
#define RA4M1_AGT			0x00U
#define RA4M1_AGTCMA		0x02U
#define RA4M1_AGTCMB		0x04U
#define RA4M1_AGTCR			0x08U
#define RA4M1_AGTMR1		0x09U
#define RA4M1_AGTMR2		0x0AU
#define RA4M1_AGTIOC		0x0CU
#define RA4M1_AGTISR		0x0DU
#define RA4M1_AGTCMSR		0x0EU
#define RA4M1_AGTIOSEL		0x0FU

// GPT register offsets, all word access
#define RA4M1_GTWP			0x00U
#define RA4M1_GTSSR			0x10U
#define RA4M1_GTPSR			0x14U
#define RA4M1_GTCSR			0x18U
#define RA4M1_GTUPSR		0x1CU
#define RA4M1_GTDNSR		0x20U
#define RA4M1_GTICASR		0x24U
#define RA4M1_GTICBSR		0x28U
#define RA4M1_GTCR			0x2CU
#define RA4M1_GTUDDTYC		0x30U
#define RA4M1_GTIOR			0x34U
#define RA4M1_GTINTAD		0x38U
#define RA4M1_GTST			0x3CU
#define RA4M1_GTBER			0x40U
#define RA4M1_GTCNT			0x48U
#define RA4M1_GTCCRA		0x4CU	// A, B, C, D, E, F every 4 bytes
#define RA4M1_GTPR			0x64U
#define RA4M1_GTPBR			0x68U
#define RA4M1_GTDTCR		0x88U
#define RA4M1_GTDVU			0x8CU

// The list above against the vendor header
#define HOST_REG_VENDOR(Periph, Member)	\
	((uintptr_t)Periph##_BASE + offsetof(Periph##_Type, Member))

static_assert(HOST_REG_VENDOR(R_SYSTEM, SCKDIVCR) == RA4M1_SCKDIVCR, "SCKDIVCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, SCKSCR) == RA4M1_SCKSCR, "SCKSCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, PLLCR) == RA4M1_PLLCR, "PLLCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, PLLCCR2) == RA4M1_PLLCCR2, "PLLCCR2");
static_assert(HOST_REG_VENDOR(R_SYSTEM, MEMWAIT) == RA4M1_MEMWAIT, "MEMWAIT");
static_assert(HOST_REG_VENDOR(R_SYSTEM, MOSCCR) == RA4M1_MOSCCR, "MOSCCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, HOCOCR) == RA4M1_HOCOCR, "HOCOCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, HOCOCR2) == RA4M1_HOCOCR2, "HOCOCR2");
static_assert(HOST_REG_VENDOR(R_SYSTEM, MOCOCR) == RA4M1_MOCOCR, "MOCOCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, OSCSF) == RA4M1_OSCSF, "OSCSF");
static_assert(HOST_REG_VENDOR(R_SYSTEM, OSTDCR) == RA4M1_OSTDCR, "OSTDCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, OSTDSR) == RA4M1_OSTDSR, "OSTDSR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, HOCOUTCR) == RA4M1_HOCOUTCR, "HOCOUTCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, USBCKCR_ALT) == RA4M1_USBCKCR, "USBCKCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, OPCCR) == RA4M1_OPCCR, "OPCCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, MOSCWTCR) == RA4M1_MOSCWTCR, "MOSCWTCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, HOCOWTCR) == RA4M1_HOCOWTCR, "HOCOWTCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, SOPCCR) == RA4M1_SOPCCR, "SOPCCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, PRCR) == RA4M1_PRCR, "PRCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, MOMCR) == RA4M1_MOMCR, "MOMCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, VBTCR1) == RA4M1_VBTCR1, "VBTCR1");
static_assert(HOST_REG_VENDOR(R_SYSTEM, SOSCCR) == RA4M1_SOSCCR, "SOSCCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, SOMCR) == RA4M1_SOMCR, "SOMCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, LOCOCR) == RA4M1_LOCOCR, "LOCOCR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, VBTSR) == RA4M1_VBTSR, "VBTSR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, VBTICTLR) == RA4M1_VBTICTLR, "VBTICTLR");
static_assert(HOST_REG_VENDOR(R_SYSTEM, VBTOCTLR) == RA4M1_VBTOCTLR, "VBTOCTLR");
static_assert(HOST_REG_VENDOR(R_FCACHE, FCACHEE) == RA4M1_FCACHEE, "FCACHEE");
static_assert(HOST_REG_VENDOR(R_FCACHE, FCACHEIV) == RA4M1_FCACHEIV, "FCACHEIV");
static_assert(HOST_REG_VENDOR(R_USB_FS0, SYSCFG) == RA4M1_USB_SYSCFG, "SYSCFG");
static_assert(HOST_REG_VENDOR(R_ICU, IRQCR) == RA4M1_IRQCR(0), "IRQCR");
static_assert(HOST_REG_VENDOR(R_ICU, IRQCR[15]) == RA4M1_IRQCR(15), "IRQCR15");
static_assert(HOST_REG_VENDOR(R_ICU, WUPEN) == RA4M1_WUPEN, "WUPEN");
static_assert(HOST_REG_VENDOR(R_ICU, DELSR) == RA4M1_DELSR(0), "DELSR");
static_assert(HOST_REG_VENDOR(R_ICU, DELSR[3]) == RA4M1_DELSR(3), "DELSR3");
static_assert(HOST_REG_VENDOR(R_ICU, IELSR) == RA4M1_IELSR(0), "IELSR");
static_assert(HOST_REG_VENDOR(R_ICU, IELSR[31]) == RA4M1_IELSR(31), "IELSR31");
static_assert(HOST_REG_VENDOR(R_PORT0, PCNTR1) == RA4M1_PCNTR1(0), "PCNTR1");
static_assert(HOST_REG_VENDOR(R_PORT0, PCNTR2) == RA4M1_PCNTR2(0), "PCNTR2");
static_assert(HOST_REG_VENDOR(R_PORT0, PCNTR3) == RA4M1_PCNTR3(0), "PCNTR3");
static_assert(HOST_REG_VENDOR(R_PORT0, PCNTR4) == RA4M1_PCNTR4(0), "PCNTR4");
static_assert(R_PORT9_BASE == RA4M1_PCNTR1(9), "PORT9");
static_assert(HOST_REG_VENDOR(R_PFS, PORT[0].PIN[0].PmnPFS) == RA4M1_PFS(0, 0), "PFS");
static_assert(HOST_REG_VENDOR(R_PFS, PORT[9].PIN[15].PmnPFS) == RA4M1_PFS(9, 15), "PFS915");
static_assert(HOST_REG_VENDOR(R_PMISC, PWPR) == RA4M1_PWPR, "PWPR");
static_assert(HOST_REG_VENDOR(R_MSTP, MSTPCRD) == RA4M1_TIMER_MSTPCRD, "MSTPCRD");
static_assert(R_AGTX0_BASE == RA4M1_AGT_BASE(0) && R_AGTX1_BASE == RA4M1_AGT_BASE(1), "AGT");
static_assert(R_GPT0_BASE == RA4M1_GPT_BASE(0) && R_GPT7_BASE == RA4M1_GPT_BASE(7), "GPT");
static_assert(offsetof(R_AGTX0_Type, AGT16.AGT) == RA4M1_AGT, "AGT");
static_assert(offsetof(R_AGTX0_Type, AGT16.AGTCMA) == RA4M1_AGTCMA, "AGTCMA");
static_assert(offsetof(R_AGTX0_Type, AGT16.AGTCMB) == RA4M1_AGTCMB, "AGTCMB");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCR) == RA4M1_AGTCR, "AGTCR");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR1) == RA4M1_AGTMR1, "AGTMR1");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR2) == RA4M1_AGTMR2, "AGTMR2");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOC) == RA4M1_AGTIOC, "AGTIOC");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTISR) == RA4M1_AGTISR, "AGTISR");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCMSR) == RA4M1_AGTCMSR, "AGTCMSR");
static_assert(offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOSEL) == RA4M1_AGTIOSEL, "AGTIOSEL");
static_assert(offsetof(R_GPT0_Type, GTWP) == RA4M1_GTWP, "GTWP");
static_assert(offsetof(R_GPT0_Type, GTSSR) == RA4M1_GTSSR, "GTSSR");
static_assert(offsetof(R_GPT0_Type, GTPSR) == RA4M1_GTPSR, "GTPSR");
static_assert(offsetof(R_GPT0_Type, GTCSR) == RA4M1_GTCSR, "GTCSR");
static_assert(offsetof(R_GPT0_Type, GTUPSR) == RA4M1_GTUPSR, "GTUPSR");
static_assert(offsetof(R_GPT0_Type, GTDNSR) == RA4M1_GTDNSR, "GTDNSR");
static_assert(offsetof(R_GPT0_Type, GTICASR) == RA4M1_GTICASR, "GTICASR");
static_assert(offsetof(R_GPT0_Type, GTICBSR) == RA4M1_GTICBSR, "GTICBSR");
static_assert(offsetof(R_GPT0_Type, GTCR) == RA4M1_GTCR, "GTCR");
static_assert(offsetof(R_GPT0_Type, GTUDDTYC) == RA4M1_GTUDDTYC, "GTUDDTYC");
static_assert(offsetof(R_GPT0_Type, GTIOR) == RA4M1_GTIOR, "GTIOR");
static_assert(offsetof(R_GPT0_Type, GTINTAD) == RA4M1_GTINTAD, "GTINTAD");
static_assert(offsetof(R_GPT0_Type, GTST) == RA4M1_GTST, "GTST");
static_assert(offsetof(R_GPT0_Type, GTBER) == RA4M1_GTBER, "GTBER");
static_assert(offsetof(R_GPT0_Type, GTCNT) == RA4M1_GTCNT, "GTCNT");
static_assert(offsetof(R_GPT0_Type, GTCCR) == RA4M1_GTCCRA, "GTCCR");
static_assert(offsetof(R_GPT0_Type, GTPR) == RA4M1_GTPR, "GTPR");
static_assert(offsetof(R_GPT0_Type, GTPBR) == RA4M1_GTPBR, "GTPBR");
static_assert(offsetof(R_GPT0_Type, GTDTCR) == RA4M1_GTDTCR, "GTDTCR");
static_assert(offsetof(R_GPT0_Type, GTDVU) == RA4M1_GTDVU, "GTDVU");

// Register objects live in this array, at the offset of their peripheral
// address from HOST_REG_FIRST. Only their address is used, never the content.
#define HOST_REG_FIRST		0x40000000UL
#define HOST_REG_SIZE		0x000A0000UL

static unsigned char s_HostRegSpace[HOST_REG_SIZE];

static inline uintptr_t HostRegAddr(const void *p)
{
	return HOST_REG_FIRST + (uintptr_t)((const unsigned char *)p - s_HostRegSpace);
}

#define HOST_REG_OBJ(Type, Base)	((Type *)(s_HostRegSpace + ((uintptr_t)(Base) - HOST_REG_FIRST)))

static inline uint8_t HostRegRead(uintptr_t Addr, uint8_t *) { return RA4M1_RD8(Addr); }
static inline uint16_t HostRegRead(uintptr_t Addr, uint16_t *) { return RA4M1_RD16(Addr); }
static inline uint32_t HostRegRead(uintptr_t Addr, uint32_t *) { return RA4M1_RD32(Addr); }
static inline void HostRegWrite(uintptr_t Addr, uint8_t Val) { RA4M1_WR8(Addr, Val); }
static inline void HostRegWrite(uintptr_t Addr, uint16_t Val) { RA4M1_WR16(Addr, Val); }
static inline void HostRegWrite(uintptr_t Addr, uint32_t Val) { RA4M1_WR32(Addr, Val); }

// Model storage of a register, defined by the model when the driver takes
// the address of that register
volatile uint8_t *HostRegPtr(uintptr_t Addr, uint8_t *);
volatile uint16_t *HostRegPtr(uintptr_t Addr, uint16_t *);
volatile uint32_t *HostRegPtr(uintptr_t Addr, uint32_t *);

// Register at a computed address, array element
template <typename T> class HostRegAt {
public:
	explicit HostRegAt(uintptr_t Addr) : vAddr(Addr) {}
	HostRegAt(const HostRegAt &Reg) = default;
	operator T() const { return HostRegRead(vAddr, (T *)0); }
	const HostRegAt &operator=(uint32_t Val) const { HostRegWrite(vAddr, (T)Val); return *this; }
	const HostRegAt &operator=(const HostRegAt &Reg) const { return *this = (uint32_t)(T)Reg; }
	const HostRegAt &operator|=(uint32_t Val) const { return *this = (uint32_t)(T)*this | Val; }
	const HostRegAt &operator&=(uint32_t Val) const { return *this = (uint32_t)(T)*this & Val; }
	volatile T *operator&() const { return HostRegPtr(vAddr, (T *)0); }
private:
	uintptr_t vAddr;
};

// Register member at offset Off of its peripheral
template <typename T, uintptr_t Off> struct HostReg {
	HostRegAt<T> At() const { return HostRegAt<T>(HostRegAddr(this) + Off); }
	operator T() const { return At(); }
	HostReg &operator=(uint32_t Val) { At() = Val; return *this; }
	HostReg &operator=(const HostReg &Reg) { At() = (uint32_t)(T)Reg; return *this; }
	HostReg &operator|=(uint32_t Val) { At() |= Val; return *this; }
	HostReg &operator&=(uint32_t Val) { At() &= Val; return *this; }
	volatile T *operator&() const { return &At(); }
};

// Register array member at offset Off of its peripheral
template <typename T, uintptr_t Off, uintptr_t Stride> struct HostRegArray {
	HostRegAt<T> operator[](unsigned Idx) const { return HostRegAt<T>(HostRegAddr(this) + Off + Stride * Idx); }
};

// Peripheral objects. All members are at offset 0 of their union, the
// register offset is in the member type.
#define HOST_REG(T, Periph, Member)		HostReg<T, offsetof(Periph, Member)> Member
#define HOST_REG_ARRAY(T, Periph, Member) \
	HostRegArray<T, offsetof(Periph, Member), sizeof(((Periph *)0)->Member[0])> Member

union HostSystem {
	HOST_REG(uint32_t, R_SYSTEM_Type, SCKDIVCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, SCKSCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, PLLCCR2);
	HOST_REG(uint8_t, R_SYSTEM_Type, PLLCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, MEMWAIT);
	HOST_REG(uint8_t, R_SYSTEM_Type, MOSCCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, SOSCCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, LOCOCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, HOCOCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, HOCOCR2);
	HOST_REG(uint8_t, R_SYSTEM_Type, MOCOCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, OSCSF);
	HOST_REG(uint8_t, R_SYSTEM_Type, OSTDCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, OSTDSR);
	HOST_REG(uint8_t, R_SYSTEM_Type, HOCOUTCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, OPCCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, MOSCWTCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, HOCOWTCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, SOPCCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, USBCKCR_ALT);
	HOST_REG(uint16_t, R_SYSTEM_Type, PRCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, MOMCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, SOMCR);
	HOST_REG(uint8_t, R_SYSTEM_Type, VBTCR1);
	HOST_REG(uint8_t, R_SYSTEM_Type, VBTSR);
	HOST_REG(uint8_t, R_SYSTEM_Type, VBTICTLR);
	HOST_REG(uint8_t, R_SYSTEM_Type, VBTOCTLR);
};

union HostFcache {
	HOST_REG(uint16_t, R_FCACHE_Type, FCACHEE);
	HOST_REG(uint16_t, R_FCACHE_Type, FCACHEIV);
};

union HostUsbFs {
	HOST_REG(uint16_t, R_USB_FS0_Type, SYSCFG);
};

union HostIcu {
	HOST_REG_ARRAY(uint8_t, R_ICU_Type, IRQCR);
	HOST_REG(uint32_t, R_ICU_Type, WUPEN);
	HOST_REG_ARRAY(uint32_t, R_ICU_Type, DELSR);
	HOST_REG_ARRAY(uint32_t, R_ICU_Type, IELSR);
};

union HostPort {
	HOST_REG(uint32_t, R_PORT0_Type, PCNTR1);
	HOST_REG(uint32_t, R_PORT0_Type, PCNTR2);
	HOST_REG(uint32_t, R_PORT0_Type, PCNTR3);
	HOST_REG(uint32_t, R_PORT0_Type, PCNTR4);
};

// R_PFS->PORT[p].PIN[n].PmnPFS
#define HOST_PFS_PORT_STRIDE	(offsetof(R_PFS_Type, PORT[1]) - offsetof(R_PFS_Type, PORT[0]))
#define HOST_PFS_PIN_STRIDE		(offsetof(R_PFS_Type, PORT[0].PIN[1]) - offsetof(R_PFS_Type, PORT[0].PIN[0]))

union HostPfsPin {
	HostReg<uint32_t, 0> PmnPFS;
};

struct HostPfsPinArray {
	HostPfsPin &operator[](unsigned Idx) { return *(HostPfsPin *)((unsigned char *)this + HOST_PFS_PIN_STRIDE * Idx); }
};

union HostPfsPort {
	HostPfsPinArray PIN;
};

struct HostPfsPortArray {
	HostPfsPort &operator[](unsigned Idx) { return *(HostPfsPort *)((unsigned char *)this + HOST_PFS_PORT_STRIDE * Idx); }
};

union HostPfs {
	HostPfsPortArray PORT;
};

static_assert(offsetof(R_PFS_Type, PORT[0].PIN[0].PmnPFS) == 0, "PFS layout");

union HostPmisc {
	HOST_REG(uint8_t, R_PMISC_Type, PWPR);
};

union HostMstp {
	HOST_REG(uint32_t, R_MSTP_Type, MSTPCRD);
};

union HostAgtCtrl {
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCR)> AGTCR;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR1)> AGTMR1;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR2)> AGTMR2;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOC)> AGTIOC;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTISR)> AGTISR;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCMSR)> AGTCMSR;
	HostReg<uint8_t, offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOSEL)> AGTIOSEL;
};

union HostAgt16 {
	HostReg<uint16_t, offsetof(R_AGTX0_Type, AGT16.AGT)> AGT;
	HostReg<uint16_t, offsetof(R_AGTX0_Type, AGT16.AGTCMA)> AGTCMA;
	HostReg<uint16_t, offsetof(R_AGTX0_Type, AGT16.AGTCMB)> AGTCMB;
	HostAgtCtrl CTRL;
};

union HostAgt {
	HostAgt16 AGT16;
};

union HostGpt {
	HOST_REG(uint32_t, R_GPT0_Type, GTWP);
	HOST_REG(uint32_t, R_GPT0_Type, GTSSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTPSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTCSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTUPSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTDNSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTICASR);
	HOST_REG(uint32_t, R_GPT0_Type, GTICBSR);
	HOST_REG(uint32_t, R_GPT0_Type, GTCR);
	HOST_REG(uint32_t, R_GPT0_Type, GTUDDTYC);
	HOST_REG(uint32_t, R_GPT0_Type, GTIOR);
	HOST_REG(uint32_t, R_GPT0_Type, GTINTAD);
	HOST_REG(uint32_t, R_GPT0_Type, GTST);
	HOST_REG(uint32_t, R_GPT0_Type, GTBER);
	HOST_REG(uint32_t, R_GPT0_Type, GTCNT);
	HOST_REG_ARRAY(uint32_t, R_GPT0_Type, GTCCR);
	HOST_REG(uint32_t, R_GPT0_Type, GTPR);
	HOST_REG(uint32_t, R_GPT0_Type, GTPBR);
	HOST_REG(uint32_t, R_GPT0_Type, GTDTCR);
	HOST_REG(uint32_t, R_GPT0_Type, GTDVU);
};

// Peripheral pointers and the type names the drivers use
#undef R_SYSTEM
#undef R_FCACHE
#undef R_USB_FS0
#undef R_ICU
#undef R_PFS
#undef R_PMISC
#undef R_MSTP
#define R_SYSTEM		HOST_REG_OBJ(HostSystem, R_SYSTEM_BASE)
#define R_FCACHE		HOST_REG_OBJ(HostFcache, R_FCACHE_BASE)
#define R_USB_FS0		HOST_REG_OBJ(HostUsbFs, R_USB_FS0_BASE)
#define R_ICU			HOST_REG_OBJ(HostIcu, R_ICU_BASE)
#define R_PFS			HOST_REG_OBJ(HostPfs, R_PFS_BASE)
#define R_PMISC			HOST_REG_OBJ(HostPmisc, R_PMISC_BASE)
#define R_MSTP			HOST_REG_OBJ(HostMstp, R_MSTP_BASE)

// The vendor R_PORTn, R_AGTn and R_GPTn are casts of their base address to
// these type names
#define R_PORT0_Type	HostPort
#define R_AGTX0_Type	HostAgt
#define R_GPT0_Type		HostGpt
#undef R_PORT0_BASE
#undef R_PORT1_BASE
#undef R_PORT2_BASE
#undef R_PORT3_BASE
#undef R_PORT4_BASE
#undef R_PORT5_BASE
#undef R_PORT6_BASE
#undef R_PORT7_BASE
#undef R_PORT8_BASE
#undef R_PORT9_BASE
#undef R_AGTX0_BASE
#undef R_AGTX1_BASE
#undef R_GPT0_BASE
#undef R_GPT1_BASE
#undef R_GPT2_BASE
#undef R_GPT3_BASE
#undef R_GPT4_BASE
#undef R_GPT5_BASE
#undef R_GPT6_BASE
#undef R_GPT7_BASE
#define HOST_REG_BASE(Addr)		((uintptr_t)(s_HostRegSpace + ((Addr) - HOST_REG_FIRST)))
#define R_PORT0_BASE	HOST_REG_BASE(RA4M1_PCNTR1(0))
#define R_PORT1_BASE	HOST_REG_BASE(RA4M1_PCNTR1(1))
#define R_PORT2_BASE	HOST_REG_BASE(RA4M1_PCNTR1(2))
#define R_PORT3_BASE	HOST_REG_BASE(RA4M1_PCNTR1(3))
#define R_PORT4_BASE	HOST_REG_BASE(RA4M1_PCNTR1(4))
#define R_PORT5_BASE	HOST_REG_BASE(RA4M1_PCNTR1(5))
#define R_PORT6_BASE	HOST_REG_BASE(RA4M1_PCNTR1(6))
#define R_PORT7_BASE	HOST_REG_BASE(RA4M1_PCNTR1(7))
#define R_PORT8_BASE	HOST_REG_BASE(RA4M1_PCNTR1(8))
#define R_PORT9_BASE	HOST_REG_BASE(RA4M1_PCNTR1(9))
#define R_AGTX0_BASE	HOST_REG_BASE(RA4M1_AGT_BASE(0))
#define R_AGTX1_BASE	HOST_REG_BASE(RA4M1_AGT_BASE(1))
#define R_GPT0_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(0))
#define R_GPT1_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(1))
#define R_GPT2_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(2))
#define R_GPT3_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(3))
#define R_GPT4_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(4))
#define R_GPT5_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(5))
#define R_GPT6_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(6))
#define R_GPT7_BASE		HOST_REG_BASE(RA4M1_GPT_BASE(7))

#endif // RA4M1_HOST_REGS_H
