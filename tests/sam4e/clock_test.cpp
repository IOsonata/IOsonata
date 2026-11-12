/**-------------------------------------------------------------------------
@file clock_test.cpp
@brief SAM4E clock transitions with modeled ready flags.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <cstdio>

#define _SAM4E_
#define __I volatile
#define __O volatile
#define __IO volatile
#define __WEAK
#include "component/pmc.h"
#include "component/efc.h"
#include "component/supc.h"
#include "component/wdt.h"

static uint32_t s_Status;
static uint32_t s_Fail;
static uint32_t s_Mck;
static unsigned s_Writes;
static unsigned s_MckWrites;
static unsigned s_FailMck;

struct MckRegister {
	operator uint32_t() const { return s_Mck; }
	void operator=(uint32_t Value)
	{
		uint32_t change = s_Mck ^ Value;

		assert(!((change & PMC_MCKR_CSS_Msk) && (change & PMC_MCKR_PRES_Msk)));
		s_Mck = Value;
		s_MckWrites++;
		s_Status &= ~PMC_SR_MCKRDY;
		if (s_MckWrites != s_FailMck)
		{
			s_Status |= PMC_SR_MCKRDY;
		}
		s_Writes++;
	}
	void operator&=(uint32_t Value) { *this = s_Mck & Value; }
};
struct MorRegister {
	uint32_t Value;
	operator uint32_t() const { return Value; }
	void operator=(uint32_t New)
	{
		assert((s_Mck & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_SLOW_CLK);
		assert((New & CKGR_MOR_KEY_Msk) == CKGR_MOR_KEY_PASSWD);
		Value = New & ~CKGR_MOR_KEY_Msk;
		s_Status &= ~(PMC_SR_MOSCRCS | PMC_SR_MOSCXTS | PMC_SR_MOSCSELS);
		if (New & CKGR_MOR_MOSCRCEN)
		{
			s_Status |= PMC_SR_MOSCRCS;
		}
		if (New & CKGR_MOR_MOSCXTEN)
		{
			s_Status |= PMC_SR_MOSCXTS;
		}
		if (New & CKGR_MOR_MOSCSEL)
		{
			s_Status |= PMC_SR_MOSCSELS;
		}
		s_Status &= ~s_Fail;
		s_Writes++;
	}
};
struct PllRegister {
	uint32_t Value;
	operator uint32_t() const { return Value; }
	void operator=(uint32_t New)
	{
		assert((s_Mck & PMC_MCKR_CSS_Msk) != PMC_MCKR_CSS_PLLA_CLK);
		Value = New;
		s_Status &= ~PMC_SR_LOCKA;
		if ((New & CKGR_PLLAR_MULA_Msk) && !(s_Fail & PMC_SR_LOCKA))
		{
			s_Status |= PMC_SR_LOCKA;
		}
		s_Writes++;
	}
};
struct StatusRegister {
	operator uint32_t() const { return s_Status; }
};
static struct {
	MorRegister CKGR_MOR;
	PllRegister CKGR_PLLAR;
	MckRegister PMC_MCKR;
	StatusRegister PMC_SR;
	uint32_t PMC_SCSR;
	uint32_t PMC_WPMR;
	uint32_t PMC_USB;
} s_Pmc;
static Sam4eEfc s_Efc;
static Sam4eSupc s_Supc;
static Sam4eWdt s_Wdt;
#define SAM4E_PMC (&s_Pmc)
#define SAM4E_EFC (&s_Efc)
#define SAM4E_SUPC (&s_Supc)
#define SAM4E_WDT (&s_Wdt)
#include "../../ARM/Microchip/SAM4E/src/system_sam4e.c"
uint32_t SystemResetReason(void) { return 0; }
void SystemOscInit(void) {}

static void Reset(void)
{
	s_Fail = 0;
	s_FailMck = 0;
	s_MckWrites = 0;
	s_Mck = PMC_MCKR_CSS_MAIN_CLK;
	s_Pmc.CKGR_MOR.Value = CKGR_MOR_MOSCRCEN;
	s_Pmc.CKGR_PLLAR.Value = 0;
	s_Pmc.PMC_SCSR = 0;
	s_Pmc.PMC_WPMR = 0;
	s_Status = PMC_SR_MCKRDY | PMC_SR_MOSCRCS;
	s_Writes = 0;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
	g_McuOsc.CoreOsc.Freq = 12000000;
	g_McuOsc.bUSBClk = false;
	s_Supc.SUPC_SR = 0;
	s_Efc.EEFC_FMR = EEFC_FMR_FRDY;
}

int main(void)
{
	uint32_t failures[] = {PMC_SR_MOSCRCS, PMC_SR_MOSCXTS, PMC_SR_MOSCSELS, PMC_SR_LOCKA};
	uint32_t requests[] = {0, 6000000, 6000001, 10000000, 10000001, UINT32_MAX};
	uint32_t expected[] = {4000000, 4000000, 8000000, 8000000, 12000000, 12000000};
	unsigned i;

	Reset();
	SystemInit();
	assert(SystemCoreClock == 120000000);
	assert(s_Efc.EEFC_FMR & EEFC_FMR_FRDY);
	assert(s_Wdt.WDT_MR == WDT_MR_WDDIS);
	// A second selection must leave PLLA before modifying its source.
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 16000000));
	assert(SystemCoreClock == 120000000);
	for (i = 0; i < sizeof(requests) / sizeof(requests[0]); i++)
	{
		assert(SystemCoreClockSelect(OSC_TYPE_RC, requests[i]));
		assert(g_McuOsc.CoreOsc.Freq == expected[i]);
		assert(SystemCoreClock == 120000000);
	}
	Reset();
	assert(SystemCoreClockSelect(OSC_TYPE_TCXO, 50000000));
	assert(s_Pmc.CKGR_MOR.Value & CKGR_MOR_MOSCXTBY);
	assert(!(s_Pmc.CKGR_MOR.Value & CKGR_MOR_MOSCXTEN));
	assert(SystemCoreClock <= 120000000 && SystemCoreClock > 110000000);
	Reset();
	g_McuOsc.bUSBClk = true;
	assert(SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
	assert(s_Pmc.PMC_USB == PMC_USB_USBDIV(4));
	Reset();
	assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 20000001));
	assert(!SystemCoreClockSelect(OSC_TYPE_TCXO, 50000001));
	assert(!SystemCoreClockSelect((OSC_TYPE)3, 12000000));
	assert(s_Writes == 0);
	s_Pmc.PMC_SCSR = PMC_SCSR_UDP;
	assert(!SystemCoreClockSelect(OSC_TYPE_RC, 4000000));
	assert(s_Writes == 0);
	s_Pmc.PMC_SCSR = 0;
	s_Pmc.PMC_WPMR = PMC_WPMR_WPEN;
	assert(!SystemCoreClockSelect(OSC_TYPE_RC, 4000000));
	assert(s_Writes == 0);
	for (i = 0; i < sizeof(failures) / sizeof(failures[0]); i++)
	{
		Reset();
		s_Fail = failures[i];
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(SystemCoreClock == 32000);
	}
	for (i = 1; i <= 3; i++)
	{
		Reset();
		s_FailMck = i;
		assert(!SystemCoreClockSelect(OSC_TYPE_XTAL, 12000000));
		assert(s_MckWrites == i);
	}
	Reset();
	s_Mck = PMC_MCKR_CSS_PLLA_CLK | PMC_MCKR_PLLADIV2 | PMC_MCKR_PRES_CLK_3;
	s_Pmc.CKGR_PLLAR.Value = CKGR_PLLAR_DIVA(1) | CKGR_PLLAR_MULA(59);
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 40000000);
	s_Pmc.CKGR_PLLAR.Value = 0;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 0);
	assert(SystemPeriphClockGet(-1) == 0);
	assert(SystemPeriphClockGet(1) == 0);
	std::puts("SAM4E clock selection, transition ordering, PLL limits and timeout tests PASS");
}
