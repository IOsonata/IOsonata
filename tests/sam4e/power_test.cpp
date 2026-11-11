/**-------------------------------------------------------------------------
@file power_test.cpp
@brief SAM4E power functions against host registers and vendor bit definitions.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <cassert>
#include <cstdint>
#include <csetjmp>
#include <cstdio>

#define _SAM4E_
#define __INTERRUPT_H__
#define __I volatile
#define __O volatile
#define __IO volatile
#include "component/supc.h"
#include "component/rstc.h"
#include "component/gpbr.h"
#include "component/rtc.h"
#include "component/rtt.h"

static Sam4eSupc s_Supc;
static Sam4eRstc s_Rstc;
static Sam4eGpbr s_Gpbr;
static Sam4eRtc s_Rtc;
static Sam4eRtt s_Rtt;
static std::jmp_buf s_Off;
static uint32_t s_Mask;
static uint32_t DisableInterrupt()
{
	uint32_t previous;

	previous = s_Mask;
	s_Mask = 1;
	return previous;
}
static void EnableInterrupt(uint32_t Previous)
{
	s_Mask = Previous;
}
static void __DSB() {}
static void __WFE()
{
	std::longjmp(s_Off, 1);
}
#define SAM4E_SUPC (&s_Supc)
#define SAM4E_RSTC (&s_Rstc)
#define SAM4E_GPBR (&s_Gpbr)
#define SAM4E_RTC (&s_Rtc)
#define SAM4E_RTT (&s_Rtt)
#include "../../ARM/Microchip/SAM4E/src/system_power_sam4e.cpp"

static bool s_ClockStarts;
static uint32_t s_ClockCommand;
struct ClockStatus {
	uint32_t Value;
	operator uint32_t()
	{
		if (s_ClockStarts && (s_ClockCommand & SUPC_CR_XTALSEL))
		{
			Value |= SUPC_SR_OSCSEL;
		}
		return Value;
	}
};
static struct {
	uint32_t SUPC_MR;
	uint32_t &SUPC_CR = s_ClockCommand;
	ClockStatus SUPC_SR;
} s_Clock;
McuOsc_t g_McuOsc = {};
#undef SAM4E_SUPC
#define SAM4E_SUPC (&s_Clock)
#include "../../ARM/Microchip/SAM4E/src/system_clock_sam4e.cpp"

int main()
{
	static const uint32_t reasons[] = {
		SYSTEM_RESET_POWERON, SYSTEM_RESET_WAKE_PIN, SYSTEM_RESET_WDT,
		SYSTEM_RESET_SOFT, SYSTEM_RESET_PIN
	};
	static const int ports[] = {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0};
	static const int pins[] = {0,1,2,4,5,8,9,11,14,19,20,30,2,5,15,16};
	uint32_t before;
	uint32_t bit;
	int i;

	for (i = 0; i < 5; i++)
	{
		s_Sam4eResetRead = false;
		s_Rstc.RSTC_SR = uint32_t(i) << 8;
		s_Supc.SUPC_SR = SUPC_SR_WKUPS;
		assert(SystemResetReason() == reasons[i]);
		s_Rstc.RSTC_SR = 0;
		s_Supc.SUPC_SR = 0;
		assert(SystemResetReason() == reasons[i]);
	}
	s_Sam4eResetRead = false;
	s_Rstc.RSTC_SR = RSTC_SR_RSTTYP_BACKUP_RST;
	s_Supc.SUPC_WUMR = SUPC_WUMR_RTCEN;
	s_Rtc.RTC_SR = RTC_SR_ALARM;
	assert(SystemResetReason() == SYSTEM_RESET_WAKE_TIMER);
	s_Sam4eResetRead = false;
	s_Supc.SUPC_WUMR = SUPC_WUMR_RTTEN;
	s_Rtt.RTT_SR = RTT_SR_ALMS;
	assert(SystemResetReason() == SYSTEM_RESET_WAKE_TIMER);
	for (i = 0; i < 16; i++)
	{
		bit = 1U << i;
		s_Supc.SUPC_WUIR = 0xffffffffU;
		assert(SystemWakePin(ports[i], pins[i], IOPINSENSE_LOW_TRANSITION));
		assert(s_Supc.SUPC_WUIR == (0xffffffffU & ~(bit << 16)));
		assert(SystemWakePin(ports[i], pins[i], IOPINSENSE_HIGH_TRANSITION));
		assert(s_Supc.SUPC_WUIR == 0xffffffffU);
		assert(SystemWakePin(ports[i], pins[i], IOPINSENSE_DISABLE));
		assert(s_Supc.SUPC_WUIR == (0xffffffffU & ~(bit | (bit << 16))));
		assert(s_Mask == 0);
	}
	before = s_Supc.SUPC_WUIR;
	assert(!SystemWakePin(-1, 0, IOPINSENSE_LOW_TRANSITION));
	assert(!SystemWakePin(0, 3, IOPINSENSE_LOW_TRANSITION));
	assert(!SystemWakePin(0, 0, IOPINSENSE_TOGGLE));
	assert(s_Supc.SUPC_WUIR == before);
	assert(SystemRetRegCount() == 20);
	for (i = 0; i < 20; i++)
	{
		assert(SystemRetRegWrite(i, 0xa5a50000U + i));
		assert(SystemRetRegRead(i) == 0xa5a50000U + i);
	}
	assert(!SystemRetRegWrite(-1, 1));
	assert(!SystemRetRegWrite(20, 1));
	assert(SystemRetRegRead(-1) == 0);
	assert(SystemRetRegRead(20) == 0);
	assert(!SystemRamRetain(0x20000000U, 4, true));
	assert(!SystemRamRetain(UINTPTR_MAX, UINT32_MAX, false));
	if (setjmp(s_Off) == 0)
	{
		SystemPowerOff();
		assert(false);
	}
	assert(s_Supc.SUPC_CR == (SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF));
	assert(SystemLowFreqClockSelect(OSC_TYPE_RC, 32000));
	assert(g_McuOsc.LowPwrOsc.Freq == 32000);
	assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 40000));
	assert(s_ClockCommand == 0);
	assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
	assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC);
	assert(s_ClockCommand == (SUPC_CR_KEY_PASSWD | SUPC_CR_XTALSEL));
	s_ClockStarts = true;
	assert(SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
	assert(g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL);
	assert(g_McuOsc.LowPwrOsc.Freq == 32768);
	assert(!SystemLowFreqClockSelect(OSC_TYPE_RC, 32000));
	assert(!SystemLowFreqClockSelect(OSC_TYPE_TCXO, 32768));
	s_Clock.SUPC_SR.Value = 0;
	s_ClockCommand = 0;
	s_Clock.SUPC_MR = SUPC_MR_BODRSTEN;
	assert(SystemLowFreqClockSelect(OSC_TYPE_TCXO, 32768));
	assert(s_Clock.SUPC_MR == (SUPC_MR_KEY_PASSWD | SUPC_MR_BODRSTEN | SUPC_MR_OSCBYPASS));
	assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL, 32768));
	puts("SAM4E reset, wake pins, retention, Backup entry and slow clock PASS");
}
