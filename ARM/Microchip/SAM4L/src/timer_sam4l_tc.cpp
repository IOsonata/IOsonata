/**-------------------------------------------------------------------------
@file	timer_sam4l.cpp

@brief	SAM4Lxx series TC timer implementation

@author	Hoang Nguyen Hoan
@date	Aug. 24, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include "timer_sam4l.h"

void Sam4lTcSetup(Sam4l_TimerData_t &d)
{
	Sam4lTimerPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK | d.ClockMask);
	// TIMER_CLOCK2..5: PBA /2, /8, /32, /128 (datasheet 30.10).
	uint32_t bit = 1UL << (2 * (d.Select - 1));
	Sam4lTimerPmWrite(&SAM4L_PM->PM_PBADIVMASK, SAM4L_PM->PM_PBADIVMASK | bit);
	d.TcReg->TC_CCR = TC_CCR_CLKDIS;
	d.TcReg->TC_IDR = 0xFFFFFFFF;
	d.TcReg->TC_CMR = TC_CMR_WAVEFORM_WAVE | d.Select;
	d.TcReg->TC_SMMR = 0;
	(void)d.TcReg->TC_SR;
	d.TcReg->TC_IER = TC_IER_COVFS;
}
void Sam4lTcRun(Sam4l_TimerData_t &d, bool run)
{
	if (!run) {
		d.TcReg->TC_CCR = TC_CCR_CLKDIS;
		return;
	}
	// Enabling and starting are separate operations (30.6.1.4). A trigger
	// issued while CLKDIS is set cannot start a newly configured counter.
	// Start/reset on the first enable after Reset; ordinary resume keeps CV.
	d.TcReg->TC_CCR = TC_CCR_CLKEN | (d.StartPending ? TC_CCR_SWTRG : 0);
	d.StartPending = false;
}
void Sam4lTcReset(Sam4l_TimerData_t &d)
{
	// Leave the counter paused. The next enable resets and starts it with
	// CLKEN | SWTRG. Count reads return zero until that first enable.
	d.TcReg->TC_CCR = TC_CCR_CLKDIS;
	d.StartPending = true;
	(void)d.TcReg->TC_SR;
}
uint64_t Sam4lTcCount(Sam4l_TimerData_t &d)
{
	if (d.StartPending) return 0;
	// TC_SR is read-to-clear. Account for rollover here as well as in the
	// ISR; absolute deadlines preserve compares consumed by foreground reads.
	uint32_t before = d.TcReg->TC_SR;
	uint32_t low = d.TcReg->TC_CV & 0xFFFF;
	uint32_t after = d.TcReg->TC_SR;
	if (after & TC_SR_COVFS) low = d.TcReg->TC_CV & 0xFFFF;
	if ((before | after) & TC_SR_COVFS) {
		d.Timer->Rollover += 65536ULL;
		d.Overflow = true;
	}
	if ((before | after) & d.TcReg->TC_IMR & (TC_SR_COVFS | TC_SR_CPAS | TC_SR_CPBS | TC_SR_CPCS)) NVIC_SetPendingIRQ(d.Irq);
	return d.Timer->Rollover + low;
}
void Sam4lTcArm(Sam4l_TimerData_t &d, int n, bool enable)
{
	uint32_t bit = TC_IER_CPAS << n;
	d.TcReg->TC_IDR = bit;
	if (!enable) return;
	uint32_t value = (uint32_t)d.Trigger[n].Deadline & 0xFFFF;
	switch (n) {
	case 0: d.TcReg->TC_RA = value; break;
	case 1: d.TcReg->TC_RB = value; break;
	case 2: d.TcReg->TC_RC = value; break;
	}
	d.TcReg->TC_IER = bit;
}
extern "C" void TC00_Handler(void) { Sam4lTimerIRQ(1); }
extern "C" void TC01_Handler(void) { Sam4lTimerIRQ(2); }
extern "C" void TC02_Handler(void) { Sam4lTimerIRQ(3); }
extern "C" void TC10_Handler(void) { Sam4lTimerIRQ(4); }
extern "C" void TC11_Handler(void) { Sam4lTimerIRQ(5); }
extern "C" void TC12_Handler(void) { Sam4lTimerIRQ(6); }
