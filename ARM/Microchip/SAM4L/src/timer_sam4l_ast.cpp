/**-------------------------------------------------------------------------
@file	timer_sam4l_ast.cpp

@brief	SAM4Lxx series AST timer implementation

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

// Bounded even if the selected source fails. At 48 MHz this allows several
// 32 kHz synchronization cycles. No unbounded waits in a callback/ISR.
bool Sam4lAstWait(uint32_t mask)
{
	for (uint32_t i = 0; i < 100000; ++i)
		if (!(SAM4L_AST->AST_SR & mask)) return true;
	return false;
}
static bool Write(volatile uint32_t *reg, uint32_t value)
{
	if (!Sam4lAstWait(AST_SR_BUSY)) return false;
	*reg = value;
	return Sam4lAstWait(AST_SR_BUSY);
}
bool Sam4lAstSetup(Sam4l_TimerData_t &d)
{
	Sam4lTimerPmWrite(&SAM4L_PM->PM_PBDMASK, SAM4L_PM->PM_PBDMASK | PM_PBDMASK_AST);
	SAM4L_AST->AST_IDR = 0xFFFFFFFF;
	if (!Sam4lAstWait(AST_SR_BUSY | AST_SR_CLKBUSY)) return false;
	// CSSEL and CEN must never change in the same write (19.5.1).
	SAM4L_AST->AST_CLOCK = SAM4L_AST->AST_CLOCK & ~AST_CLOCK_CEN;
	if (!Sam4lAstWait(AST_SR_CLKBUSY)) return false;
	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK;
	if (!Sam4lAstWait(AST_SR_CLKBUSY)) return false;
	SAM4L_AST->AST_CLOCK = AST_CLOCK_CSSEL_32KHZCLK | AST_CLOCK_CEN;
	if (!Sam4lAstWait(AST_SR_CLKBUSY)) return false;
	if (!Write(&SAM4L_AST->AST_CR, AST_CR_PSEL(d.Select)) ||
		!Write(&SAM4L_AST->AST_DTR, 0) ||
		!Write(&SAM4L_AST->AST_SCR, AST_SCR_MASK)) return false;
	SAM4L_AST->AST_IER = AST_SR_OVF;
	return true;
}
bool Sam4lAstRun(Sam4l_TimerData_t &d, bool run)
{
	return Write(&SAM4L_AST->AST_CR, AST_CR_PSEL(d.Select) | (run ? AST_CR_EN : 0));
}
bool Sam4lAstReset(Sam4l_TimerData_t &d)
{
	return Sam4lAstRun(d, false) &&
		Write(&SAM4L_AST->AST_CR, AST_CR_PSEL(d.Select) | AST_CR_PCLR) &&
		Write(&SAM4L_AST->AST_CV, 0) && Write(&SAM4L_AST->AST_SCR, AST_SCR_MASK);
}
uint64_t Sam4lAstCount(Sam4l_TimerData_t &d)
{
	if (!Sam4lAstWait(AST_SR_BUSY)) { d.Healthy = false; return d.Timer->Rollover + d.Timer->LastCount; }
	uint32_t before = SAM4L_AST->AST_SR;
	uint32_t low = SAM4L_AST->AST_CV;
	uint32_t after = SAM4L_AST->AST_SR;
	if ((after & AST_SR_OVF) && !(before & AST_SR_OVF)) low = SAM4L_AST->AST_CV;
	uint32_t flags = (before | after) & (AST_SR_OVF | AST_SR_ALARM0);
	if (flags) {
		if (!Write(&SAM4L_AST->AST_SCR, flags)) d.Healthy = false;
		if (flags & AST_SR_OVF) { d.Timer->Rollover += 0x100000000ULL; d.Overflow = true; }
		NVIC_SetPendingIRQ(d.Irq);
	}
	return d.Timer->Rollover + low;
}
bool Sam4lAstArm(Sam4l_TimerData_t &d, bool enable)
{
	SAM4L_AST->AST_IDR = AST_SR_ALARM0;
	if (!Write(&SAM4L_AST->AST_SCR, AST_SR_ALARM0)) return false;
	if (!enable) return true;
	if (!Write(&SAM4L_AST->AST_AR0, (uint32_t)d.Trigger[0].Deadline)) return false;
	SAM4L_AST->AST_IER = AST_SR_ALARM0;
	return true;
}
extern "C" void AST_ALARM_Handler(void) { Sam4lTimerIRQ(0); }
extern "C" void AST_OVF_Handler(void) { Sam4lTimerIRQ(0); }
