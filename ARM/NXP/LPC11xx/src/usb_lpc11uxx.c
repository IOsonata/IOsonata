/**-------------------------------------------------------------------------
@file usb_lpc11uxx.c
@brief LPC11Uxx native full-speed USB device controller.

Hardware preparation, lifecycle, EP0 and bulk/interrupt transport. Generic USB
owns Chapter 9 and device classes. No ROM USB stack is initialized here.
ISO and low-power suspend are not implemented.

Register sequences: NXP UM10462, chapters 3 and 11. Register declarations
come from the vendor CMSIS header. Only the DMA memory layout is defined here.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
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
#include <stddef.h>
#include <string.h>

#include "LPC11Uxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "lpc_iap.h"
#include "usb/usb.h"
#include "usb_hw_lpc11uxx.h"
#include "usb_ep0_lpc_fs.h"
#include "usb_data_lpc_fs.h"

#define LPC11_USB_AHB_CLOCKS ((1UL << 14) | (1UL << 27))
#define LPC11_USB_ENABLE (1UL << 7)
#define LPC11_USB_CONNECT (1UL << 16)
#define LPC11_USB_PLL_ON (1UL << 9)
#define LPC11_USB_NO_WAKE ((1UL << 17) | (1UL << 19))
#define LPC11_USB_STATUS_W1C ((1UL << 8) | (7UL << 24))
#define LPC11_USB_COMMAND_MASK (0xFFUL | (1UL << 9) | (1UL << 11) | (0x1FUL << 12))
#define LPC11_USB_INTERRUPTS (0x3FFUL | (3UL << 30))
#define LPC11_USB_ACTIVE (1UL << 31)
#define LPC11_USB_DISABLED (1UL << 30)
#define LPC11_USB_WAIT_COUNT 1024U
#define LPC11_USB_STALL (1UL << 29)
#define LPC11_USB_SETUP (1UL << 8)
#define LPC11_USB_NAK_CONTROL (3UL << 14)
#define LPC11_USB_RESET (1UL << 26)
#define LPC11_USB_SUSPEND_CHANGE (1UL << 25)
#define LPC11_USB_CONNECT_CHANGE (1UL << 24)
#define LPC11_USB_DEVICE_INTERRUPT (1UL << 31)
#define LPC11_USB_SOF_INTERRUPT (1UL << 30)
#define LPC11_USB_TOGGLE_RESET (1UL << 28)

// Maintained by the existing LPC11Uxx clock implementation, including its
// legacy SystemCoreClockSet entry point (which need not update g_McuOsc).
extern uint32_t SystemMainClkFreq;

// NOLOAD: initialized explicitly only AFTER the USB SRAM clock is enabled.
// No KEEP in the linker script: a non-USB program pays no memory cost.
static Lpc11UsbMemory_t s_Memory
	__attribute__((section(".usb_ram"), aligned(256)));

static bool Lpc11UsbControlReady(void);
static void Lpc11UsbEp0Fault(void);
static void Lpc11UsbControlEvent(const UsbCtrlrEvt_t *pEvent);

static LpcUsbFsEp0_t s_Ep0 = {
	.pReg = LPC_USB,
	.pMemory = &s_Memory,
	.Ready = Lpc11UsbControlReady,
	.Fault = Lpc11UsbEp0Fault,
};
static bool s_Prepared;
static uint32_t s_AddedClocks;
static bool s_Initialized;
static int s_VbusInt = -1;
static int s_IntPrio;
static bool s_Closing;
static bool s_Detached;
static bool s_SofEnabled;
static bool s_WakeAddedClock;
// 0 idle, 1..3 waiting for the minimum bus-idle time, 4 signalling resume.
static uint8_t s_WakeFrames;

static LpcUsbFsData_t s_Data = {
	.pEp0 = &s_Ep0,
	.pPrepared = &s_Prepared,
	.pClosing = &s_Closing,
	.Irq = USB_IRQn,
};

static bool Lpc11UsbRetire(uint32_t Mask);
static void Lpc11UsbWakeCancel(void);
static void Lpc11UsbBusReset(uint32_t Status, bool Detached);

_Static_assert(offsetof(Lpc11UsbMemory_t, Setup) == 128, "SETUP alignment");
_Static_assert(offsetof(Lpc11UsbMemory_t, Packet) == 192, "packet alignment");
_Static_assert(sizeof(Lpc11UsbMemory_t) == 832, "USB SRAM storage size");

static bool Lpc11UsbClockReady(void)
{
	const uint32_t ahbdiv = LPC_SYSCON->SYSAHBCLKDIV & 0xFFU;
	const uint32_t usbdiv = LPC_SYSCON->USBCLKDIV & 0xFFU;
	const uint32_t main = LPC_SYSCON->MAINCLKSEL & MAINCLKSEL_MASK;
	const uint32_t power = LPC_SYSCON->PDRUNCFG;
	const uint32_t sysmul = (LPC_SYSCON->SYSPLLCTRL & 0x1FU) + 1U;

	// The supported clock setup uses SYSOSC for the CPU and USB. Do not
	// infer SYSOSC from g_McuOsc: the legacy clock setter may have changed it.
	if (ahbdiv == 0U || usbdiv == 0U || SystemCoreClockGet() < 16000000U ||
		SystemMainClkFreq / ahbdiv != SystemCoreClockGet() ||
		(power & (PDRUNCFG_SYSOSC_PD | PDRUNCFG_USBPAD_PD)) != 0U ||
		(LPC_SYSCON->SYSPLLCLKSEL & SYSPLLCLKSEL_MASK) != SYSPLLCLKSEL_SYSOSC ||
		(LPC_SYSCON->SYSPLLCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->MAINCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->USBCLKUEN & 1U) == 0U ||
		(main != MAINCLKSEL_PLLOUT && main != MAINCLKSEL_PLLIN))
		return false;
	if (main == MAINCLKSEL_PLLOUT &&
		((power & PDRUNCFG_SYSPLL_PD) != 0U ||
		 (LPC_SYSCON->SYSPLLSTAT & 1U) == 0U || SystemMainClkFreq % sysmul != 0U))
		return false;

	const uint32_t input = main == MAINCLKSEL_PLLOUT ?
		SystemMainClkFreq / sysmul : SystemMainClkFreq;
	if (input < 10000000U || input > 25000000U)
		return false;

	const uint32_t source = LPC_SYSCON->USBCLKSEL & 3U;
	if (source == USBCLKSEL_MAINCLK)
		return main == MAINCLKSEL_PLLOUT && SystemMainClkFreq == 48000000U && usbdiv == 1U;
	if (source != USBCLKSEL_PLLOUT ||
		(power & PDRUNCFG_USBPLL_PD) != 0U ||
		(LPC_SYSCON->USBPLLCLKSEL & USBPLLCLKSEL_MASK) != USBPLLCLKSEL_SYSOSC ||
		(LPC_SYSCON->USBPLLCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->USBPLLSTAT & 1U) == 0U)
		return false;

	const uint32_t control = LPC_SYSCON->USBPLLCTRL;
	const uint64_t output = (uint64_t)input * ((control & 0x1FU) + 1U);
	const uint64_t cco = output * (2U << ((control >> 5) & 3U));
	return output == (uint64_t)48000000U * usbdiv &&
		cco >= 156000000U && cco <= 320000000U;
}

void Lpc11UsbHwAckStatus(uint32_t Flags)
{
	if (!s_Prepared)
		return;
	// Reading and writing all bits would clear unrelated W1C events.
	// DSUS/LPM_SUS must be written as one even if the read saw zero: the
	// controller may have suspended between that read and this write.
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT & LPC11_USB_COMMAND_MASK) |
		LPC11_USB_NO_WAKE | (Flags & LPC11_USB_STATUS_W1C);
}

Lpc11UsbMemory_t *Lpc11UsbHwPrepare(int IntPrio)
{
	const uintptr_t address = (uintptr_t)&s_Memory;
	if (s_Prepared || address < 0x20004000U ||
		address > 0x20004800U - sizeof(s_Memory) || (address & 255U) != 0U ||
		!Lpc11UsbClockReady())
		return NULL;

	const uint32_t clocks = LPC_SYSCON->SYSAHBCLKCTRL;
	LPC_SYSCON->SYSAHBCLKCTRL = clocks | LPC11_USB_AHB_CLOCKS;
	__DSB();
	// Do not take over a live native/ROM controller, or its DMA list.
	if ((LPC_USB->DEVCMDSTAT & (LPC11_USB_ENABLE | LPC11_USB_CONNECT)) != 0U ||
		(LPC_USB->EPSKIP & 0x3FFU) != 0U)
	{
		LPC_SYSCON->SYSAHBCLKCTRL = clocks;
		return NULL;
	}

	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	LPC_USB->DEVCMDSTAT = LPC11_USB_PLL_ON | LPC11_USB_NO_WAKE | LPC11_USB_STATUS_W1C;
	memset(&s_Memory, 0, sizeof(s_Memory));
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		const uint32_t buffer = (uint32_t)(uintptr_t)s_Memory.Packet[physical];
		s_Memory.Command[physical * 2U] = ((buffer >> 6) & 0xFFFFU) |
			(physical >= 2U ? LPC11_USB_DISABLED : 0U);
		if (physical >= 2U)
			s_Memory.Command[physical * 2U + 1U] = LPC11_USB_DISABLED;
	}
	s_Memory.Command[1] = ((uint32_t)(uintptr_t)s_Memory.Setup >> 6) & 0xFFFFU;
	__DMB();
	LPC_USB->EPLISTSTART = (uint32_t)address;
	LPC_USB->DATABUFSTART = (uint32_t)address & 0xFFC00000U;
	LPC_USB->EPINUSE = 0U;
	LPC_USB->EPBUFCFG = 0U;
	LPC_USB->INTROUTING = 0U;
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	NVIC_ClearPendingIRQ(USB_IRQn);
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	NVIC_SetPriority(USB_IRQn, IntPrio < 0 ? 0U :
		(IntPrio > maxprio ? (uint32_t)maxprio : (uint32_t)IntPrio));
	s_AddedClocks = LPC11_USB_AHB_CLOCKS & ~clocks;
	s_Detached = false;
	s_Prepared = true;
	return &s_Memory;
}

bool Lpc11UsbHwRelease(void)
{
	if (s_Closing)
		return false;
	s_Ep0.Enabled = false;
	++s_Ep0.Generation;
	if (!s_Prepared)
		return true;
	s_SofEnabled = false;
	Lpc11UsbWakeCancel();
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	// Disconnect but leave DEV_EN set until outstanding buffers retire.
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT &
		(LPC11_USB_ENABLE | 0x7FU)) | LPC11_USB_PLL_ON | LPC11_USB_NO_WAKE;
	if (!Lpc11UsbRetire(0x3FFU))
		return false;
	LpcUsbFsDataForget(0x3FCU);
	LPC_USB->DEVCMDSTAT = LPC11_USB_NO_WAKE | LPC11_USB_STATUS_W1C;
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	__DSB();
	NVIC_ClearPendingIRQ(USB_IRQn);
	LPC_SYSCON->SYSAHBCLKCTRL &= ~s_AddedClocks;
	s_Prepared = false;
	return true;
}

static void Lpc11UsbCommand(uint32_t Clear, uint32_t Set, uint32_t Ack)
{
	LpcUsbFsEp0Command(&s_Ep0, Clear, Set, Ack);
}

static void Lpc11UsbVbusEvent(int IntNo, void *pContext)
{
	(void)IntNo;
	(void)pContext;
	// GPIO detection stays live while USB is stopped. DCON_C alone only
	// reports loss of VBUS and cannot start a board powered without a cable.
	UsbProcessQue(0);
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == NULL || pCfg->bLowPowerSuspend ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 1)
		return false;
	const IOPinCfg_t *vbus = &pCfg->pIOPinMap[USB_VBUS_PIN_IDX];
	if (vbus->PortNo != 0 || vbus->PinNo != 3 ||
		vbus->PinOp != IOPINOP_FUNC1 || vbus->PinDir != IOPINDIR_INPUT ||
		vbus->Res != IOPINRES_NONE || vbus->Type != IOPINTYPE_NORMAL)
		return false;
	const uint32_t state = DisableInterrupt();
	// A failed retirement still owns SRAM and must not be reinitialized.
	if (s_Prepared || s_Closing)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Initialized = false;
	LpcUsbFsDataInit();
	if (s_VbusInt >= 0)
		IOPinDisableInterrupt(s_VbusInt);
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	s_IntPrio = pCfg->IntPrio < 0 ? 0 :
		(pCfg->IntPrio > maxprio ? maxprio : pCfg->IntPrio);
	IOPinCfg(vbus, 1);
	s_VbusInt = IOPinAllocateInterrupt(s_IntPrio, 0, 3, IOPINSENSE_TOGGLE,
		Lpc11UsbVbusEvent, NULL);
	s_Initialized = s_VbusInt >= 0;
	EnableInterrupt(state);
	return s_Initialized;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	// UM10462 9.5.3.5: PIN reads the pad even in an alternate function.
	// No USB register access is needed while its clocks are disabled.
	return DevNo == 0 && s_Initialized && IOPinRead(0, 3) != 0;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || s_Closing)
		return false;
	const uint32_t state = DisableInterrupt();
	bool ready = false;
	if (UsbCtrlrVbusDetected(0))
	{
		if (s_Prepared)
			ready = s_Ep0.Enabled;
		else if (Lpc11UsbHwPrepare(s_IntPrio) != NULL)
		{
			ready = Lpc11UsbEp0Enable();
			if (ready)
			{
				// Fixed MCU soft-connect output (UM10462 table 67), not the
				// generic pin map's ID entry. DCON remains off until Connect.
				IOPinConfig(0, 6, IOPINOP_FUNC1, IOPINDIR_OUTPUT,
					IOPINRES_NONE, IOPINTYPE_NORMAL);
			}
			else
				(void)Lpc11UsbHwRelease();
		}
	}
	EnableInterrupt(state);
	return ready;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	// Retry is allowed after a timed-out skip. Release leaves the controller
	// disconnected and disabled to callers until retirement is confirmed.
	(void)Lpc11UsbHwRelease();
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	if (s_Prepared && s_Ep0.Enabled && !s_Closing && s_Initialized)
	{
		if (UsbCtrlrVbusDetected(0))
			s_Detached = false;
		else if (!s_Detached)
			Lpc11UsbBusReset(LPC_USB->DEVCMDSTAT, true);
	}
	EnableInterrupt(state);
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}

void UsbCtrlrIntEnable(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled)
		NVIC_EnableIRQ(USB_IRQn);
	EnableInterrupt(state);
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
		NVIC_DisableIRQ(USB_IRQn);
}

void UsbCtrlrConnect(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled && UsbCtrlrVbusDetected(0))
		Lpc11UsbCommand(0U, LPC11_USB_CONNECT, 0U);
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared)
	{
		Lpc11UsbWakeCancel();
		Lpc11UsbCommand(LPC11_USB_CONNECT, 0U, 0U);
	}
	EnableInterrupt(state);
}

static void Lpc11UsbFrameEnable(void)
{
	if (s_SofEnabled || (s_WakeFrames != 0U && s_WakeFrames < 4U))
		LPC_USB->INTEN |= LPC11_USB_SOF_INTERRUPT;
	else
		LPC_USB->INTEN &= ~LPC11_USB_SOF_INTERRUPT;
}

static void Lpc11UsbWakeCancel(void)
{
	s_WakeFrames = 0U;
	if (s_WakeAddedClock)
		LPC_SYSCON->USBCLKCTRL &= ~1U;
	s_WakeAddedClock = false;
	Lpc11UsbFrameEnable();
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled && !s_Closing && !s_Detached &&
		s_WakeFrames == 0U && UsbCtrlrVbusDetected(0))
	{
		const uint32_t status = LPC_USB->DEVCMDSTAT;
		if ((status & (LPC11_USB_CONNECT | (1U << 17))) ==
			(LPC11_USB_CONNECT | (1U << 17)) &&
			(status & (LPC11_USB_RESET | LPC11_USB_CONNECT_CHANGE)) == 0U)
		{
			// The generic core checks host permission. DSUS proves 3 ms
			// idle, not the required 5 ms. UM10462 table 205 FRAME_INT is
			// a local millisecond timer, including suspend with PLL_ON=1.
			// Discard an old tick, then three edges give at least 2 ms.
			LPC_USB->INTSTAT = LPC11_USB_SOF_INTERRUPT;
			s_WakeFrames = 3U;
			Lpc11UsbFrameEnable();
		}
	}
	EnableInterrupt(state);
}

static void Lpc11UsbWakeFrame(void)
{
	if (s_WakeFrames == 0U || s_WakeFrames == 4U)
		return;
	const uint32_t status = LPC_USB->DEVCMDSTAT;
	if (!UsbCtrlrVbusDetected(0) ||
		(status & (LPC11_USB_CONNECT | (1U << 17))) !=
		(LPC11_USB_CONNECT | (1U << 17)) ||
		(status & (LPC11_USB_RESET | LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE)) != 0U)
	{
		Lpc11UsbWakeCancel();
		return;
	}
	if (--s_WakeFrames != 0U)
		return;
	// Clocks remain powered throughout suspend in this mode. Follow 11.7.6.2:
	// force need_clock before writing DSUS=0; release our force on resume.
	s_WakeAddedClock = (LPC_SYSCON->USBCLKCTRL & 1U) == 0U;
	LPC_SYSCON->USBCLKCTRL |= 1U;
	__DSB();
	s_WakeFrames = 4U;
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT & LPC11_USB_COMMAND_MASK) | (1U << 19);
	Lpc11UsbFrameEnable();
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled)
	{
		s_SofEnabled = Enable;
		LPC_USB->INTSTAT = LPC11_USB_SOF_INTERRUPT;
		Lpc11UsbFrameEnable();
	}
	EnableInterrupt(state);
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (pBuff == NULL || BuffLen == 0U)
		return 0U;
	pBuff[0] = '\0';
	if (DevNo != 0 || BuffLen == 1U)
		return 0U;
	IAP_CMD_STATUS result;
	if (IAPReadUID(&result) != IAPSTATUS_SUCCESS)
		return 0U;
	// Four ROM UID words, eight uppercase hexadecimal digits per word.
	// Usable before Init: the generic core asks for its serial first.
	const size_t count = BuffLen > 32U ? 32U : BuffLen - 1U;
	for (size_t i = 0U; i < count; ++i)
	{
		const uint8_t digit = (result.Data[i / 8U] >> (28U - (i % 8U) * 4U)) & 15U;
		pBuff[i] = digit < 10U ? '0' + digit : 'A' + digit - 10;
	}
	pBuff[count] = '\0';
	return count;
}

static bool Lpc11UsbRetire(uint32_t Mask)
{
	return LpcUsbFsEp0Retire(&s_Ep0, Mask);
}

static void Lpc11UsbEp0Fault(void)
{
	// Retain DMA storage on a failed skip. Stop bus/IRQ activity without
	// pretending the hardware has returned ownership of an active buffer.
	s_Ep0.Enabled = false;
	s_SofEnabled = false;
	Lpc11UsbWakeCancel();
	++s_Ep0.Generation;
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	Lpc11UsbCommand(LPC11_USB_CONNECT | LPC11_USB_NAK_CONTROL, 0U, 0U);
}

bool Lpc11UsbEp0Enable(void)
{
	if (!s_Prepared || s_Ep0.Enabled)
		return false;
	const uint32_t state = DisableInterrupt();
	if ((LPC_USB->DEVCMDSTAT & LPC11_USB_CONNECT) != 0U ||
		!Lpc11UsbRetire(3U))
	{
		EnableInterrupt(state);
		return false;
	}
	s_Ep0.Event = Lpc11UsbControlEvent;
	LpcUsbFsEp0Reset(&s_Ep0);
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	Lpc11UsbCommand(0x7FU | LPC11_USB_NAK_CONTROL,
		LPC11_USB_ENABLE, LPC11_USB_STATUS_W1C);
	LPC_USB->INTEN = 3U | LPC11_USB_DEVICE_INTERRUPT;
	EnableInterrupt(state);
	return true;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	return LpcUsbFsEp0Send(&s_Ep0, DevNo, pBuffer, Length);
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	return LpcUsbFsEp0Status(&s_Ep0, DevNo, EpAddr);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	LpcUsbFsEp0SetAddress(&s_Ep0, DevNo, Address);
}

// Reset and cable loss share the ownership sequence. Do not reset the generic
// classes until every DMA bank is retired and its owner has been notified.
static void Lpc11UsbBusReset(uint32_t Status, bool Detached)
{
	// Only a real bus reset can leave a new host SETUP to process. Cable
	// loss invalidates the old request even if VBUS has already returned.
	const bool clearSetup = Detached || (Status & LPC11_USB_RESET) == 0U;
	s_Detached = Detached;
	Lpc11UsbWakeCancel();
	if (!Lpc11UsbRetire(0x3FFU))
	{
		Lpc11UsbEp0Fault();
		return;
	}
	LpcUsbFsDataForget(0x3FCU);
	LPC_USB->INTEN &= ~0x3FCU;
	LpcUsbFsEp0Reset(&s_Ep0);
	for (unsigned physical = 2U; physical < 10U; ++physical)
	{
		s_Memory.Command[physical * 2U] = LPC11_USB_DISABLED;
		s_Memory.Command[physical * 2U + 1U] = LPC11_USB_DISABLED;
	}
	LPC_USB->INTSTAT = 0x3FFU;
	Lpc11UsbCommand(0x7FU | LPC11_USB_NAK_CONTROL, 0U,
		(Status & (LPC11_USB_RESET | LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE)) |
		(clearSetup ? LPC11_USB_SETUP : 0U));
	if (!clearSetup && (LPC_USB->DEVCMDSTAT & LPC11_USB_SETUP) != 0U)
		LPC_USB->INTSETSTAT = 1U;
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_RESET;
	UsbDevProcessEvent(0, &event);
}

void USB_IRQHandler(void)
{
	const uint32_t state = DisableInterrupt();
	if (!s_Prepared || !s_Ep0.Enabled)
	{
		EnableInterrupt(state);
		return;
	}
	const uint32_t generation = s_Ep0.Generation;
	const uint32_t pending = LPC_USB->INTSTAT & LPC_USB->INTEN;
	const uint32_t status = LPC_USB->DEVCMDSTAT;
	if ((pending & LPC11_USB_DEVICE_INTERRUPT) != 0U)
	{
		LPC_USB->INTSTAT = LPC11_USB_DEVICE_INTERRUPT;
		if ((status & LPC11_USB_RESET) != 0U)
		{
			Lpc11UsbBusReset(status, s_Initialized && !UsbCtrlrVbusDetected(0));
			__DSB();
			EnableInterrupt(state);
			return;
		}
		if ((status & LPC11_USB_CONNECT_CHANGE) != 0U && s_Initialized)
		{
			// DCON_C records VBUS loss, even if the cable is back by now.
			if (!s_Detached)
				Lpc11UsbBusReset(status, !UsbCtrlrVbusDetected(0));
			else
				Lpc11UsbHwAckStatus(status & (LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE));
			UsbProcessQue(0);
			__DSB();
			EnableInterrupt(state);
			return;
		}
		Lpc11UsbHwAckStatus(status & (LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE));
		if ((status & (LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE)) != 0U)
			Lpc11UsbWakeCancel();
		if ((status & LPC11_USB_CONNECT_CHANGE) != 0U)
			UsbProcessQue(0);
		if (!s_Detached && (status & LPC11_USB_SUSPEND_CHANGE) != 0U)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & (1U << 17)) != 0U ?
				USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			UsbDevProcessEvent(0, &event);
		}
	}
	if (s_Prepared && s_Ep0.Enabled && !s_Detached && s_Ep0.Generation == generation &&
		(LPC_USB->DEVCMDSTAT & LPC11_USB_RESET) == 0U && (pending & 3U) != 0U)
		LpcUsbFsEp0Interrupt(&s_Ep0, pending);
	for (unsigned physical = 2U; physical < 10U && LpcUsbFsDataCurrent(); ++physical)
		LpcUsbFsDataInterrupt(physical);
	if (s_Prepared && s_Ep0.Enabled && !s_Detached && s_Ep0.Generation == generation &&
		(LPC_USB->DEVCMDSTAT & LPC11_USB_RESET) == 0U &&
		(LPC_USB->INTSTAT & LPC_USB->INTEN & LPC11_USB_SOF_INTERRUPT) != 0U)
	{
		LPC_USB->INTSTAT = LPC11_USB_SOF_INTERRUPT;
		Lpc11UsbWakeFrame();
		if (s_SofEnabled)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = USB_CTRLR_EVT_SOF;
			event.FrameNo = LPC_USB->INFO & 0x7FFU;
			UsbDevProcessEvent(0, &event);
		}
	}
	__DSB();
	EnableInterrupt(state);
}

static bool Lpc11UsbControlReady(void)
{
	return s_Prepared && !s_Closing && !s_Detached;
}

static void Lpc11UsbControlEvent(const UsbCtrlrEvt_t *pEvent)
{
	UsbDevProcessEvent(0, pEvent);
}

LpcUsbFsData_t *LpcUsbFsDataContext(void)
{
	return &s_Data;
}
