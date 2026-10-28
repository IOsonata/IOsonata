/* LPC54628 generic USB integration model. MIT License, I-SYST, 2026. */
#define LPC546_USB_INTEGRATION
#define LPC546_USB_EP0_FIXTURE_ONLY
#include "ep0546_test.c"
#include "lpc_iap.h"

// VBUS pin of the controller under test
#ifndef TEST_VBUS_PORT
#define TEST_VBUS_PORT 0
#define TEST_VBUS_PIN 22
#define TEST_VBUS_OP IOPINOP_FUNC7
#endif

static IOPinEvtHandler_t s_VbusHandler;
static void *s_VbusContext;

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	*TestIrqEnabled(Irq) = true;
}

void IOPinConfig(int Port, int Pin, int Op, IOPINDIR Dir, IOPINRES Res, IOPINTYPE Type)
{
	assert(Port == TEST_VBUS_PORT && Pin == TEST_VBUS_PIN && Op == TEST_VBUS_OP);
	assert(Dir == IOPINDIR_INPUT && Res == IOPINRES_NONE && Type == IOPINTYPE_NORMAL);
}

int IOPinAllocateInterrupt(int Priority, int Port, int Pin, IOPINSENSE Sense,
	IOPinEvtHandler_t Handler, void *pContext)
{
	assert(Priority >= 0 && Priority < (1 << __NVIC_PRIO_BITS));
	assert(Port == TEST_VBUS_PORT && Pin == TEST_VBUS_PIN && Sense == IOPINSENSE_TOGGLE);
	assert(pContext != NULL);
	s_VbusHandler = Handler;
	s_VbusContext = pContext;
	return 5;
}

void IOPinDisableInterrupt(int IntNo)
{
	assert(IntNo == 5);
	s_VbusHandler = NULL;
}

IAPSTATUS IAPReadUID(IAP_CMD_STATUS *pResult)
{
	assert(SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_ROM_MASK);
	assert((SYSCON->PDRUNCFG[0] & (SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK)) == 0);
	for (int i = 0; i < 4; i++)
		pResult->Data[i] = 0x12345678U + i;
	return IAPSTATUS_SUCCESS;
}

void TestUsbInitialize(void)
{
	MapAll();
	assert(mmap((void *)GPIO_BASE, sizeof(*GPIO), PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	Reset();
	s_Memory = (LpcUsbFsMemory_t *)0x40100000;
	s_AutoSkip = true;
	GPIO->B[TEST_VBUS_PORT][TEST_VBUS_PIN] = 1;
}

LpcUsbFsMemory_t *TestUsbMemory(void)
{
	return s_Memory;
}

void TestUsbPacketDone(unsigned Physical, unsigned Length)
{
	const unsigned capacity = (s_Memory->Command[2 * Physical] >> 16) & 0x3FF;
	assert((s_Memory->Command[2 * Physical] & ACTIVE) != 0 && Length <= capacity);
	s_Memory->Command[2 * Physical] &= ~(ACTIVE | (0x3FFU << 16));
	s_Memory->Command[2 * Physical] |= (capacity - Length) << 16;
	USB0->INTSTAT |= 1U << Physical;
	USB0_IRQHandler();
}

void TestUsbStatusAckPending(unsigned Physical)
{
	assert(Physical < 2U);
	assert((s_Memory->Command[2 * Physical] & ACTIVE) != 0U);
	assert(((s_Memory->Command[2 * Physical] >> 16) & 0x3FFU) == 0U);
	s_Memory->Command[2 * Physical] &= ~ACTIVE;
	USB0->INTSTAT |= 1U << Physical;
}

void TestUsbSetup(const UsbSetupData_t *pSetup)
{
	memcpy(s_Memory->Setup, pSetup, sizeof(*pSetup));
	USB0->DEVCMDSTAT |= SETUP;
	USB0->INTSTAT |= 1U;
	USB0_IRQHandler();
}

void TestUsbCable(bool Connected, bool DeviceInterrupt)
{
	GPIO->B[TEST_VBUS_PORT][TEST_VBUS_PIN] = Connected ? 1 : 0;
	if (DeviceInterrupt)
	{
		TEST_USB->DEVCMDSTAT |= 1U << 24;
		TEST_USB->INTSTAT |= DEVICE | ((1U << TEST_USB_PHYSICAL) - 1U);
		TEST_USB_IRQHandler();
	}
}

void TestUsbVbusInterrupt(void)
{
	assert(s_VbusHandler != NULL);
	s_VbusHandler(5, s_VbusContext);
}

void TestUsbSuspend(bool Suspended)
{
	if (Suspended)
		USB0->DEVCMDSTAT |= USB_DEVCMDSTAT_DSUS_MASK;
	else
		USB0->DEVCMDSTAT &= ~USB_DEVCMDSTAT_DSUS_MASK;
	USB0->DEVCMDSTAT |= USB_DEVCMDSTAT_DSUS_C_MASK;
	USB0->INTSTAT |= DEVICE;
	USB0_IRQHandler();
}

void TestUsbFrame(void)
{
	USB0->INTSTAT |= USB_INTSTAT_FRAME_INT_MASK;
	USB0_IRQHandler();
}

unsigned TestUsbWakeCount(void)
{
	return s_WakeCount;
}
