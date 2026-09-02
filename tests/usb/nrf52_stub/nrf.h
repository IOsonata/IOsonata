#ifndef TESTS_USB_NRF52_STUB_NRF_H
#define TESTS_USB_NRF52_STUB_NRF_H

#include <stdint.h>

#define USBD_PRESENT 1

typedef int IRQn_Type;

enum
{
	USBD_IRQn = 42,
};

extern volatile bool g_UsbdIrqEnabled;
extern volatile bool g_UsbdIrqPending;

static inline void NVIC_EnableIRQ(IRQn_Type)
{
	g_UsbdIrqEnabled = true;
}

static inline void NVIC_DisableIRQ(IRQn_Type)
{
	g_UsbdIrqEnabled = false;
}

static inline void NVIC_SetPendingIRQ(IRQn_Type)
{
	g_UsbdIrqPending = true;
}

#define __ISB() ((void)0)
#define __DSB() ((void)0)

enum
{
	USBD_INTEN_USBRESET_Pos = 0,
	USBD_INTEN_ENDEPIN0_Pos = 1,
	USBD_INTEN_EP0DATADONE_Pos = 9,
	USBD_INTEN_ENDEPOUT0_Pos = 10,
	USBD_INTEN_USBEVENT_Pos = 18,
	USBD_INTEN_SOF_Pos = 19,
	USBD_INTEN_EP0SETUP_Pos = 20,
	USBD_INTEN_EPDATA_Pos = 21,
};

#define USBD_INTEN_USBRESET_Msk (1UL << USBD_INTEN_USBRESET_Pos)
#define USBD_INTEN_ENDEPIN0_Msk (1UL << USBD_INTEN_ENDEPIN0_Pos)
#define USBD_INTEN_EP0DATADONE_Msk (1UL << USBD_INTEN_EP0DATADONE_Pos)
#define USBD_INTEN_ENDEPOUT0_Msk (1UL << USBD_INTEN_ENDEPOUT0_Pos)
#define USBD_INTEN_USBEVENT_Msk (1UL << USBD_INTEN_USBEVENT_Pos)
#define USBD_INTEN_SOF_Msk (1UL << USBD_INTEN_SOF_Pos)
#define USBD_INTEN_EP0SETUP_Msk (1UL << USBD_INTEN_EP0SETUP_Pos)
#define USBD_INTEN_EPDATA_Msk (1UL << USBD_INTEN_EPDATA_Pos)
#define USBD_INTENSET_SOF_Msk USBD_INTEN_SOF_Msk
#define USBD_INTENCLR_SOF_Msk USBD_INTEN_SOF_Msk

#define USBD_EVENTCAUSE_SUSPEND_Msk (1UL << 0)
#define USBD_EVENTCAUSE_RESUME_Msk (1UL << 1)
#define USBD_EVENTCAUSE_USBWUALLOWED_Msk (1UL << 2)

#define USBD_LOWPOWER_LOWPOWER_Pos 0
#define USBD_LOWPOWER_LOWPOWER_ForceNormal 0
#define USBD_LOWPOWER_LOWPOWER_LowPower 1

#define USBD_DPDMVALUE_STATE_Resume 1

#define USBD_EPSTALL_STALL_Pos 8
#define USBD_EPSTALL_STALL_UnStall 0
#define USBD_EPSTALL_STALL_Stall 1

#define USBD_DTOGGLE_VALUE_Pos 8
#define USBD_DTOGGLE_VALUE_Data0 1

typedef struct
{
	volatile uint32_t PTR;
	volatile uint32_t MAXCNT;
	volatile uint32_t AMOUNT;
} TestUsbdDmaEp_t;

typedef struct
{
	volatile uint32_t EPOUT[8];
} TestUsbdSize_t;

typedef struct
{
	union
	{
		struct
		{
			volatile uint32_t EVENTS_USBRESET;
			volatile uint32_t EVENTS_ENDEPIN[8];
			volatile uint32_t EVENTS_EP0DATADONE;
			volatile uint32_t EVENTS_ENDEPOUT[8];
			volatile uint32_t EVENTS_USBEVENT;
			volatile uint32_t EVENTS_SOF;
			volatile uint32_t EVENTS_EP0SETUP;
			volatile uint32_t EVENTS_EPDATA;
		};
		volatile uint32_t EVENTS[USBD_INTEN_EPDATA_Pos + 1];
	};

	volatile uint32_t EVENTCAUSE;
	volatile uint32_t INTEN;
	volatile uint32_t INTENSET;
	volatile uint32_t INTENCLR;
	volatile uint32_t EPDATASTATUS;
	volatile uint32_t EPOUTEN;
	volatile uint32_t EPINEN;
	volatile uint32_t LOWPOWER;
	volatile uint32_t DPDMVALUE;
	volatile uint32_t DTOGGLE;
	volatile uint32_t EPSTALL;
	volatile uint32_t USBPULLUP;
	volatile uint32_t FRAMECNTR;

	volatile uint32_t BMREQUESTTYPE;
	volatile uint32_t BREQUEST;
	volatile uint32_t WVALUEL;
	volatile uint32_t WVALUEH;
	volatile uint32_t WINDEXL;
	volatile uint32_t WINDEXH;
	volatile uint32_t WLENGTHL;
	volatile uint32_t WLENGTHH;

	TestUsbdDmaEp_t EPIN[8];
	TestUsbdDmaEp_t EPOUT[8];
	TestUsbdSize_t SIZE;

	volatile uint32_t TASKS_STARTEPIN[8];
	volatile uint32_t TASKS_STARTEPOUT[8];
	volatile uint32_t TASKS_EP0STATUS;
	volatile uint32_t TASKS_EP0RCVOUT;
	volatile uint32_t TASKS_EP0STALL;
	volatile uint32_t TASKS_DPDMDRIVE;
} TestUsbd_t;

extern TestUsbd_t g_TestUsbd;

#define NRF_USBD (&g_TestUsbd)

#endif
