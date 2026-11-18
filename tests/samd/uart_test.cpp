// UART mode selection and FIFO/DMA lifecycle model; not serial-line timing.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/shared_intrf.h"
#define _UINT32_(x) ((uint32_t)(x))
#define _UINT16_(x) ((uint32_t)(x))
#define _UINT8_(x) ((uint32_t)(x))
#define __I volatile
#define __O volatile
#define __IO volatile
#include "component/sercom.h"
#include "component/dmac.h"
#define SAMD_SERCOM_CNT 3
#define SAMD_GCLK_MAIN 0
#define SAMD_DMAC_COUNT_MAX 65535
#define SAMD_DMAC_TRIG_SERCOM_TX(n) (2 + 2 * (n))
typedef int IRQn_Type;
typedef void (*SamdDmacHandler_t)(int, uint32_t, void *);
static sercom_registers_t s_Regs[3];
static IrqHandler_t s_Handler;
static SamdDmacHandler_t s_DmaHandler;
static void *s_DmaCtx;
static bool s_Channel;
static bool s_Busy;
static unsigned s_Opens;
static unsigned s_Closes;
static unsigned s_Starts;
static const IOPinCfg_t s_Pins[] = {{0, 3}, {0, 2}};
static void NVIC_DisableIRQ(int);
static void NVIC_EnableIRQ(int);
static void NVIC_ClearPendingIRQ(int);
static void NVIC_SetPriority(int, uint32_t);
static int SamdSercomIrq(int);
static uint32_t SamdIrqPrio(int);
static bool SamdSercomOwned(int, const DevIntrf_t *);
static bool SamdSercomClockEnable(int);
static void SamdSercomClockDisable(int);
static sercom_registers_t *SamdSercomGetReg(int);
static int SamdSercomPad(int, const IOPinCfg_t *);
static uint32_t SystemPeriphClockGet(int);
static int SamdDmacOpen(int, SamdDmacHandler_t, void *, int);
static void SamdDmacClose(int);
static uint32_t SamdDmacStop(int);
static bool SamdDmacBusy(int);
static void SamdDmacStart(int, const volatile void *, bool, volatile void *, bool, uint32_t);
#include "uart_driver.h"

static void NVIC_DisableIRQ(int) {}
static void NVIC_EnableIRQ(int) {}
static void NVIC_ClearPendingIRQ(int) {}
static void NVIC_SetPriority(int, uint32_t) {}
static int SamdSercomIrq(int DevNo) { return DevNo; }
static uint32_t SamdIrqPrio(int Prio) { return Prio > 3 ? 3 : Prio; }
static bool SamdSercomOwned(int, const DevIntrf_t *) { return false; }
static bool SamdSercomClockEnable(int) { return true; }
static void SamdSercomClockDisable(int) {}
static sercom_registers_t *SamdSercomGetReg(int DevNo) { return &s_Regs[DevNo]; }
static int SamdSercomPad(int, const IOPinCfg_t *pPin) { return pPin->PinNo; }
static uint32_t SystemPeriphClockGet(int) { return 48000000; }
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinDisable(int, int) {}
void SharedIntrfSetIrqHandler(int, DevIntrf_t *, IrqHandler_t Handler) { s_Handler = Handler; }
static int SamdDmacOpen(int, SamdDmacHandler_t Handler, void *Ctx, int)
{
	assert(!s_Channel);
	s_Channel = true;
	s_DmaHandler = Handler;
	s_DmaCtx = Ctx;
	s_Opens++;
	return 0;
}
static void SamdDmacClose(int Chan)
{
	assert(Chan == 0 && s_Channel);
	s_Channel = false;
	s_Busy = false;
	s_Closes++;
}
static uint32_t SamdDmacStop(int Chan)
{
	assert(Chan == 0 && s_Channel);
	s_Busy = false;
	return 0;
}
static bool SamdDmacBusy(int Chan)
{
	assert(Chan == 0 && s_Channel);
	return s_Busy;
}
static void SamdDmacStart(int Chan, const volatile void *, bool, volatile void *, bool, uint32_t Count)
{
	assert(Chan == 0 && s_Channel && Count == 4);
	s_Busy = true;
	s_Starts++;
}
int main()
{
	UARTDev_t uart = {};
	UARTCfg_t cfg = {
		.DevNo = 2, .pIOPinMap = s_Pins, .NbIOPins = 2, .Rate = 1000000,
		.DataBits = 8, .Parity = UART_PARITY_NONE, .StopBits = 1,
		.FlowControl = UART_FLWCTRL_NONE, .IntPrio = 1, .bFifoBlocking = true
	};
	uint8_t data[] = {1, 2, 3, 4};
	uint8_t received[2];
	auto &reg = s_Regs[2].USART_INT;
	for (int mode : {3, 0, 2, 1})
	{
		cfg.bDMAMode = (mode & 2) != 0;
		cfg.bIntMode = (mode & 1) != 0;
		assert(UARTInit(&uart, &cfg));
		assert(s_Channel == cfg.bDMAMode);
		assert(s_SamdUartDev[2].pDmaOps == (cfg.bDMAMode ? &s_SamdUartDmaOps : nullptr));
		assert((s_SamdUartDev[2].Poll == nullptr) == cfg.bIntMode);
		assert(uart.Rate == 1000000);
		reg.SERCOM_INTFLAG = cfg.bDMAMode ? 0 : SERCOM_USART_INT_INTFLAG_DRE_Msk;
		assert(uart.DevIntrf.TxData(&uart.DevIntrf, data, sizeof(data)) == 4);
		if (cfg.bDMAMode)
		{
			assert(s_Busy);
			s_Busy = false;
			reg.SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_DRE_Msk;
			if (cfg.bIntMode)
			{
				s_DmaHandler(0, DMAC_CHINTFLAG_TCMPL_Msk, s_DmaCtx);
			}
			else
			{
				assert(uart.DevIntrf.RxData(&uart.DevIntrf, received, 1) == 0);
			}
		}
		else if (cfg.bIntMode)
		{
			for (int i = 0; i < 4; i++)
			{
				s_Handler(2, &uart.DevIntrf);
			}
		}
		assert(CFifoUsed(uart.hTxFifo) == 0 && uart.bTxReady);
		assert(CFifoPutByte(uart.hRxFifo, 0x31));
		assert(CFifoPutByte(uart.hRxFifo, 0x32));
		assert(uart.DevIntrf.RxData(&uart.DevIntrf, received, 2) == 2);
		assert(received[0] == 0x31 && received[1] == 0x32);
		uart.DevIntrf.Disable(&uart.DevIntrf);
		uart.DevIntrf.Enable(&uart.DevIntrf);
		assert(CFifoUsed(uart.hTxFifo) == 0);
	}
	cfg.bDMAMode = true;
	assert(UARTInit(&uart, &cfg));
	uart.DevIntrf.PowerOff(&uart.DevIntrf);
	assert(!s_Channel);
	assert(uart.DevIntrf.TxData(&uart.DevIntrf, data, sizeof(data)) == 0);
	assert(s_Opens == s_Closes && s_Starts == 2);
	puts("SAMD UART DMA/poll selection, FIFO transfers and lifecycle: PASS");
}
