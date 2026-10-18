// STM32H753 register tests. Runs under tests/stm32h7/run.py, not on a board.
#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "iopinctrl.h"

#define CHECK(Expr) do { if (!(Expr)) { return __LINE__; } } while (0)

extern "C" int main(void)
{
	CHECK(SystemCoreClock == 400000000UL);
	CHECK(SystemD2Clock == 200000000UL);
	for (int i = 0; i < 4; i++)
	{
		CHECK(SystemPeriphClockGet(i) == 100000000UL);
		CHECK(SystemPeriphClockSet(i, 24000000UL) == 25000000UL);
		CHECK(SystemPeriphClockSet(i, 0) == 12500000UL);
		CHECK(SystemPeriphClockSet(i, UINT32_MAX) == 100000000UL);
	}
	CHECK(SystemPeriphClockGet(-1) == 0);
	CHECK(SystemPeriphClockSet(4, 1) == 0);

	static const IOPinCfg_t pins[] = {
		{ IOPORTD, 9, IOPINOP_FUNC7, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
		{ IOPORTD, 8, IOPINOP_FUNC7, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
	};
	UARTCfg_t cfg = {};
	cfg.DevNo = 3;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 1000000;
	cfg.DataBits = 8;
	cfg.Parity = UART_PARITY_NONE;
	cfg.StopBits = 1;
	cfg.FlowControl = UART_FLWCTRL_NONE;
	cfg.IntPrio = 6;
	cfg.bIntMode = true;
	cfg.bFifoBlocking = true;
	static UARTDev_t uart;
	RCC->D2CCIP2R = RCC_D2CCIP2R_USART28SEL;
	CHECK(UARTInit(&uart, &cfg));
	CHECK(USART3->BRR == 100);
	CHECK((RCC->D2CCIP2R & RCC_D2CCIP2R_USART28SEL) == 0);
	CHECK(RCC->APB1LENR & RCC_APB1LENR_USART3EN);
	CHECK(RCC->AHB4ENR & RCC_AHB4ENR_GPIODEN);
	CHECK((GPIOD->AFR[1] & 0xFFU) == 0x77U);
	CHECK((GPIOD->MODER & (15U << 16)) == (10U << 16));
	CHECK(USART3->CR1 & USART_CR1_RXNEIE_RXFNEIE);
	cfg.bDMAMode = true;
	CHECK(UARTInit(&uart, &cfg) == false);

	TimerCfg_t tc = {};
	tc.DevNo = 8; // TIM2
	tc.Freq = 100000;
	tc.IntPrio = 6;
	tc.ClkSrc = TIMER_CLKSRC_DEFAULT;
	static TimerDev_t timer;
	CHECK(TimerGetLowFreqDevCount() == 0);
	CHECK(TimerGetHighFreqDevCount() == 14);
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 1999);
	// APB /1: HCLK is still half the CPU clock; do not double it.
	RCC->D2CFGR &= ~RCC_D2CFGR_D2PPRE1;
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 1999);
	// APB /8 with TIMPRE=1: kernel is four times PCLK, 100 MHz.
	RCC->D2CFGR |= 6U << RCC_D2CFGR_D2PPRE1_Pos;
	RCC->CFGR |= RCC_CFGR_TIMPRE;
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 999);
	// Every advertised TIM maps to the correct owned vector.
	for (int i = 0; i < TimerGetHighFreqDevCount(); i++)
	{
		tc.DevNo = i;
		CHECK(TimerInit(&timer, &tc));
	}
	return 0;
}
