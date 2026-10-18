// STM32L152RE register checks under tests/stm32l1/run.py.
#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "timer_stm32.h"
#include "iopinctrl.h"

#define CHECK(Expr) do { if (!(Expr)) { return __LINE__; } } while (0)

static void PinEvent(int IntNo, void *pContext)
{
	(void)IntNo;
	(void)pContext;
}

extern "C" int main(void)
{
	CHECK(SystemCoreClock == 32000000UL);
	CHECK((PWR->CR & PWR_CR_VOS) == PWR_CR_VOS_0);
	CHECK(FLASH->ACR & FLASH_ACR_LATENCY);
	CHECK(FLASH->ACR & FLASH_ACR_ACC64);
	for (int i = 0; i < 2; i++)
	{
		CHECK(SystemPeriphClockGet(i) == 32000000UL);
		CHECK(SystemPeriphClockSet(i, 7000000UL) == 8000000UL);
		CHECK(SystemPeriphClockSet(i, 0) == 2000000UL);
		CHECK(SystemPeriphClockSet(i, UINT32_MAX) == 32000000UL);
	}
	CHECK(SystemPeriphClockGet(-1) == 0);
	CHECK(SystemPeriphClockSet(2, 1) == 0);

	// L1 GPIOH uses bit 5, not its alphabetical port index 7.
	IOPinConfig(IOPORTH, 0, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	CHECK(RCC->AHBENR & RCC_AHBENR_GPIOHEN);
	IOPinConfig(IOPORTA, 5, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	CHECK((GPIOA->MODER & (3U << 10)) == (1U << 10));
	CHECK(IOPinEnableInterrupt(2, 6, IOPORTF, 2, IOPINSENSE_LOW_TRANSITION, PinEvent, nullptr));
	CHECK((SYSCFG->EXTICR[0] & 0xF00U) == 0x600U);
	CHECK(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN);
	CHECK(EXTI->FTSR & (1U << 2));
	IOPinDisableInterrupt(2);
	static const IOPinCfg_t pins[] = {
		{ IOPORTA, 3, IOPINOP_FUNC7, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
		{ IOPORTA, 2, IOPINOP_FUNC7, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
	};
	UARTCfg_t cfg = {};
	cfg.DevNo = 1;
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
	CHECK(UARTInit(&uart, &cfg));
	CHECK(USART2->BRR == 32);
	CHECK(RCC->APB1ENR & RCC_APB1ENR_USART2EN);
	CHECK(RCC->AHBENR & RCC_AHBENR_GPIOAEN);
	CHECK((GPIOA->AFR[0] & 0xFF00U) == 0x7700U);
	CHECK((GPIOA->MODER & (15U << 4)) == (10U << 4));
	CHECK(USART2->CR1 & USART_CR1_RXNEIE);
	cfg.bDMAMode = true;
	CHECK(UARTInit(&uart, &cfg) == false);

	TimerCfg_t tc = {};
	tc.DevNo = 5;
	tc.Freq = 100000;
	tc.IntPrio = 6;
	tc.ClkSrc = TIMER_CLKSRC_DEFAULT;
	static TimerDev_t timer;
	CHECK(TimerGetLowFreqDevCount() == 0);
	CHECK(TimerGetHighFreqDevCount() == 9);
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 319);
	CHECK(SystemPeriphClockSet(0, 16000000) == 16000000);
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 319);
	CHECK(SystemPeriphClockSet(0, 8000000) == 8000000);
	CHECK(TimerInit(&timer, &tc));
	CHECK(TIM2->PSC == 159);
	static const int channels[] = {1, 1, 1, 1, 2, 4, 4, 4, 4};
	for (int i = 0; i < TimerGetHighFreqDevCount(); i++)
	{
		tc.DevNo = i;
		CHECK(TimerInit(&timer, &tc));
		CHECK(TimerGetMaxTrigger(&timer) == channels[i]);
	}
	return 0;
}
