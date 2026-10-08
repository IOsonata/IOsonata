// Host register model: real ST peripheral definitions, no hardware timing.
#include <cassert>
#include <cstring>
#include <sys/mman.h>
#include "stm32f0xx.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
extern "C" void USART1_IRQHandler();
#if defined(STM32F030xC)
extern "C" void USART3_6_IRQHandler();
#elif defined(STM32F070xB)
extern "C" void USART3_4_IRQHandler();
#endif
static void pin_event(int, void *) {}
static void map(unsigned long addr, unsigned long len) {
	assert(mmap((void *)addr, len, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)addr);
}
int main() {
	map(0x40000000, 0x30000); map(0x48000000, 0x2000);
	RCC->CFGR = RCC_CFGR_SWS_PLL | RCC_CFGR_PLLMUL12;
	SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 48000000);
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2 | RCC_CFGR_PPRE_DIV2;
	SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 24000000);
	assert(SystemPeriphClockGet(0) == 12000000);
	RCC->CFGR = 0; SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 8000000);
	IOPINCFG pins[] = {{0,10,18,IOPINDIR_INPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
		{0,9,18,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL}};
	UARTCFG cfg = {}; cfg.pIOPinMap = pins; cfg.NbIOPins = 2;
	cfg.Rate = 115200; cfg.DataBits = 8; cfg.StopBits = 1;
	cfg.Mode = UART_MODE_UART; cfg.Parity = UART_PARITY_NONE; cfg.bIntMode = true;
	UARTDEV dev = {}; assert(UARTInit(&dev, &cfg));
	USART1->RDR = 0xA5; USART1->ISR = USART_ISR_RXNE;
	USART1_IRQHandler(); USART1->ISR = 0;
	uint8_t byte = 0; assert(dev.DevIntrf.RxData(&dev.DevIntrf, &byte, 1) == 1);
	assert(byte == 0xA5); assert(dev.DevIntrf.RxData(&dev.DevIntrf, &byte, 1) == 0);
	USART1->RDR = 0xFF; USART1->ISR = USART_ISR_RXNE | USART_ISR_FE;
	USART1_IRQHandler(); USART1->ISR = 0;
	assert(dev.DevIntrf.RxData(&dev.DevIntrf, &byte, 1) == 0);
	// A temporarily unavailable TDR must still arm the queued transfer.
	byte = 0x34; assert(dev.DevIntrf.TxData(&dev.DevIntrf, &byte, 1) == 1);
	assert(USART1->CR1 & USART_CR1_TXEIE);
	USART1->ISR = USART_ISR_TXE; USART1_IRQHandler(); assert(USART1->TDR == byte);
	USART1_IRQHandler(); assert(!(USART1->CR1 & USART_CR1_TXEIE));
	cfg.Parity = UART_PARITY_EVEN; cfg.StopBits = 2; cfg.FlowControl = UART_FLWCTRL_HW;
	assert(UARTInit(&dev, &cfg)); assert(USART1->CR1 & USART_CR1_M);
	assert((USART1->CR1 & (USART_CR1_PCE | USART_CR1_PS)) == USART_CR1_PCE);
	assert((USART1->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1);
	assert(!(USART1->CR3 & USART_CR3_CTSIE));
	cfg.Parity = UART_PARITY_ODD; assert(UARTInit(&dev, &cfg));
	assert(USART1->CR1 & USART_CR1_PS);
	cfg.DataBits = 7; assert(UARTInit(&dev, &cfg)); assert(!(USART1->CR1 & USART_CR1_M));
	cfg.DataBits = 9; assert(!UARTInit(&dev, &cfg)); cfg.DataBits = 8;
	cfg.bDMAMode = true; assert(!UARTInit(&dev, &cfg)); cfg.bDMAMode = false;
	cfg.bIntMode = false; cfg.Parity = UART_PARITY_NONE; assert(UARTInit(&dev, &cfg));
	assert(!(USART1->CR3 & USART_CR3_EIE));
	USART1->ISR = 0; assert(dev.DevIntrf.TxData(&dev.DevIntrf, &byte, 1) == 0);
	assert(dev.DevIntrf.RxData(&dev.DevIntrf, &byte, 1) == 0);
	USART1->ISR = USART_ISR_TXE; assert(dev.DevIntrf.TxData(&dev.DevIntrf, &byte, 1) == 1);
	USART1->ISR = USART_ISR_RXNE; USART1->RDR = 0x63;
	assert(dev.DevIntrf.RxData(&dev.DevIntrf, &byte, 1) == 1 && byte == 0x63);
#if defined(STM32F030xC) || defined(STM32F070xB)
	UARTDEV shared = {}; cfg.DevNo = 2; cfg.bIntMode = true;
	assert(UARTInit(&shared, &cfg)); USART3->RDR = 0x42; USART3->ISR = USART_ISR_RXNE;
#ifdef STM32F030xC
	USART3_6_IRQHandler();
#else
	USART3_4_IRQHandler();
#endif
	USART3->ISR = 0;
	assert(shared.DevIntrf.RxData(&shared.DevIntrf, &byte, 1) == 1 && byte == 0x42);
#endif
	assert(IOPinAllocateInterrupt(1,0,13,IOPINSENSE_TOGGLE,pin_event,nullptr) == 13);
	assert(IOPinAllocateInterrupt(1,1,13,IOPINSENSE_TOGGLE,pin_event,nullptr) < 0);
	IOPinDisableInterrupt(13); IOPinDisableInterrupt(13);
	assert(IOPinAllocateInterrupt(1,1,13,IOPINSENSE_TOGGLE,pin_event,nullptr) == 13);
	assert(!IOPinEnableInterrupt(2,1,0,13,IOPINSENSE_TOGGLE,pin_event,nullptr));
}
