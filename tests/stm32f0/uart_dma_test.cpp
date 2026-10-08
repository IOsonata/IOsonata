// Register model for the production UART driver. This tests data ownership
// and IRQ/lifecycle behavior, not DMA bus timing or UART throughput.
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
#include <sys/mman.h>
#include "stm32f0xx.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
extern "C" void DMA1_Channel2_3_IRQHandler();
extern "C" void DMA1_Channel4_5_IRQHandler();
extern "C" void USART1_IRQHandler();
static int ready;
static int callback(UARTDEV *, UART_EVT event, uint8_t *, int) {
	if (event == UART_EVT_TXREADY) ++ready;
	return 0;
}
static void irq(unsigned shift) {
	DMA1->IFCR = 0;
	if (shift == 4) DMA1_Channel2_3_IRQHandler();
	else DMA1_Channel4_5_IRQHandler();
	// Model IFCR's write-one-to-clear behavior.
	if (DMA1->IFCR & (DMA_IFCR_CGIF1 << shift)) DMA1->ISR &= ~(15U << shift);
}
static void finish(DMA_Channel_TypeDef *ch, unsigned shift, std::vector<uint8_t> &out) {
	assert(ch->CCR & DMA_CCR_EN);
	assert((ch->CCR & (DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE)) ==
		(DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE));
	assert(!(ch->CCR & (DMA_CCR_CIRC | DMA_CCR_PINC | DMA_CCR_MSIZE | DMA_CCR_PSIZE)));
	assert(ch->CNDTR > 0 && ch->CNDTR <= 16);
	const auto *bytes = reinterpret_cast<const uint8_t *>(uintptr_t(ch->CMAR));
	out.insert(out.end(), bytes, bytes + ch->CNDTR);
	ch->CNDTR = 0;
	DMA1->ISR |= DMA_ISR_TCIF1 << shift;
	irq(shift);
	assert(DMA1->IFCR == (DMA_IFCR_CGIF1 << shift));
}
int main() {
	assert(mmap((void *)0x40000000, 0x30000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40000000);
	assert(mmap((void *)0x48000000, 0x2000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x48000000);
	RCC->CFGR = RCC_CFGR_SWS_PLL | RCC_CFGR_PLLMUL12;
	SystemCoreClockUpdate();
	IOPINCFG pins[] = {{0,10,18,IOPINDIR_INPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
		{0,9,18,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL}};
	UARTCFG cfg = {};
	cfg.pIOPinMap = pins; cfg.NbIOPins = 2; cfg.Rate = 1000000;
	cfg.DataBits = 8; cfg.StopBits = 1; cfg.Mode = UART_MODE_UART;
	cfg.bIntMode = true; cfg.bDMAMode = true; cfg.bFifoBlocking = true;
	cfg.EvtCallback = callback;
	UARTDEV uart = {}, other = {}, second = {};
	// Do not overwrite another DMA user's configured channel.
	DMA1_Channel2->CCR = DMA_CCR_MINC;
	assert(!UARTInit(&uart, &cfg));
	assert(DMA1_Channel2->CCR == DMA_CCR_MINC);
	DMA1_Channel2->CCR = 0;
	SYSCFG->CFGR1 = SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_TIM16_DMA_RMP;
	assert(UARTInit(&uart, &cfg));
	assert((SYSCFG->CFGR1 & SYSCFG_CFGR1_USART1TX_DMA_RMP) == 0);
	assert(SYSCFG->CFGR1 & SYSCFG_CFGR1_TIM16_DMA_RMP);
	assert(USART1->BRR == 48 && uart.DevIntrf.bDma);
	assert(!UARTInit(&other, &cfg));
	assert(!(USART1->CR3 & USART_CR3_DMAR));
	assert(USART1->CR1 & USART_CR1_RXNEIE);
	uint8_t b = 0xA5;
	assert(UARTTx(&uart, &b, 1) == 1);
	assert(DMA1_Channel2->CPAR == uintptr_t(&USART1->TDR));
	assert(USART1->CR3 & USART_CR3_DMAT);
	assert(uart.DevIntrf.SetRate(&uart.DevIntrf, 115200) == 0);
	assert(USART1->BRR == 48);
	assert(!(USART1->CR1 & USART_CR1_TXEIE));
	b = 0; // Caller storage may change immediately after Tx returns.
	std::vector<uint8_t> output;
	finish(DMA1_Channel2, 4, output);
	assert(output == std::vector<uint8_t>{0xA5} && ready == 1);
	assert(uart.bTxReady && !(DMA1_Channel2->CCR & DMA_CCR_EN));
	assert(USART1->CR3 & USART_CR3_DMAT); // Disabled channel gates idle requests.
	// Repeated isolated writes must rearm the same DMA addresses and never
	// repeat a completed byte when the FIFO is empty between calls.
	uint32_t cache = DMA1_Channel2->CMAR;
	output.clear();
	for (int i = 0; i < 256; ++i) {
		b = uint8_t(i);
		assert(UARTTx(&uart, &b, 1) == 1);
		b = 0;
		assert(DMA1_Channel2->CMAR == cache);
		finish(DMA1_Channel2, 4, output);
		assert(output.back() == uint8_t(i));
	}
	// Byte-at-a-time producer, repeated FIFO wrap and backpressure.
	output.clear(); std::vector<uint8_t> expected;
	for (int i = 0; i < 4096; ++i) {
		b = uint8_t(i * 73 + 19);
		while (UARTTx(&uart, &b, 1) == 0) finish(DMA1_Channel2, 4, output);
		expected.push_back(b); b = 0;
		if (i % 11 == 10) finish(DMA1_Channel2, 4, output);
	}
	while (!uart.bTxReady) finish(DMA1_Channel2, 4, output);
	assert(output == expected);
	// Drop-mode FIFO writes cannot overwrite the separate active DMA cache.
	cfg.bFifoBlocking = false;
	assert(UARTInit(&uart, &cfg));
	b = 0xB6; assert(UARTTx(&uart, &b, 1) == 1);
	for (int i = 0; i < 64; ++i) {
		b = uint8_t(i); assert(UARTTx(&uart, &b, 1) == 1);
	}
	output.clear(); finish(DMA1_Channel2, 4, output);
	assert(output.front() == 0xB6);
	while (!uart.bTxReady) finish(DMA1_Channel2, 4, output);
	assert(output.size() == 17);
	for (int i = 0; i < 16; ++i) assert(output[i + 1] == i + 48);
	cfg.bFifoBlocking = true;
	// Caller-supplied non-power-of-two FIFO; bursts larger than DMA cache.
	alignas(4) uint8_t txmem[CFIFO_MEMSIZE(37)];
	cfg.pTxMem = txmem; cfg.TxMemSize = sizeof(txmem);
	assert(UARTInit(&uart, &cfg));
	output.clear(); expected.resize(997);
	for (unsigned i = 0; i < expected.size(); ++i) expected[i] = uint8_t(i);
	for (unsigned i = 0; i < expected.size();) {
		int n = UARTTx(&uart, expected.data() + i, expected.size() - i);
		i += n;
		if (!uart.bTxReady) finish(DMA1_Channel2, 4, output);
	}
	while (!uart.bTxReady) finish(DMA1_Channel2, 4, output);
	assert(output == expected);
	// USART2 is independent; an unrelated channel's flag is not cleared.
	cfg.DevNo = 1; cfg.pTxMem = nullptr; cfg.TxMemSize = 0;
	assert(UARTInit(&second, &cfg));
	b = 0x37; assert(UARTTx(&second, &b, 1) == 1);
	assert(DMA1_Channel4->CPAR == uintptr_t(&USART2->TDR));
	DMA1->ISR = DMA_ISR_TCIF3; irq(4);
	assert(DMA1->IFCR == 0 && (DMA1->ISR & DMA_ISR_TCIF3));
	b = 0x48; assert(UARTTx(&uart, &b, 1) == 1);
	output.clear(); finish(DMA1_Channel4, 12, output);
	assert(output == std::vector<uint8_t>{0x37});
	assert(DMA1_Channel2->CCR & DMA_CCR_EN);
	output.clear(); finish(DMA1_Channel2, 4, output);
	assert(output == std::vector<uint8_t>{0x48});
	assert(DMA1->ISR & DMA_ISR_TCIF3); DMA1->ISR = 0;
	// RX remains live while TX DMA is active.
	b = 0x59; assert(UARTTx(&uart, &b, 1) == 1);
	USART1->RDR = 0x6A; USART1->ISR = USART_ISR_RXNE;
	USART1_IRQHandler(); USART1->ISR = 0;
	assert(UARTRx(&uart, &b, 1) == 1 && b == 0x6A);
	// Transfer error accounts for unsent bytes and permits queued work.
	b = 0x7B; assert(UARTTx(&uart, &b, 1) == 1);
	DMA1->ISR = DMA_ISR_TEIF2; irq(4);
	assert(uart.TxDropCnt == 1);
	output.clear(); finish(DMA1_Channel2, 4, output);
	assert(output == std::vector<uint8_t>{0x7B});
	// Disabling aborts DMA; enabling flushes queued data as in IRQ mode.
	b = 0x8C; assert(UARTTx(&uart, &b, 1) == 1);
	assert(UARTTx(&uart, &b, 1) == 1);
	uart.DevIntrf.Disable(&uart.DevIntrf);
	assert(DMA1_Channel2->CCR == 0 && !(USART1->CR3 & USART_CR3_DMAT));
	assert(UARTTx(&uart, &b, 1) == 0);
	DMA1->ISR = DMA_ISR_TCIF2; irq(4); // Late interrupt cannot restart TX.
	assert(DMA1_Channel2->CCR == 0); DMA1->ISR = 0;
	uart.DevIntrf.Enable(&uart.DevIntrf);
	assert(CFifoUsed(uart.hTxFifo) == 0);
	assert(UARTTx(&uart, &b, 1) == 1);
	uart.DevIntrf.Reset(&uart.DevIntrf);
	assert(DMA1_Channel2->CCR == 0 && CFifoUsed(uart.hTxFifo) == 0);
	cfg.DevNo = 0; assert(UARTInit(&uart, &cfg));
	assert(UARTTx(&uart, &b, 1) == 1);
	assert(UARTInit(&uart, &cfg)); // Reinit while DMA is active.
	assert(DMA1_Channel2->CCR == 0 && uart.bTxReady);
	assert(UARTTx(&uart, &b, 1) == 1);
	uart.DevIntrf.PowerOff(&uart.DevIntrf);
	assert(DMA1_Channel2->CCR == 0 && !(USART1->CR3 & USART_CR3_DMAT));
	assert(!(RCC->APB2ENR & RCC_APB2ENR_USART1EN));
	// Reinitializing into interrupt-only mode releases the DMA channel.
	cfg.bDMAMode = false; assert(UARTInit(&uart, &cfg));
	DMA1->ISR = DMA_ISR_TCIF2; irq(4);
	assert(DMA1->IFCR == 0);
	USART1->ISR = USART_ISR_TXE;
	assert(UARTTx(&uart, &b, 1) == 1 && USART1->TDR == b);
	assert(USART1->CR1 & USART_CR1_TXEIE);
}
