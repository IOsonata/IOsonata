/**-------------------------------------------------------------------------
@example	spi_hal_reference.cpp

@brief	Isolated ST HAL reference for H753 SPI diagnostics.

@author	Hoang Nguyen Hoan
@date	July 21, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

// Isolated ST HAL reference. Does not instantiate the IOsonata SPI driver.
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"
#include "iopinctrl.h"
#include "stm32h7xx_hal.h"

#define FIFOSIZE			CFIFO_MEMSIZE(256)

typedef struct {
	uint32_t Cycles;
	uint32_t SlaveSR;
	uint32_t SlaveCR1;
	uint32_t SlaveIER;
	uint32_t MasterSR;
	uint32_t MasterCR1;
	uint16_t TxLeft;
	uint16_t RxLeft;
	uint16_t MasterTxLeft;
	uint16_t MasterRxLeft;
	unsigned Event;
	unsigned NSS;
} SpiTrace_t;

static SpiTrace_t s_Trace[32];
static volatile bool s_TraceEnabled;
static volatile bool s_TraceFrozen;
static unsigned s_TraceCount;
static uint32_t s_TraceStart;
static uint32_t s_ArmedSR;
static unsigned s_ArmedTxLeft;
static unsigned s_ArmedRxLeft;

uint8_t g_UarTxBuff[FIFOSIZE];

// Assign UART pins
static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	UART_DEVNO,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	NULL,//nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_UarTxBuff,
};

UART g_Uart;



static SPI_HandleTypeDef s_Master;
static SPI_HandleTypeDef s_Slave;
static uint8_t s_MasterTx[129];
static uint8_t s_MasterRx[129];
static uint8_t s_SlaveTx[129];
static uint8_t s_SlaveRx[129];
static volatile unsigned s_Complete;
static volatile uint32_t s_Error;
static uint32_t s_TickCycles;
static uint32_t s_TickRemainder;
static uint32_t s_TickMs;
static const IOPinCfg_t s_Pins[] = {
	{IOPORTA, 5, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 6, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{IOPORTB, 5, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTD, 14, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTC, 10, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTC, 11, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTC, 12, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 15, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

extern "C" uint32_t HAL_GetTick(void)
{
	uint32_t now = DWT->CYCCNT;
	uint32_t delta = now - s_TickCycles;
	uint32_t period = SystemCoreClock / 1000U;

	s_TickCycles = now;
	s_TickMs += delta / period;
	s_TickRemainder += delta % period;
	s_TickMs += s_TickRemainder / period;
	s_TickRemainder %= period;
	return s_TickMs;
}

// Diagnostic replay only. Never read RXDR or clear a peripheral flag here.
static void Trace(unsigned Event)
{
	uint32_t mask;
	SpiTrace_t *entry;

	if (!s_TraceEnabled)
	{
		return;
	}
	mask = __get_PRIMASK();
	__disable_irq();
	if (!s_TraceFrozen)
	{
		entry = &s_Trace[s_TraceCount % 32U];
		entry->Cycles = DWT->CYCCNT - s_TraceStart;
		entry->SlaveSR = SPI3->SR;
		entry->SlaveCR1 = SPI3->CR1;
		entry->SlaveIER = SPI3->IER;
		entry->MasterSR = SPI1->SR;
		entry->MasterCR1 = SPI1->CR1;
		entry->TxLeft = s_Slave.TxXferCount;
		entry->RxLeft = s_Slave.RxXferCount;
		entry->MasterTxLeft = s_Master.TxXferCount;
		entry->MasterRxLeft = s_Master.RxXferCount;
		entry->Event = Event;
		entry->NSS = (GPIOA->IDR >> 15) & 1U;
		s_TraceCount++;
		if (entry->SlaveSR & (SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_TIFRE))
		{
			s_TraceFrozen = true;
		}
	}
	__set_PRIMASK(mask);
}

static void PrintTrace(void)
{
	unsigned i;
	unsigned count = s_TraceCount;
	const SpiTrace_t *entry;

	printf("Armed before CS: slave SR=%08lx TX=%u RX=%u\r\n",
		(unsigned long)s_ArmedSR, s_ArmedTxLeft, s_ArmedRxLeft);
	printf("Trace events=%u; 1=armed 2=IRQ entry 3=IRQ exit 4=master returned\r\n", count);
	printf("cycles are core cycles since arm; TX/RX are HAL items remaining\r\n");
	for (i = count > 32U ? count - 32U : 0U; i < count; i++)
	{
		entry = &s_Trace[i % 32U];
		printf("%u ev=%u cyc=%lu NSS=%u slave SR=%08lx CR1=%08lx IER=%08lx TX=%u RX=%u master SR=%08lx CR1=%08lx TX=%u RX=%u\r\n",
			i, entry->Event, (unsigned long)entry->Cycles, entry->NSS,
			(unsigned long)entry->SlaveSR, (unsigned long)entry->SlaveCR1,
			(unsigned long)entry->SlaveIER, entry->TxLeft, entry->RxLeft,
			(unsigned long)entry->MasterSR, (unsigned long)entry->MasterCR1,
			entry->MasterTxLeft, entry->MasterRxLeft);
	}
}

extern "C" void SPI3_IRQHandler(void)
{
	if (s_TraceEnabled)
	{
		Trace(2);
		HAL_SPI_IRQHandler(&s_Slave);
		Trace(3);
		return;
	}
	HAL_SPI_IRQHandler(&s_Slave);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *Handle)
{
	if (Handle == &s_Slave)
	{
		s_Complete = s_Complete + 1U;
	}
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *Handle)
{
	s_Error = Handle->ErrorCode;
}

static bool Init(unsigned Mode, uint32_t Rate)
{
	unsigned i;

	NVIC_DisableIRQ(SPI3_IRQn);
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->APB1LENR |= RCC_APB1LENR_SPI3EN;
	(void)RCC->APB1LENR;
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB1LRSTR |= RCC_APB1LRSTR_SPI3RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	RCC->APB1LRSTR &= ~RCC_APB1LRSTR_SPI3RST;
	RCC->D2CCIP1R = (RCC->D2CCIP1R & ~RCC_D2CCIP1R_SPI123SEL) | RCC_D2CCIP1R_SPI123SEL_2;
	// Same HSI64 PER clock as the IOsonata port. Refuse other clock setups.
	if ((RCC->D1CCIPR & RCC_D1CCIPR_CKPERSEL) ||
		!(RCC->CR & RCC_CR_HSIRDY) || (RCC->CR & RCC_CR_HSIDIV))
	{
		return false;
	}
	IOPinSet(IOPORTD, 14);
	IOPinCfg(s_Pins, sizeof(s_Pins) / sizeof(s_Pins[0]));
	for (i = 0; i < sizeof(s_Pins) / sizeof(s_Pins[0]); i++)
	{
		IOPinSetSpeed(s_Pins[i].PortNo, s_Pins[i].PinNo, IOPINSPEED_TURBO);
	}
	IOPinSetSpeed(IOPORTA, 5, Rate == 1000000U ? IOPINSPEED_LOW : IOPINSPEED_MEDIUM);
	memset(&s_Master, 0, sizeof(s_Master));
	memset(&s_Slave, 0, sizeof(s_Slave));
	s_Master.Instance = SPI1;
	s_Master.Init.Mode = SPI_MODE_MASTER;
	s_Master.Init.Direction = SPI_DIRECTION_2LINES;
	s_Master.Init.DataSize = SPI_DATASIZE_8BIT;
	s_Master.Init.CLKPolarity = (Mode & 2U) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
	s_Master.Init.CLKPhase = (Mode & 1U) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
	s_Master.Init.NSS = SPI_NSS_SOFT;
	s_Master.Init.BaudRatePrescaler = Rate == 1000000U ? SPI_BAUDRATEPRESCALER_64 : SPI_BAUDRATEPRESCALER_8;
	s_Master.Init.FirstBit = SPI_FIRSTBIT_MSB;
	s_Master.Init.TIMode = SPI_TIMODE_DISABLE;
	s_Master.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	s_Master.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	s_Master.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
	s_Slave.Init = s_Master.Init;
	s_Slave.Instance = SPI3;
	s_Slave.Init.Mode = SPI_MODE_SLAVE;
	s_Slave.Init.NSS = SPI_NSS_HARD_INPUT;
	s_Slave.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	if (HAL_SPI_Init(&s_Master) != HAL_OK || HAL_SPI_Init(&s_Slave) != HAL_OK)
	{
		return false;
	}
	NVIC_ClearPendingIRQ(SPI3_IRQn);
	NVIC_SetPriority(SPI3_IRQn, 6);
	NVIC_EnableIRQ(SPI3_IRQn);
	return true;
}

static bool Frame(unsigned Length, uint8_t Seed, bool Receive)
{
	unsigned i;
	uint32_t start;
	HAL_StatusTypeDef arm;
	HAL_StatusTypeDef status;
	bool pass;
	int mismatch = -1;

	for (i = 0; i < sizeof(s_MasterTx); i++)
	{
		s_MasterTx[i] = Receive ? 0xFF : (uint8_t)(Seed + i);
		s_SlaveTx[i] = (uint8_t)(0xA0 ^ i);
		s_MasterRx[i] = 0;
		s_SlaveRx[i] = 0;
	}
	s_Complete = 0;
	s_Error = 0;
	s_TraceCount = 0;
	s_TraceFrozen = false;
	s_TraceStart = DWT->CYCCNT;
	s_TickCycles = DWT->CYCCNT;
	arm = HAL_SPI_TransmitReceive_IT(&s_Slave, s_SlaveTx, s_SlaveRx, Length);
	if (arm != HAL_OK)
	{
		printf("HAL slave arm failed status=%u error=%08lx\r\n", (unsigned)arm, (unsigned long)s_Slave.ErrorCode);
		return false;
	}
	if (s_TraceEnabled)
	{
		s_ArmedSR = SPI3->SR;
		s_ArmedTxLeft = s_Slave.TxXferCount;
		s_ArmedRxLeft = s_Slave.RxXferCount;
		Trace(1);
	}
	IOPinClear(IOPORTD, 14);
	status = HAL_SPI_TransmitReceive(&s_Master, s_MasterTx, s_MasterRx, Length, 250);
	if (s_TraceEnabled)
	{
		Trace(4);
	}
	IOPinSet(IOPORTD, 14);
	start = HAL_GetTick();
	while (!s_Complete && !s_Error && HAL_GetTick() - start < 250U)
	{
		__NOP();
	}
	pass = status == HAL_OK && s_Complete == 1U && s_Error == 0U;
	for (i = 0; i < Length; i++)
	{
		if (s_SlaveRx[i] != s_MasterTx[i] || (Receive && s_MasterRx[i] != s_SlaveTx[i]))
		{
			pass = false;
			if (mismatch < 0)
			{
				mismatch = i;
			}
		}
	}
	if (s_TraceEnabled)
	{
		// Freeze before UART output, including a timeout without an error IRQ.
		s_TraceFrozen = true;
		__DSB();
	}
	printf("HAL %s %u seed=%02x status=%u complete=%u master_err=%08lx slave_err=%08lx mismatch=%d %s\r\n",
		Receive ? "RX" : "TX", Length, Seed, (unsigned)status, s_Complete,
		(unsigned long)s_Master.ErrorCode, (unsigned long)s_Slave.ErrorCode, mismatch, pass ? "PASS" : "FAIL");
	if (!pass)
	{
		printf("Slave received:");
		for (i = 0; i < Length; i++)
		{
			printf(" %02x", s_SlaveRx[i]);
		}
		printf("\r\n");
		if (s_TraceEnabled)
		{
			PrintTrace();
			printf("Master received:");
			for (i = 0; i < Length; i++)
			{
				printf(" %02x", s_MasterRx[i]);
			}
			printf("\r\n");
		}
	}
	return pass;
}

int main(void)
{
	const unsigned lengths[] = {1, 16, 129};
	const uint32_t rates[] = {1000000U, 8000000U};
	unsigned r;
	unsigned mode;
	unsigned n;
	unsigned passed = 0;
	unsigned failed = 0;
	unsigned failedModes = 0;
	uint32_t cycles;
	bool pass;

	g_Uart.Init(s_UartCfg);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	cycles = DWT->CYCCNT;
	for (n = 0; n < 100U; n++)
	{
		__NOP();
	}
	if (DWT->CYCCNT == cycles)
	{
		printf("Reference clock counter unavailable\r\n");
		while (1)
		{
			__WFE();
		}
	}
	s_TickCycles = DWT->CYCCNT;
	printf("ST HAL SPI reference; UART 1000000 8N1; core=%lu\r\n", (unsigned long)SystemCoreClock);
	printf("%s\r\n", SPI_LOOPBACK_WIRING);
	printf("Fixed-length HAL slave; comparison only, not IOsonata validation\r\n");
	for (r = 0; r < 2; r++)
	{
		for (mode = 0; mode < 4; mode++)
		{
			pass = Init(mode, rates[r]);
			printf("HAL mode=%u rate=%lu SCK_OSPEED=%lu\r\n", mode, (unsigned long)rates[r],
				(unsigned long)((GPIOA->OSPEEDR >> 10) & 3U));
			for (n = 0; pass && n < 3; n++)
			{
				pass = Frame(lengths[n], 0x30, false) && Frame(lengths[n], 0x50, false) && Frame(lengths[n], 0, true);
				if (pass && lengths[n] > 1)
				{
					pass = Frame(1, 0x70, false) && Frame(lengths[n], 0x90, false);
				}
			}
			if (pass)
			{
				passed++;
			}
			else
			{
				failed++;
				failedModes |= 1U << (r * 4U + mode);
			}
		}
	}
	printf("HAL reference: %u passed, %u failed\r\n", passed, failed);
	// Keep the reference result above separate from instrumented replays.
	s_TraceEnabled = true;
	for (r = 0; r < 2; r++)
	{
		for (mode = 0; mode < 4; mode++)
		{
			if (!(failedModes & (1U << (r * 4U + mode))))
			{
				continue;
			}
			printf("Diagnostic replay mode=%u rate=%lu\r\n", mode, (unsigned long)rates[r]);
			pass = Init(mode, rates[r]);
			for (n = 0; pass && n < 3; n++)
			{
				pass = Frame(lengths[n], 0x30, false) && Frame(lengths[n], 0x50, false) && Frame(lengths[n], 0, true);
				if (pass && lengths[n] > 1)
				{
					pass = Frame(1, 0x70, false) && Frame(lengths[n], 0x90, false);
				}
			}
			printf("Diagnostic replay %s (instrumented, not a validation result)\r\n", pass ? "PASS" : "FAIL");
		}
	}
	while (1)
	{
		__WFE();
	}
}
