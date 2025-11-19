/**-------------------------------------------------------------------------
@example	PdmMicDemo.cpp

@brief	PDM microphone example for nRF52


@author	Hoang Nguyen Hoan
@date	Dev. 9, 2020

@license

Copyright (c) 2020, I-SYST inc., all rights reserved

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
#include "nrf.h"
#include "nrfx_pdm.h"

#include "converters/analog_comp.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"

// This include contain i/o definition the board in use
#include "board.h"

#define NB_SAMPLES	64

//#define ADC_DEMO_INTERRUPT_ENABLE

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void AnaCompHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt);

#define FIFOSIZE			CFIFO_MEMSIZE(10*1024)

uint8_t g_TxBuff[FIFOSIZE];

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

static const AnalogCompCfg_t s_ACompCfg = {
	.DevNo = 0,
	.Mode = ANALOG_COMP_MODE_SINGLE,
	.RefSrc = 0,
	.RefVolt = 3000,
	.CompVolt = 1000,
	.AnalogIn = 0,
	.bHystersys = true,
	.IntPrio = 6,
	.EvtHandler = AnaCompHandler,
};

AnalogComp g_AComp;

volatile bool g_bDataReady = true;

void AnaCompHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt)
{
	if (Evt == ANALOG_COMP_EVT_LOWER)
	{
		g_bDataReady = true;
		//IOPinClear(0, 29);
		//msDelay(50);
		//IOPinSet(0, 29);
	}
	else if (Evt == ANALOG_COMP_EVT_HIGHER)
	{
		g_bDataReady = false;
		//IOPinClear(0, 30);
		//msDelay(50);
		//IOPinSet(0, 30);
	}
	//g_AComp.Start();
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	//g_Uart.Init(s_UartCfg);
//	UARTRetargetEnable(g_Uart, STDIN_FILENO);
//	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
	//IOPinSet(0, 2);
	//IOPinConfig(0, 2, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	//IOPinConfig(0, 3, 0, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	IOPinDisable(0, 2);
	IOPinDisable(0, 3);
//	IOPinConfig(0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	//IOPinSet(0, 3);

	IOPinSet(0, 30);
	IOPinConfig(0, 30, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinClear(0, 30);
	msDelay(500);
	IOPinSet(0, 30);

	IOPinSet(0, 29);
	IOPinConfig(0, 29, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	//printf("Init ADC\r\n");

	g_AComp.Init(s_ACompCfg);
	//g_AComp.Enable();
}

bool flag =0;
alignas(4) uint16_t buff1[1024];
alignas(4) uint16_t buff2[1024];
alignas(4) uint8_t buff3[10 * 1024];
volatile bool WriteFlag = 0;
volatile int g_CurIdx = 0;
volatile bool FullFlag = false;

static void drv_pdm_hand(const nrfx_pdm_evt_t *evt){
	//interupt++;
	//if((*evt).error != 0){int hhh = 1;} //(*evt).error, evt->error

	if (evt->error)
	{
		printf("%x\n", evt->error);
	}
#if 1
	if (evt->buffer_released != NULL)
	{
//		g_Uart.Tx((uint8_t*)evt->buffer_released, NB_SAMPLES * 2);

		memcpy(&buff3[g_CurIdx], evt->buffer_released, NB_SAMPLES * 2);
		g_CurIdx += NB_SAMPLES * 2;
		if (g_CurIdx >= 10240)
		{
			g_CurIdx = 0;
			FullFlag = true;
			//nrfx_pdm_stop();
			//g_Uart.Tx(buff3, 10240);
		}
	}
#endif
	if((*evt).buffer_requested){
		if(!flag){
			int32_t error = nrfx_pdm_buffer_set((int16_t*)&buff1[0], NB_SAMPLES);
			flag = 1;
			WriteFlag = 1;}
		else{
			int32_t error = nrfx_pdm_buffer_set((int16_t*)&buff2[0], NB_SAMPLES);
			flag = 0;
			WriteFlag = 1;
		}

	}
}
#define DISPLAY_POWER_PORT		0
#define DISPLAY_POWER_PIN		4	// P0.04
#define DISPLAY_POWER_PINOP		0

#define NRFX_PDM_CONFIG(_pin_clk, _pin_din)                   \
{                                                                     \
    .mode               = (nrf_pdm_mode_t)NRFX_PDM_CONFIG_MODE,       \
    .edge               = (nrf_pdm_edge_t)NRFX_PDM_CONFIG_EDGE,       \
    .pin_clk            = _pin_clk,                                   \
    .pin_din            = _pin_din,                                   \
    .clock_freq         = (nrf_pdm_freq_t)NRFX_PDM_CONFIG_CLOCK_FREQ, \
    .gain_l             = 0,                       \
    .gain_r             = 0,                       \
    .interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY                \
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	bool st = g_bDataReady;

	IOPinConfig(DISPLAY_POWER_PORT, DISPLAY_POWER_PIN, DISPLAY_POWER_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(DISPLAY_POWER_PORT, DISPLAY_POWER_PIN);

	g_Uart.Init(s_UartCfg);

	memset(buff1, 0, 1024);
	memset(buff2, 0, 1024);

	//HardwareInit();

	uint32_t error = 0;
	nrfx_pdm_config_t config1 = NRFX_PDM_CONFIG(0x0 | 24, 0x0 | 25);
	error = nrfx_pdm_init(&config1, drv_pdm_hand);
	error = nrfx_pdm_start();

	int cnt = 10;

	while (1)
	{
		__WFE();
		//if (--cnt <= 0)
		{
#if 1
			if (flag)
			{
				for (int i = 0; i < 16; i++)
				{
//					g_Uart.printf("%04x ", buff1[0]);//, buff2[0]);
					printf("%04x ", buff2[i]);//, buff2[0]);
				}
				//g_Uart.Tx((uint8_t*)buff1, NB_SAMPLES * 2);
			}
			else
#endif
			{
//				for (int i = 0; i < 1024; i++)
				{
					//g_Uart.printf("%04x ", buff3[1]);//, buff2[0]);
				//	g_Uart.Tx((uint8_t*)buff2, NB_SAMPLES * 2);
				}
			}
			printf("\n");
			cnt = 10;
		}
	}

	return 0;
}

//extern "C" void PDM_IRQHandler()
//{
//	nrfx_pdm_irq_handler();
//}
