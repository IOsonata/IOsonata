/**-------------------------------------------------------------------------
@example	eink_display_demo.cpp

@brief	Blinky example

This example shows how to use GPIO to
- Blink LED
- Detect button
- Generate a pulse train. Pulse moving from 1 GPIO to the next

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <stdbool.h>

extern "C" {
#include "nrf_lcd.h"
#include "nrf_gfx.h"
}

#include "coredev/iopincfg.h"
#include "coredev/spi.h"
#include "display/eink_vb3300.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "pulse_train.h"

#include "paper.h"
#include "board.h"

static const IOPinCfg_t s_DispPins[] = {
	{EINK_DC_PORT, EINK_DC_PIN, EINK_DC_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{EINK_BUSY_PORT, EINK_BUSY_PIN, EINK_BUSY_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{EINK_RESET_PORT, EINK_RESET_PIN, EINK_RESET_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{EINK_BSI_PORT, EINK_BSI_PIN, EINK_BSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{EINK_SCK_PORT, EINK_SCK_PIN, EINK_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{EINK_SDA_PORT, EINK_SDA_PIN, EINK_SDA_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{EINK_CS_PORT, EINK_CS_PIN, EINK_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_NbDispPins = sizeof(s_DispPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_SpiPins[] = {
    {EINK_SCK_PORT, EINK_SCK_PIN, EINK_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// SCK
   // {EINK_SDA_PORT, EINK_SDA_PIN, EINK_SDA_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
    {-1, -1, -1, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// MISO
    {EINK_SDA_PORT, EINK_SDA_PIN, EINK_SDA_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
    {EINK_CS_PORT, EINK_CS_PIN, EINK_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// CS
};

static const SPICFG s_SpiCfg = {
	.DevNo = 0,
	.Phy = SPIPHY_NORMAL,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = 4000000,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

const EInkIntrfCfg_t s_EInkIntrfCfg = {
	.Type = EIINTRF_TYPE_BITBANG,
	.pIOPinMap = s_DispPins,
	.NbIOPins = s_NbDispPins,
	.pSpiDev = g_Spi,
	.SpiCsIdx = 0,
};

EInkIntrf g_EInkIntrf;

#ifdef QTD
static const IOPinCfg_t s_LedPins[] = {
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);
#endif

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
extern "C" void EPD_Init( void );
extern "C" void EPD_Display_Black( void );
extern "C" void EPD_Display_White( void );

extern const nrf_gfx_font_desc_t orkney_8ptFontInfo;

static lcd_cb_t lcd_cb = {
 .state    = NRFX_DRV_STATE_UNINITIALIZED,
 .height   = 480,//PAPER_PIXEL_HEIGHT,
 .width    = 280,//PAPER_PIXEL_WIDTH,
 .rotation = NRF_LCD_ROTATE_90
};

static const nrf_lcd_t lcd = {
  .lcd_init = paper_init,
  .lcd_uninit = paper_uninit,
  .lcd_pixel_draw = paper_pixel_draw,
  .lcd_rect_draw = paper_rect_draw,
  .lcd_display = paper_display,
  .lcd_rotation_set = paper_rotation_set,
  .lcd_display_invert = paper_display_invert,
  .p_lcd_cb = &lcd_cb
};

extern "C" void spi_9b_send_9b( uint16_t dat )
{
    if (dat & 0x100)
    {
//    	g_EInkIntrf.SetDataMode(true);
    	EInkIntrfSetDataMode(g_EInkIntrf, true);
//        spi_9b_send( DCX_DATA, (uint8_t)dat );
    }
    else
    {
    	//g_EInkIntrf.SetDataMode(false);
    	EInkIntrfSetDataMode(g_EInkIntrf, false);

//        spi_9b_send( DCX_CMD, (uint8_t)dat );
    }
	//g_EInkIntrf.Tx(0, (uint8_t*)&dat, 1);
    EInkIntrfTx(g_EInkIntrf, (uint8_t*)&dat, 1);
}

extern "C" void spi_9b_send( uint16_t dcx, uint8_t dat )
{
    if (dcx)
    {
    	//g_EInkIntrf.SetDataMode(true);
    	EInkIntrfSetDataMode(g_EInkIntrf, true);

//        spi_9b_send( DCX_DATA, (uint8_t)dat );
    }
    else
    {
//    	g_EInkIntrf.SetDataMode(false);
    	EInkIntrfSetDataMode(g_EInkIntrf, false);
//        spi_9b_send( DCX_CMD, (uint8_t)dat );
    }
//	g_EInkIntrf.Tx(0, (uint8_t*)&dat, 1);
    EInkIntrfTx(g_EInkIntrf, &dat, 1);
}

int main()
{
	// Configure
 	//IOPinCfg(s_DispPins, s_NbDispPins);
	//IOPinSet(EINK_DC_PORT, EINK_DC_PIN);
#ifdef QTD
	IOPinCfg(s_LedPins, s_NbLedPins);
	IOPinClear(LED1_PORT, LED1_PIN);
	IOPinClear(LED2_PORT, LED2_PIN);
	IOPinClear(LED3_PORT, LED3_PIN);
	IOPinClear(LED4_PORT, LED4_PIN);

	msDelay(250);

	IOPinConfig(0, 30, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, 30);
	IOPinSet(LED1_PORT, LED1_PIN);
#endif

	//g_Spi.Init(s_SpiCfg);

	g_EInkIntrf.Init(s_EInkIntrfCfg);
#ifdef QTD
	msDelay(250);
	IOPinSet(LED2_PORT, LED2_PIN);
#endif
	EPD_Init();
	msDelay(2000);

	nrf_gfx_init(&lcd);
#ifdef QTD
	IOPinSet(LED3_PORT, LED3_PIN);
#endif
	EPD_Display_Black();
	// draw a horizontal line
	nrf_gfx_line_t hline = NRF_GFX_LINE(0, nrf_gfx_height_get(&lcd) / 2,
										nrf_gfx_width_get(&lcd),
										nrf_gfx_height_get(&lcd) / 2, 4);
	nrf_gfx_line_draw(&lcd, &hline, 1);

	// draw a vertical line
	nrf_gfx_line_t vline = NRF_GFX_LINE(nrf_gfx_width_get(&lcd) / 2, 0,
										nrf_gfx_width_get(&lcd) / 2,
										nrf_gfx_height_get(&lcd), 4);
	nrf_gfx_line_draw(&lcd, &vline, 1);

	nrf_gfx_circle_t cir = {
		nrf_gfx_width_get(&lcd) / 2,
		nrf_gfx_height_get(&lcd) / 2,
		nrf_gfx_width_get(&lcd) / 2
	};
	nrf_gfx_circle_draw(&lcd, &cir, 1, 0);
	nrf_gfx_display(&lcd);

#ifdef QTD
	IOPinSet(LED4_PORT, LED4_PIN);
#endif
	while (1) __WFE();

	return 0;
}

#define PAPER_BYTE_WIDTH ((PAPER_PIXEL_WIDTH % 8 == 0) ? PAPER_PIXEL_WIDTH / 8 : PAPER_PIXEL_WIDTH / 8 + 1)
#define PAPER_BYTE_HEIGHT PAPER_PIXEL_HEIGHT
#define PAPER_BUFLEN PAPER_BYTE_WIDTH * PAPER_BYTE_HEIGHT

extern uint8_t paper_buffer[PAPER_BUFLEN];
extern "C" void  check_busy_high();
#if 0
extern "C" ret_code_t _paper_display(void)
{
  ret_code_t ret = 0;

  /*
   * TODO transfer chunks of 256 bytes rather than calling nrf_drv_spi_transfer
   * for each byte.
   */

  uint8_t command[1];
#if 0
  command[0] = 0x10;
  ret = paper_tx_cmd(command, 1);
  //if (ret != NRF_SUCCESS) return ret;

  //paper_begin_data();
  for (int i = 0; i < PAPER_BUFLEN; i++) {
  	uint16_t d = 0x100 | ~paper_buffer[i];//paper_prev_buffer[i];
  	 spi_9b_send_9b(d);

   // ret = nrf_drv_spi_transfer(&spi, paper_prev_buffer + i, 1, NULL, 0);
    if (ret != NRF_SUCCESS) return ret;
  }

  msDelay(10);
#endif
  spi_9b_send_9b( 0x10 );

  EInkIntrfSetDataMode(g_EInkIntrf, true);

  uint8_t d = 0;
  for (int i = 0; i < PAPER_BUFLEN; i++)
  {
  	EInkIntrfTx(g_EInkIntrf, &d, 1);
  }

  /*
   * TODO i'm not yet sure about the purpose of sending the previous buffer too.
   * just sending 0x00s works pretty good. when sending the actual previous
   * image buffer, it looks like sections that have not changed become kinda
   * dirty / greyish (which could be useful because one can use three colors).
   */

  // memcpy(paper_prev_buffer, paper_buffer, PAPER_BUFLEN);
#if 0
  command[0] = 0x13;
  ret = paper_tx_cmd(command, 1);
  if (ret != NRF_SUCCESS) return ret;

  //paper_begin_data();
  for (int i = 0; i < PAPER_BUFLEN; i++) {
	  	uint16_t d = 0x100 | paper_buffer[i];
	  	 spi_9b_send_9b(d);
   // ret = nrf_drv_spi_transfer(&spi, paper_buffer + i, 1, NULL, 0);
    if (ret != NRF_SUCCESS) return ret;
  }
#endif
  //spi_9b_send_9b( 0x13 );
  //EInkIntrfSetDataMode(g_EInkIntrf, true);

  //EInkIntrfTx(g_EInkIntrf, paper_buffer, PAPER_BUFLEN);
  uint8_t cmd = 0x13;

  EInkIntrfWrite(g_EInkIntrf, &cmd, 1, paper_buffer, PAPER_BUFLEN);

  msDelay(10);

  //ret = paper_tx_lut();
  //if (ret != NRF_SUCCESS) return ret;

  //spi_9b_send_9b( DRF );
  //check_busy_high();
  spi_9b_send_9b( 0x12);//DRF );
  check_busy_high();

//  spi_9b_send_9b( POF );
//  check_busy_low();
//  ret = paper_tx_cmd(paper_cmd_disp, sizeof(paper_cmd_disp));
//  if (ret != NRF_SUCCESS) return ret;

  msDelay(100); // TODO use busy pin rather than arbitrary delays

  return ret;
}
#endif
