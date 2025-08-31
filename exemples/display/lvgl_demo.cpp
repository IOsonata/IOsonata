/**-------------------------------------------------------------------------
@example	lvgl_demo.cpp

@brief	lvgl.io graphics library integration demo


@author	Hoang Nguyen Hoan
@date	Mar. 9, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#include <stdio.h>

#include "lv_conf.h"
#include "lvgl.h"
#include "idelay.h"
#include "convutil.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "display/display_st77xx.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "iopinctrl.h"
#include "lv_demos.h"
#include "lv_examples.h"

#include "board.h"

static const IOPinCfg_t s_But[] = BUT_PINS;
static const int s_NbBut = sizeof(s_But) / sizeof(IOPinCfg_t);

void TimerHandler(TimerDev_t * const pTimer, uint32_t Evt);

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

Timer g_Timer;

static const IOPinCfg_t s_TFTCtrlPins[] = TFT_PINS;
const int s_NbTFTCtrlPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = SPI_RATE,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_TFTCtrlPins,
	.NbPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t),
	.Stride = 320 * 3,
	.Width = 240,
	.Height = 320,
	.PixelSize = 16,
	.Orient = DISPL_ORIENT_PORTRAIT,
};

//LcdST77xx g_Lcd;
LcdHX8357 g_Lcd;
//LcdILI9341 g_Lcd;

static lv_disp_drv_t s_LvglDriver;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LVBUFF_SIZE];
uint8_t g_LvglMem[LV_MEM_SIZE];
lv_disp_t *g_pDispl = NULL;
bool g_bLandscape = false;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	lv_tick_inc(LV_DISP_DEF_REFR_PERIOD);
    }
}

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    g_Lcd.BitBlt(area->x1, area->y1, w, h, ( uint8_t * )color_p );
    lv_disp_flush_ready( disp );
}

void HardwareInit()
{
	IOPinCfg(s_But, s_NbBut);

	g_Spi.Init(s_SpiCfg);

	g_Lcd.Init(s_LcdCfg, &g_Spi);
	g_Lcd.Backlight(true);

	g_Lcd.Clear();

	lv_init();
	lv_disp_draw_buf_init( &draw_buf, buf, NULL, LVBUFF_SIZE );

	lv_disp_drv_init( &s_LvglDriver );

	s_LvglDriver.hor_res = g_Lcd.Width();
	s_LvglDriver.ver_res = g_Lcd.Height();
	s_LvglDriver.flush_cb = my_disp_flush;
	s_LvglDriver.draw_buf = &draw_buf;
	g_pDispl = lv_disp_drv_register( &s_LvglDriver );

    g_Timer.Init(s_TimerCfg);

}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{
	HardwareInit();
	//lv_demo_widgets();
	//lv_example_get_started_1();
	//DisplayInit();
	//lv_img_set_src(&img, &logo);
	//lv_demo_stress();
	lv_demo_benchmark();
	//lv_example_anim_1();

	uint64_t period = g_Timer.EnableTimerTrigger(0, LV_DISP_DEF_REFR_PERIOD, TIMER_TRIG_TYPE_CONTINUOUS);

	int orient = (int)g_Lcd.Orientation();

	while (1) {
		__WFE();
		if (IOPinRead(BUT1_PORT, BUT1_PIN) == 0 || IOPinRead(BUT2_PORT, BUT2_PIN) == 0)
		{
			// Switch display mode portrait/landscape
			g_bLandscape = !g_bLandscape;

			g_Timer.DisableTimerTrigger(0);

			orient++;
			if (orient > DISPL_ORIENT_LANDSCAPE_INV)
			{
				orient = DISPL_ORIENT_PORTRAIT;
			}
			//g_Lcd.Orientation((DISPL_ORIENT)orient);
			g_Lcd.Rotate90();
#if 0
			if (g_bLandscape == true)
			{
				s_LcdCfg.Width = 320;
				s_LcdCfg.Height = 240;
				s_LcdCfg.Orient = DISPL_ORIENT_LANDSCAPE;
			}
			else
			{
				s_LcdCfg.Width = 240;
				s_LcdCfg.Height = 320;
				s_LcdCfg.Orient = DISPL_ORIENT_PORTRAIT;
			}
			g_Lcd.Init(s_LcdCfg, &g_Spi);
			g_Lcd.Backlight(true);
			g_Lcd.Clear();
#endif

			s_LvglDriver.hor_res = g_Lcd.Width();
			s_LvglDriver.ver_res = g_Lcd.Height();

			lv_disp_drv_update(g_pDispl, &s_LvglDriver);

			g_Timer.EnableTimerTrigger(0, LV_DISP_DEF_REFR_PERIOD, TIMER_TRIG_TYPE_CONTINUOUS);
			while(IOPinRead(BUT1_PORT, BUT1_PIN) == 0 ||  IOPinRead(BUT2_PORT, BUT2_PIN) == 0);
		}
		else
		{
			lv_task_handler();
		}
	}

	return 0;
}
