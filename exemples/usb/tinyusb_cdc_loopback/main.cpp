/**-------------------------------------------------------------------------
@example	tinyusb_cdc_loopback/main.cpp

@brief	TinyUSB CDC loopback performance comparison.

This benchmark mirrors usb_cdc_loopback.cpp but uses TinyUSB directly. It
reads at most one full-speed bulk packet at a time and queues the same bytes
back to the CDC IN endpoint.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <string.h>

#define TINYUSB_BENCH_PID		0x0001U
#define TINYUSB_BENCH_PRODUCT	"TinyUSB CDC Loopback"
#include "../tinyusb_common/tinyusb_cdc_bench.h"

static const char s_Banner[] = "\r\nTinyUSB USB CDC Loopback\r\n";

extern "C" void tud_cdc_line_state_cb(uint8_t Instance, bool Dtr, bool Rts)
{
	(void)Rts;

	if (Instance == 0U && Dtr)
	{
		(void)tud_cdc_write(s_Banner, sizeof(s_Banner) - 1U);
		(void)tud_cdc_write_flush();
	}
}

int main()
{
	uint8_t buff[TINYUSB_BENCH_DATA_MPS];
	uint32_t pending = 0;
	uint32_t offset = 0;

	if (!TinyUsbBenchInit())
	{
		return -1;
	}

	while (1)
	{
		TinyUsbBenchPowerProcess();
		tud_task_ext(0, false);

		if (pending > 0U)
		{
			const uint32_t n =
				tud_cdc_write(&buff[offset], pending);

			offset += n;
			pending -= n;

			if (pending == 0U)
			{
				(void)tud_cdc_write_flush();
			}
			continue;
		}

		if (!tud_cdc_connected())
		{
			continue;
		}

		const uint32_t available = tud_cdc_available();
		if (available == 0U)
		{
			continue;
		}

		pending = tud_cdc_read(buff, sizeof(buff));
		offset = 0;
	}

	return 0;
}
