/**-------------------------------------------------------------------------
@example	tinyusb_cdc_prbs_tx/main.cpp

@brief	TinyUSB CDC PRBS transmit performance comparison.

This benchmark uses the same PRBS producer and BYTE_MODE switch as
usb_cdc_prbs_tx.cpp. Only the USB stack calls differ, so byte mode and buffered
mode exercise the same application workload for the IOsonata/TinyUSB
comparison.

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

#include "prbs.h"


#define BYTE_MODE

#define TEST_BUFSIZE			16

#define TINYUSB_BENCH_PID		0x0002U
#define TINYUSB_BENCH_PRODUCT	"TinyUSB CDC PRBS Tx"
#include "../tinyusb_common/tinyusb_cdc_bench.h"

int main()
{
	uint8_t d = 0xff;
#ifndef BYTE_MODE
	uint8_t buff[TEST_BUFSIZE];
#endif

	if (!TinyUsbBenchInit())
	{
		return -1;
	}

	while (1)
	{
		TinyUsbBenchPowerProcess();
		tud_task_ext(0, false);

		if (tud_cdc_connected() == false)
		{
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte. The value advances only when the octet
		// was accepted into the FIFO. If the FIFO is full, retry this same byte
		// while TinyUSB makes room.
		if (tud_cdc_write_available() > 0U &&
			tud_cdc_write(&d, 1U) > 0U)
		{
			d = Prbs8(d);
		}
#else
		// Demo transfer buffer
		for (int i = 0; i < TEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}

		int len = TEST_BUFSIZE;
		uint8_t *p = buff;

		while (len > 0)
		{
			uint32_t l = tud_cdc_write(p, static_cast<uint32_t>(len));

			len -= static_cast<int>(l);
			p += l;

			if (tud_cdc_connected() == false)
			{
				break;
			}

			if (len > 0)
			{
				TinyUsbBenchPowerProcess();
				tud_task_ext(0, false);
			}
		}
#endif
	}

	return 0;
}
