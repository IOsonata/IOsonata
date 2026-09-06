/**-------------------------------------------------------------------------
@example	tinyusb_cdc_prbs_tx/main.cpp

@brief	TinyUSB CDC PRBS transmit performance comparison.

This benchmark mirrors usb_cdc_prbs_tx.cpp but uses TinyUSB directly. The
application offers one PRBS byte at a time. TinyUSB's CDC FIFO and completion
path decide when endpoint packets are submitted.

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

#define TINYUSB_BENCH_PID		0x0002U
#define TINYUSB_BENCH_PRODUCT	"TinyUSB CDC PRBS Tx"
#include "../tinyusb_common/tinyusb_cdc_bench.h"

int main()
{
	uint8_t d = 0xff;

	if (!TinyUsbBenchInit())
	{
		return -1;
	}

	while (1)
	{
		TinyUsbBenchPowerProcess();
		tud_task_ext(0, false);

		if (!tud_cdc_connected() || tud_cdc_write_available() == 0U)
		{
			continue;
		}

		if (tud_cdc_write(&d, 1U) == 1U)
		{
			d = Prbs8(d);
		}
	}

	return 0;
}
