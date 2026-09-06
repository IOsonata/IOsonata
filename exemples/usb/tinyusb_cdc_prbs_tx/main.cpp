/**-------------------------------------------------------------------------
@example	tinyusb_cdc_prbs_tx/main.cpp

@brief	TinyUSB CDC PRBS transmit performance comparison.

This benchmark mirrors usb_cdc_prbs_tx.cpp but uses TinyUSB directly.
TINYUSB_BENCH_TX_BLOCK_SIZE selects how many PRBS bytes are offered to TinyUSB
per write call. A value of 1 matches the IOsonata one-byte benchmark. A value
of 64 measures TinyUSB with one full-speed bulk packet per application write.

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

#ifndef TINYUSB_BENCH_TX_BLOCK_SIZE
#define TINYUSB_BENCH_TX_BLOCK_SIZE	64U
#endif

#if TINYUSB_BENCH_TX_BLOCK_SIZE == 0 || TINYUSB_BENCH_TX_BLOCK_SIZE > 64
#error TINYUSB_BENCH_TX_BLOCK_SIZE must be between 1 and 64
#endif

#define TINYUSB_BENCH_PID		0x0002U
#define TINYUSB_BENCH_PRODUCT	"TinyUSB CDC PRBS Tx"
#include "../tinyusb_common/tinyusb_cdc_bench.h"

int main()
{
	uint8_t d = 0xff;
	uint8_t tx[TINYUSB_BENCH_TX_BLOCK_SIZE];

	if (!TinyUsbBenchInit())
	{
		return -1;
	}

	while (1)
	{
		TinyUsbBenchPowerProcess();
		tud_task_ext(0, false);

		if (!tud_cdc_connected() ||
			tud_cdc_write_available() < TINYUSB_BENCH_TX_BLOCK_SIZE)
		{
			continue;
		}

		uint8_t next = d;
		for (unsigned i = 0; i < TINYUSB_BENCH_TX_BLOCK_SIZE; i++)
		{
			tx[i] = next;
			next = Prbs8(next);
		}

		const uint32_t count =
			tud_cdc_write(tx, TINYUSB_BENCH_TX_BLOCK_SIZE);

		if (count == TINYUSB_BENCH_TX_BLOCK_SIZE)
		{
			d = next;
		}
		else if (count > 0U)
		{
			d = tx[count];
		}
	}

	return 0;
}
