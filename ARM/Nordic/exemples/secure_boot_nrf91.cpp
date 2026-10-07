/**-------------------------------------------------------------------------
@example	secure_boot_nrf91.cpp

@brief	nRF91 secure stage: start the non secure application.

Flash this image at 0 with the non secure application at the start of the
non secure flash of the TrustZone layout (tz_layout_nrf9160.ld,
tz_layout_nrf9120.ld). It gives the application its memory and peripherals
and starts it, see spe_nrf91.h. Link with nrf91xx_xxaa_spe.ld and a secure
configuration of the library; the application with nrf91xx_xxaa_ns.ld and a
non secure one.

With no valid application in flash the secure stage stays here.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include "nrf.h"

#include "spe_nrf91.h"

int main()
{
	// Keeps nothing secure besides what the SPU fixes
	(void)nRF91SpeStart(nullptr);

	// No valid application
	while (1)
	{
		__WFE();
	}

	return 0;
}
