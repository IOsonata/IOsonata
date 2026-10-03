/**-------------------------------------------------------------------------
@file	app_run.cpp

@brief	Bare metal main loop

AppRun runs the application event queue and waits for the next interrupt when
the queue is empty. Every subsystem hands its deferred work to that queue, so
one loop serves all of them, first in first out.

@author	Hoang Nguyen Hoan
@date	Oct. 3, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include "app_evt_handler.h"

void AppRun(void)
{
	while (1)
	{
		// AppWait returns on any interrupt, including one that queued an event
		// after the queue was found empty.
		if (AppEvtHandlerExec() == false)
		{
			AppWait();
		}
	}
}
