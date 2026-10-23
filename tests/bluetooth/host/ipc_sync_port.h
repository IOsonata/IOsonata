/**-------------------------------------------------------------------------
@file	ipc_sync_port.h

@brief	Synchronous IpcIntrf port for host tests

Raising a channel runs the receive work of the other side at once, in the
same thread. Include it in one source file of a test program: it defines the
three port functions. The race coverage of the rings is in tests/ipc.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#ifndef __IPC_SYNC_PORT_H__
#define __IPC_SYNC_PORT_H__

#include "ipc/ipc_intrf.h"

// Synchronous IPC port. A channel number is both the send and the receive
// channel of the pair, as in the nRF5340 port.
static IpcIntrfDev_t *s_pIpcByRx[16];

extern "C" bool IpcCtrlrInit(IpcIntrfDev_t * const pDev)
{
	s_pIpcByRx[pDev->RxChan] = pDev;

	return true;
}

extern "C" void IpcCtrlrSend(int Chan)
{
	if (s_pIpcByRx[Chan] != nullptr)
	{
		IpcIntrfIrqHandler(s_pIpcByRx[Chan]);
	}
}

extern "C" void IpcCtrlrRxEnable(IpcIntrfDev_t * const, bool)
{
}

#endif
