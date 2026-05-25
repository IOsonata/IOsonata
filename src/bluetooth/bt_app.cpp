/**-------------------------------------------------------------------------
@file	bt_app.cpp

@brief	Generic Bluetooth application firmware type.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

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

#include "bluetooth/bt_host.h"
#include "bluetooth/bt_app.h"

// g_BtAppData is defined in each port's bt_app_<port>.cpp.
// This file is kept as a stub; once it's added to all port builds and the
// duplicate BtAppInit/BtAppRun stubs are reconciled, the definition can move
// here. For step 1 the definition lives in the port file that's actually
// compiled into the binary.

static BtHostDev_t s_BtHostDev;

bool BtAppInit(const BtHostCfg_t *pCfg)
{
	BtHostInit(&s_BtHostDev, pCfg);

	return true;
}

void BtAppRun()
{
	if (g_BtAppData.bInitialized == false)
	{
		return;
	}

	if (g_BtAppData.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		BtDevAdvStart();
	}

}
