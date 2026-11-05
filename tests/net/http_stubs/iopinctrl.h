/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Pin control of the host run of exemples/net/http_server.cpp.
 */
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include "coredev/iopincfg.h"

extern int g_LedToggles;

static inline void IOPinToggle(int PortNo, int PinNo)
{
	(void)PortNo;
	(void)PinNo;
	g_LedToggles++;
}

#endif
