/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Setup services for the generic protocol examples, not a library interface.
 */
#ifndef __NET_DEMO_H__
#define __NET_DEMO_H__

#include "crypto/icrypto.h"

#define NET_DEMO_MEDIUM_LTE		1
#define NET_DEMO_MEDIUM_ETHERNET	2
#define NET_DEMO_MEDIUM_CUSTOM	3

// Prepare the console, event queue, timer and RNG. Does not start a network.
bool NetDemoInit();
// Start the selected medium and a deferred 20 ms foreground application pump.
// A failed medium initialization is retried after 30 seconds.
bool NetDemoStart(void (*pHandler)(uint32_t Now));
uint32_t NetDemoNow();
RngEngine *NetDemoRng();
bool NetDemoReady();

// Select and open the concrete interface outside the protocol application.
// Datagram=false supplies a connected byte stream; true supplies connected
// UDP semantics. The returned DeviceIntrf is caller-owned and MaxRetry=0.
// Open may block; read NetDemoNow() afterward before starting deadlines.
DeviceIntrf *NetDemoOpen(bool Datagram, const char *pHost, uint16_t Port);
bool NetDemoConnected();
void NetDemoClose();

// CUSTOM medium: provide these functions in an application source file.
// They handle connection setup only. All data still flows through the returned
// existing DeviceIntrf. A Wi-Fi implementation can be selected here without
// changing either protocol example. Called only from the foreground owner.
// Keep DeviceIntrf, its DevIntrf_t and callbacks valid through protocol reset,
// including after Close; closing releases endpoint resources, not that storage.
// Process must not silently replace an active endpoint. Latch a lost endpoint
// as disconnected until Close/Open, so the application starts a new session.
bool NetDemoCustomInit(TimerDev_t *pTimer);
void NetDemoCustomProcess(uint32_t Now);
bool NetDemoCustomReady();
DeviceIntrf *NetDemoCustomOpen(bool Datagram, const char *pHost, uint16_t Port);
bool NetDemoCustomConnected();
void NetDemoCustomClose();

#endif
