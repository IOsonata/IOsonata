/* Just enough of nrf.h for the IpcIntrf test firmware on the host. */
#ifndef HOST_NRF_H
#define HOST_NRF_H

#include <stdint.h>

#define __WFE()		((void)0)

typedef struct {
	struct { volatile uint32_t FORCEOFF; } NETWORK;
} HostReset_t;

extern HostReset_t g_HostReset;

#define NRF_RESET_S		(&g_HostReset)
#define RESET_NETWORK_FORCEOFF_FORCEOFF_Release		0UL
#define RESET_NETWORK_FORCEOFF_FORCEOFF_Hold		1UL

#endif
