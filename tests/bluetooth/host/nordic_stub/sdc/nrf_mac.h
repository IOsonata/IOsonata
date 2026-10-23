// Host stand-in for the factory address of the FICR, which bt_hci_ctlr_sdc.cpp
// answers Read BD_ADDR with. A fixed value with the static random marker
// already set, as the real one has.
#ifndef __NRF_MAC_STUB_H__
#define __NRF_MAC_STUB_H__

#include <stdint.h>

static inline uint64_t nrf_get_mac_address() {
	return 0xC0A5A5A5A5A5UL;
};

#endif // __NRF_MAC_STUB_H__
