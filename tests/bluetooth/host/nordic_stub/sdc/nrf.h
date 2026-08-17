// Host stand-in for the Nordic device header. The real one describes the
// peripherals and the CMSIS core of a particular part, none of which a host
// build has or needs. bt_hci_ctlr_sdc.cpp includes it and uses nothing from
// it, so this is deliberately empty rather than a place to collect symbols.
//
// The -NRF_EPERM and -NRF_EINVAL codes sdc_init answers with are named in that
// file's comments only. In nrfxlib they come from nrf_errno.h, so they do not
// belong here even if a caller did use them.
#ifndef __NRF_STUB_H__
#define __NRF_STUB_H__

#endif // __NRF_STUB_H__
