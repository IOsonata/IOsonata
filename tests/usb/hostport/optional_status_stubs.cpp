/**-------------------------------------------------------------------------
@file	optional_status_stubs.cpp

@brief	Host-only fallbacks for optional application status hooks on macOS.

Mach-O links cannot always leave weak undefined references unresolved in
test executables. Keep these implementations test-local; real subsystems
provide strong implementations when present.
----------------------------------------------------------------------------*/
extern "C" {

__attribute__((weak)) void BtAppCheckStatus(void) {}
__attribute__((weak)) void LteCheckStatus(void) {}
__attribute__((weak)) void GnssCheckStatus(void) {}
__attribute__((weak)) void IpCheckStatus(void) {}
__attribute__((weak)) void Nrf70WorkCheckStatus(void) {}
__attribute__((weak)) void Nrf70TaskletCheckStatus(void) {}
__attribute__((weak)) bool Nrf70TaskletPending(void) { return false; }
__attribute__((weak)) bool Nrf70WorkPending(void) { return false; }

}
