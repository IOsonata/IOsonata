# Shared LPC IAP

Run `python3 tests/lpc/iap_shared/run.py`. Host GCC compiles the production
shared IAP implementation as C, then GCC and G++ link callers of the public
IAPReadExtFlashSignature declaration at -O0 and -O2 with UBSan.

A ROM stub verifies command 73, the start/end flash addresses, wait-state
encoding and required zero fourth parameter. Tests verify all four words
of the 128-bit result, surrounding output sentinels, unchanged output on
ROM error and rejection of a null output without a ROM call. Before the
fix, the test failed to link because the implementation had a different name.

Command 70 is also checked for its single-word result, unchanged output on
ROM error and null-output rejection. The ROM stub supplies deliberately
invalid result words on error to catch wrappers that copy them anyway.
The ordinary signature regression failed before its error handling was fixed.

Factory-settings reads are checked for success, parameter/unsupported-command/
busy errors and null output. Error-result words must not replace the caller's
existing value. The test register address is only a value passed to the ROM
stub; this test does not establish command support on any particular chip.

The command layout follows NXP UM10912 section 5.6.11, table 83. Although
earlier argument names said StartPage/EndPage, LPC546xx ROM expects flash
addresses. The public header now states this and requires a four-word
output buffer. No generic driver API is changed.

The EEPROM page test covers the previously missing IAPEepromWritePage symbol.
Its ROM stub checks command 81, page number, buffer address and conversion
of the core clock from Hz to kHz at 12 and 48 MHz. It checks success and ROM
error propagation and rejects a null source without calling ROM. The command
layout follows NXP UM10912 section 5.6.13, table 85. This is a complete-page
write; the caller supplies all 128 bytes.
The test failed to link before the implementation was added.

The interrupt test exercises all 16 wrappers with interrupts initially enabled
and already disabled, for both success and error returns. ROM entry must be
masked and the original state must be restored exactly once. The old code
failed this check. Production uses coredev/interrupt.h, which saves and
restores PRIMASK through CMSIS on ARM. Test substitutes model these helpers.
This follows UM10912 section 5.3.8.1. Per-call protection does not serialize a
multi-call prepare/erase/write sequence; the caller still owns that sequence.
The ISP case tests a returning ROM stub, not bootloader entry.

The shared IAP source also compiles to Cortex-M0 and Cortex-M4 assembly with
LPC11XX_SERIES and LPC54628_SERIES at -O0 and -O2. Tests do not execute hardware ROM or validate
the numerical signature for real flash contents. Full firmware linking
and hardware validation remain pending.
