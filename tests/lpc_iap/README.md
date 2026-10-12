# LPC11Uxx IAP interrupt protection

Run `python3 tests/lpc_iap/run.py` with host GCC. It builds the production
shared LPC IAP source with LPC11XX_SERIES at -O0 and -O2 with
undefined-behavior checking. Test interrupt helpers model the CMSIS-backed
production helpers. The test verifies the LPC11 ROM entry address before
replacing IapEntry with a ROM stub, and includes both old and shared headers
to check compatible types and the legacy command-name alias.

All twelve wrappers must call ROM with PRIMASK set and restore the original
mask after success and error returns. Each is checked with interrupts
initially enabled and already disabled, covering an outer critical section
such as DFU's prepare-plus-erase/write sequence. The test fails on the
unprotected implementation. The ISP test covers a returning ROM stub only;
it does not simulate entering the bootloader.

UM10462 section 20.8.7 requires interrupt protection before any IAP call.
The wrappers now provide per-call protection. Callers still own serialization
across multiple calls, flash address/size requirements, and reserving ROM
workspace and stack. This change does not complete ISP entry preparation.

Cortex-M0 compilation to assembly passed at -O0 and -O2 using the real
CMSIS core header. Host tests do not execute ROM or erase/program hardware.
Hardware IAP and complete firmware link validation remain pending.

LPC11U35 now links ARM/NXP/src/lpc_iap.c in Debug and Release, as LPC54628
does. The duplicated LPC11xx/src/iap_lpc11uxx.c is removed. Custom build
projects must make the same source substitution, define LPC11XX_SERIES and
include ARM/NXP/include. Existing iap_lpc11uxx.h includes and public function
calls remain valid. Clean/rebuild the library after updating project links.
Only the original twelve LPC11 commands are exercised here; the shared
header does not imply that LPC546xx-only commands are supported by LPC11 ROM.
