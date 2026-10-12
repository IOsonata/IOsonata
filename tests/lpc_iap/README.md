# LPC11Uxx IAP interrupt protection

Run `python3 tests/lpc_iap/run.py` with host GCC. It builds the production
LPC11Uxx IAP source at -O0 and -O2 with undefined-behavior checking. The
vendor device header remains in use; a test CMSIS core header substitutes
only the CPU intrinsics. The existing IapEntry pointer targets a ROM stub.

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
