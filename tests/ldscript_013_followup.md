# Release 0.13 linker follow-up: disposition and validation

These items apply to branch `fix_ldscript_naming_013`. Static linker correctness,
build results, and upgrade compatibility are separate acceptance criteria.

## RE01 DFU

`tests/re01/build_dfu.py` selects `gcc_re01_1500kb_dfu_boot.ld`
for boot and `gcc_re01_1500kb_dfu.ld` for the application. Its common
compile flags now include `uECC_ENABLE_VLI_API=1`, needed by
`src/crypto/crypto_uecc.cpp` to declare the VLI functions and by
`micro-ecc/uECC.c` to export their definitions. Rebuild and execute the
fixture before recording a compiler PASS.

## nRF52840 SDC existing bonds

Eight configurations are affected by the old 12 KB peer-manager NVM0
being split into 8 KB NVM0 and 4 KB NVM1. No on-device migration
exists. Static link tests cannot establish that bonds survive an upgrade.
Do not deploy this new partition layout on bonded devices without
tested migration or explicit erase/re-pairing. Keep this a release
compatibility limitation, not an unacknowledged build PASS.

## STM32F030x8 DFU

`gcc_stm32f030x8_dfu.ld` is unselected. A prior Debug trial overflowed
its DFU slot by 1,744 bytes, while Release fitted. Do not enlarge the
partition to force a Debug image to fit. If adding a managed DFU
configuration, use Release and validate image/slot boundaries.

## nRF54LM20B S145/DFU address reconciliation

The LM20B S145 script and shared DFU layout originally placed S145 at
`0x0DA800` and storage at `0x0D8800`; the Nordic S145 image and
`bm_config_defaults.h` use `0x1DA800`. The scripts now match the
LM20A 2 MB layout: NVM0 `0x1D8800`, NVM1 `0x1D9800`,
S145 `0x1DA800`, and DFU slot 1 ends at `0x1D8800`.
The DFU boot/application scripts already include the shared layout,
so only the shared layout and non-DFU S145 partition definitions
needed adjustment.

No managed project selected these scripts in the baseline. This is
a substantive change to storage addresses. Any deployed LM20B image
using the former 1 MB map needs an explicit migration or clean
provisioning plan before adopting this layout.

## Obsolete documentation and comments

Eight nRF91 example READMEs now use the existing `gcc_nrf9160_xxaa.ld`
and `gcc_nrf9120_xxaa.ld` names. LM20A and LM20B DFU layout comments
now name their real `gcc_..._s145_dfu_boot.ld` files.
Other outdated comment-only script mentions are non-build references.
They should be cleaned separately from active project settings, but
must not be counted as build failures.

## Validation

Run from the root of `fix_ldscript_naming_013`:

```sh
python3 -m unittest discover -s tests -p 'test_ldscript_013_followup.py' -v
python3 -m unittest discover -s tests -p 'test_ldscript_reference_audit.py' -v
python3 tests/ldscript_reference_audit.py
python3 tests/re01/build_dfu.py --build-only
```

The RE01 fixture requires the cross-compiler and Python dependencies
already documented by its test harness. No new build-matrix result is
claimed here.
