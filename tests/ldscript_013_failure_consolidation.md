# Release 0.13: 68-failure consolidation (November 3, 2026)

This is the disposition of the last full GNU managed-project build matrix at
`afac5e95`, **1469 PASS / 68 FAIL / 36 BLOCKED** (1573 counted). The
matrix was emulated from Eclipse CDT managed projects, not run by Eclipse.
There was no firmware hardware testing and no IAR compilation.

The commits below are **source or project-setting correction candidates**, not
verified firmware build passes. A later targeted rebuild and one final complete
matrix must establish which failures actually disappear. Do not reduce the 68
failure count by adding proposed fixes, excluding failures from the accounting,
or enlarging a production DFU partition to fit a Debug image.

## Release 0.13 scope update: Nordic Mesh deferred

Nordic Mesh support is deferred to a future release in favor of an IOsonata
Mesh implementation. The four vendor Mesh light-switch examples
(`MeshLgtSwClient` and `MeshLgtSwServer`, nRF52832 and nRF52840, Debug and
Release) are **out of scope** for the 0.13 GNU build matrix. Do not attempt
to repair or build them for 0.13, and do not count their eight configurations
as PASS, FAIL or BLOCKED. Keep their project/source files unchanged.

The historical 68-FAIL matrix below remains as recorded and is not rewritten.
For the latest separately reported 1487 PASS / 50 FAIL / 36 BLOCKED matrix,
removing 32 IAR, two NCS and eight Nordic Mesh configurations gives the
reclassified **0.13 in-scope historical baseline: 1487 PASS / 42 FAIL /
2 BLOCKED (1531 configurations)**. This is an accounting change, not a newly
observed successful build. A complete rebuild at the current branch head is
still required for final acceptance.

## Status by root cause

| Cause | Last FAIL | Correction candidates | Still requires implementation or dependencies |
| --- | ---: | ---: | ---: |
| Vendor SDK revision and missing components | 24 | 24 | 0 |
| MCU ports and example source defects | 24 | 22 | 2 |
| Memory region capacity | 11 | 0 | 11 |
| Project source and configuration gaps | 6 | 0 | 6 |
| GCC 15.2.1 LTO internal error | 3 | 3 | 0 |
| **Total** | **68** | **49** | **19** |

"Correction candidate" is deliberately not a PASS. In particular, the
Linux library fixes and all fixes after `afac5e95` have not been compiled
in the full matrix. Previous Windows Debug and Release improvements are already
included in the 1469 PASS baseline and are not counted again.

## Historical correction candidates (49 configurations; eight Mesh configurations now excluded)

| Configurations | Scope | Commits | Required confirmation |
| ---: | --- | --- | --- |
| 4 | Linux library, Debug/Release and Debug_RPI/Release_RPI | `08e96691`, `59232ca9` | Clean archives, sources, diagnostics |
| 2 | RA4M1 UsbHidKeyboard, Debug/Release | `ff32ebe2` | Compile, pin-map and USB runtime verification |
| 4 | STM32L476/L496 FlashMemoryDemo, Debug/Release | `ff32ebe2` | Compile, external flash geometry and hardware |
| 8 | nRF52832/nRF52840 LvglDemo and LvglDualDIsplayDemo | `21f6d501` | Compile against separate pinned LVGL revision |

| 2 | nRF54LM20x TinyUsbComboStress, Debug/Release | `21f6d501` | Compile against separate pinned TinyUSB revision |
| 2 | nRF52840 QSpiTest, Debug/Release | `fe92dea6` | Compile/link against nrfx 4, QSPI read/write on board |
| 2 | nRF52840 radio_test, Debug/Release | `fe92dea6` | Compile/link against nrfx 4, radio operation on board |
| 3 | nRF5340_Net BleAdvertiser, BleIntrfPrbsTx, BleOta Release | `f7ffddad` | Confirm GCC no longer ICEs and images fit unchanged regions |
| 2 | STM32WBA65 BlePrbsRxStmCube, Debug/Release | `e838b77c` | Populate STM32CubeWBA v1.10.0 app, HAL, CMSIS and WPAN submodules; compile |
| 2 | nRF52840 EInkDisplayDemo, Debug/Release | `aa66ae4c` | Compile/link, exercise E-Ink bitbang and reset on hardware |
| 2 | nRF52832 DistMeasureRxDemo, Debug/Release | `8013904b` | Compile/link with Nordic nrf_dm and match reflector/initiator synchronization |
| 2 | SAM4E CryptoTest, Debug/Release | `d35d5fa2` | Compile/link SAM4E TC0..TC8, count/compare and callback tests on board |
| 2 | ESP32C3 CryptoTest, Debug/Release | `2f80c332` | Compile/link TG0/TG1 timers, alarm and interrupt tests |
| 4 | ESP32C6 Blinky and CryptoTest, Debug/Release | `9e4af006`, `0d1e503f`, `37db5fc8` | Compile/link minimum GPIO/UART/TIMG drivers, check ELF ResetEntry and CPU vector order; runtime ISR tests required |

The original `git clone` paths for shared `lvgl` and `tinyusb` remain in
place. Only the legacy example projects select their separate checked-out
revisions. STM32CubeWBA remains pinned at v1.10.0.


## ESP32-C6 interrupt architecture correction

ESP32-C6 has an interrupt matrix mapping peripheral sources to 32 CPU
interrupt inputs (0, 3, 4 and 7 reserved for core-local use). The old C6
startup incorrectly selected CLIC mode and installed a 96-entry source vector.
The C6 project now links the cross-chip INTMTX, GPIO, UART, UART-clock and
Timer Group implementations, routes UART1/GPIO/TG0/TG1 to CPU lines 5/6/8/9,
and keeps a 32-entry CPU vector at the start of IRAM. The two application
projects explicitly require `ResetEntry` from the static library. The C6
source ID mappings follow ESP-IDF v5.5.1 (TG0/T0=51 and TG1/T0=54).

These corrections are substantive port work, **not** observed boot, ISR or
link results. The previous C6 Blinky Release ELF entry failure remains an
acceptance test. Do not infer it is resolved from linker option inspection.

## Nordic Mesh source reconciliation (historical; deferred from 0.13)

The four Eclipse managed projects previously linked only 14-37 Mesh-related
resources, mostly headers and example helpers. Nordic Mesh 5.0.0's upstream
Segger Embedded Studio build manifests specify **102 C units for the client**
and **115 C units for the server**, excluding their app-owned main.c files;
each also lists 13 Nordic SDK C units, with chip system startup owned by the
MCU library.

The corrected projects link those missing Mesh/core/stack/access/bearer/
provisioning/model units and app-owned SDK sources, add the upstream include
directories, and use SoftDevice API level 7 in each Debug/Release config.
Existing application main.c and board selections are retained; the shared
IOsonata nRF52 libraries and linker scripts are unchanged.

Vendor manifest: `IOsonata/nRF5_SDK_Mesh` commit `526975f6` (Nordic Mesh
5.0.0), `examples/light_switch/{client,server}/*.emProject`. Its SES
configuration targets nRF5 SDK 17.0.2; the installed IOsonata fork uses
commit `77a3de1`. Source path existence is checked, but API/ABI compatibility,
duplicate definitions against existing archives, and actual firmware behavior
are **not** verified until the final managed-build and board tests.

## Remaining MCU/example implementations (2 configurations)

| Configurations | Project | Current blocker |
| ---: | --- | --- |
| 2 | RP2040 library | MCU library port incomplete |

These require real MCU implementations and the repository's normal cross-port
`AGENTS.md` review. A no-op timer or a dummy entry symbol is not a fix.

## Remaining project/dependency gaps (6 configurations)

| Configurations | Project | Current blocker |
| ---: | --- | --- |
| 2 | nRF52832 Thingy52 | Vendor Thingy:52 application modules and `app_button.h` missing from selected source set |
| 2 | ESP32C6 qemutest | Managed project has no compilable source selection |
| 2 | nRF54H20 RISC-V library | RISC-V core port and source selection remain incomplete |

Nordic's archived `Nordic-Thingy52-FW` has `source/modules` for the Thingy
application, but importing those files must be reviewed with the project's
selected nRF5 SDK version and public APIs. Do not copy them indiscriminately.

## Remaining fixed-partition overflows (11 configurations)

| Project/configuration | Overflow (bytes) |
| --- | ---: |
| SAMD11 UartPrbsRxTest Debug | 516 |
| SAMD11 UartPrbsTxTestTaktOS Debug | 1444 |
| SAMD11 UartSlipPrbsTxTest Debug | 1468 |
| SAMD11 UartSlipPrbsTxTest Release | 96 |
| nRF52840 DfuBoot Debug | 2356 |
| nRF5340_Net DfuBoot Debug | 19980 |
| RE01 DfuBoot Debug | 2488 |
| R9A02G021 DfuBoot Debug | 57680 |
| R9A02G021 DfuBoot Release | 1568 |
| R9A02G021 CryptoTest Debug | 10612 |
| nRF5340_Net BleOta Debug | 93564 |

The first four are small SAMD11 targets. DFU region boundaries and NVM
reservations must remain intact. A Release-only DFU policy may legitimately
make some Debug images unsupported, but do not relabel an attempted Debug FAIL
as PASS or remove a supported configuration without an explicit decision.
A code-size fix requires map/symbol attribution, not linker-space inflation.

## Build-agent handoff: once after consolidated corrections

1. Freeze a branch SHA; capture the dirty state before any build commands
   (the IPC test binaries under `tests/ipc/build/` may be modified).
2. Run full `python3 -m unittest discover -s tests -p 'test_*.py' -v` and
   `python3 tests/ldscript_reference_audit.py`.
3. Provision pinned vendor SDK versions, keeping the normal SDK versions
   alongside the example-specific checkouts. Report missing publications.
4. Clean build every **in-scope GNU Eclipse managed-project configuration**
   (1531 in the historical matrix after IAR, NCS and Nordic Mesh exclusions;
   recompute against the current repository inventory). Capture diagnostics
   for every remaining FAIL and compare configuration-by-configuration against
   `afac5e95` without treating exclusions as build passes.
5. Inspect every ELF/map, vector/entry, NVM and DFU boundary; separately report
   IAR, missing SDKs and hardware-only limitations. Zero hardware results remain
   zero, regardless of compile/link success.
6. Update the build counts only after observed results. Keep draft PR #111
   unmerged and the 0.13 release tag unchanged pending approval.

PR #112 is now closed following integration of the three-plugin IOcomposer
installer changes into this working branch at `01fc0642`. Do not reapply its
commits or try to merge the closed PR again. PR #111 remains the only open
release integration PR.
