# Supported Targets

This file is the "what works today" list for IOsonata.

The goal is simple:
- **Validated** means it builds and runs from the current tree using the documented workflow.
- **Experimental** means it may build, but is not consistently validated (or requires local patches).
- **Legacy** means it was supported historically but is not maintained in the current validation loop.

If you add or change support, update this file in the same PR.

Tooling note: the installer scripts default to a working root folder (e.g., `~/IOcomposer` / `%USERPROFILE%\IOcomposer`) and place vendor repos under `external/` when needed.

---

## Support tiers

### Validated
These targets are part of the active validation loop.

| Vendor | MCU family / SoC | Notes | Example baseline |
|---|---|---|---|
| Nordic | nRF52 | Primary baseline | Blinky, UART log, I2C sensor, BLE peripheral |
| Nordic | nRF54L15 | Baseline | Blinky, UART log, BLE peripheral |
| ST | STM32 (baseline family) | Initial STM32 baseline (narrow scope) | Blinky, UART log, I2C sensor |

> Update this table with exact validated examples from your CI/validation loop.

### Experimental
Targets that are in progress, partially implemented, or pending toolchain/SDK integration.

| Vendor | MCU family / SoC | What's missing / unstable |
|---|---|---|
| Nordic | nRF53 | Dual-core coordination, full example coverage |
| Nordic | nRF91 | Cellular modem integration, full networking stack validation |
| Nordic | nRF54H20 | Toolchain/SDK alignment, full example coverage |
| ST | Additional STM32 families | Linker/startup/clock-tree coverage per family |

### Legacy
Previously supported targets that have been removed from the tree.

| Vendor | MCU family / SoC | Status |
|---|---|---|
| Nordic | nRF51 | Removed - no longer maintained |

---

## Desktop targets

| Platform | Status | Use cases |
|---|---|---|
| macOS | Best-effort | Simulation, host tools, unit-test harnesses |
| Linux | Best-effort | Simulation, host tools, CI harnesses |
| Windows | Best-effort | Tooling and workspace parity |

Desktop support is primarily for developer productivity. Hardware validation remains the reference point.

---

## How to validate a new target

A target should not be marked **Validated** unless:
1. It builds from a clean checkout using the documented toolchain.
2. It runs the baseline examples on real hardware.
3. The board-level configuration stays isolated (pin-map + board definitions; no project-by-project copying).
4. Any vendor SDK requirements are documented (exact version, where it's installed, and which examples need it).

Recommended practice:
- Add a minimal "proof matrix" script (build-only first, then build+flash where possible).
- Record the exact MCU/board name(s) used for validation.

---

## Reporting support issues

When filing an issue, include:
- Host OS + toolchain version
- Eclipse version (if applicable)
- Target MCU + board
- Example name
- Build configuration (Debug/Release) and relevant defines
- Full build log (and flash log if relevant)
