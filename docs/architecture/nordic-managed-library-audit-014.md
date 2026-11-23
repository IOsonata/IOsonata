# IOsonata 0.14 - Nordic Eclipse managed library-path audit

Working branch: `port_nrf54l_power_clock_014`.

## Scope

All `ARM/Nordic/**/ioc/.cproject` files were inventoried through the
repository tree, including nRF52, nRF53, nRF54, nRF71 and nRF91. The
inventory contains **423 managed project files** (including libraries
and examples). nRF54H20 legacy library metadata was inspected but no
out-of-scope nRF54H target implementation was added.

This pass audits the **Eclipse managed library-path consistency**:
a `Release` or `Release_SDC` example must never select a `Debug`
library directory. `Debug` variants must not select `Release`.
Existing alternative paths for e.g. SDC remain in their matching
Debug or Release family, and the expected target library names are
retained. The changes affect only library search paths in
`.cproject`; no clock, power, startup, linker scripts or example
source is changed.

nRF5340 App/Net and nRF54L15/LM20A/LM20B were previously corrected;
the full Nordic pass catches remaining mismatches across nRF52,
nRF71, nRF91 and any missed entries. Changes were accumulated in Git
trees and delivered as **one batch commit**, not one commit per file.

## Regression and acceptance

Run:

```sh
python3 tests/nordic/check_managed_library_paths.py
```

The regression automatically enumerates Nordic example managed
projects, requires their Eclipse `.project` files, and checks every
named Debug/Release configuration for a matching target library
and matching library build directory. It also verifies that managed
library projects have their Eclipse metadata present.

This audit is **static repository metadata validation**. It does not
establish successful compilation, link completion, runtime behavior,
or correctness of the MCU wizard/linker scripts. Those require a
separate Eclipse managed build matrix and hardware testing. NCS,
IAR and Keil projects are out of scope.
