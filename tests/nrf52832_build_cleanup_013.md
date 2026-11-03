# nRF52832 GNU build cleanup (0.13)

Branch: `fix_build_project_settings_013`. This is a target-wide triage of
the nRF52832 Eclipse Embedded CDT managed projects, not a declaration that
every image has been rebuilt.

## Inventory and baseline

The repository tree contains 76 `.cproject` files beneath
`ARM/Nordic/nRF52/nRF52832`: the library, project template, and examples.
The original full GNU build matrix reported 88 failing configurations across
**all** platforms. Most nRF52832 configurations were already passing in that
baseline. Do not alter passing projects just to homogenize metadata.

## Project-setting defects corrected

| Project | Configurations | Correction | Verification |
| --- | --- | --- | --- |
| `UartCliDemo` | Debug, Release | Removed 12 SDK C-source links already built into the nRF52832 IOsonata library | `tests/test_nrf52832_uart_cli_source_links.py`: 1 test reported PASS |
| `UartBleFreeRTOS` | Debug_SDC, Release_SDC | Replaced S132 macros with `NRFXLIB_SDC`, corrected C/C++ linker script paths, selected matching IOsonata SDC libraries and output directories, included MPSL/FEM/SDC library search paths | `tests/test_uart_ble_freertos_sdc_project.py`: 1 test reported PASS |
| `Thingy52` | Debug, Release | Fixed the missing script reference to the existing `BlueIOThingy/src/gcc_nrf52832_thingy_xxaa.ld` | `tests/test_thingy52_project.py`: 1 test reported PASS |

All status statements above are **static validation**, not compiled firmware.

## Distinct unresolved dependencies

### Thingy52 SDK reference application

`Thingy52` has six application C source files under its own `src`
directory: `main.c` and five `ble_services` files. It has headers for
`m_ble`, `m_environment`, `m_sound`, `m_motion`, `m_ui`,
`m_batt_meas`, and the associated `drv_*` modules, but no
corresponding implementation C files in this repository. Its
`.project` links those application headers and the six C sources,
not their implementations. In particular, the Git tree contains no
`m_ble.c`, `m_environment.c`, `drv_motion.c`, `m_sound.c`,
`m_ui.c`, or `m_batt_meas.c`.

A valid linker path cannot cure missing application modules. Restore
the matching original Thingy52 SDK source package and its compatible
dependencies before expecting this reference application to link. Do not
substitute the IOsonata BlueIOThingy application or fabricate stub
implementations. Revalidate the S132/DFU memory map when restored.

### Third-party SDK examples

The baseline also identifies Nordic Mesh and LVGL SDK example failures
across the build matrix. These are SDK-version/dependency work, not
nRF52832 library or Eclipse project-setting corrections. Keep them
visible in the build report rather than recording them as project-setting
PASS.

## Completion and build handoff

Run the existing full Eclipse GNU managed-project build matrix with the
same compiler and SDK versions as the baseline. Compare results by
**project and configuration**, not by total failure count alone.
Prioritize:

1. `IOsonata_nRF52832` libraries used by the above configurations.
2. `UartCliDemo`: Debug and Release.
3. `UartBleFreeRTOS`: Debug_SDC and Release_SDC.
4. `Thingy52`: Debug and Release, recording missing module dependencies.
5. All other nRF52832 example configurations, including Mesh/LVGL SDK
   examples, to check for regressions.

The static checks can be run together:

```sh
python3 -m unittest discover -s tests -p 'test_nrf52832_uart_cli_source_links.py' -v
python3 -m unittest discover -s tests -p 'test_uart_ble_freertos_sdc_project.py' -v
python3 -m unittest discover -s tests -p 'test_thingy52_project.py' -v
```

The previous per-project tests were each reported PASS after the
corrections. No full cross-compilation has been executed from this
session, so there is no updated PASS/FAIL matrix or measured reduction
from 88 yet. Preserve the original failed configurations as baseline
until a new run supplies build logs.
