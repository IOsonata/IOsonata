# RISC-V linker-script organization

The linker filename convention is `<compiler>_<target>[_<memory-layout>].ld`.
All selectable scripts use lowercase MCU identifiers and live in a common
`ldscript` directory for their device family. The shared architecture section
definitions live in `RISCV/ldscript`. Eclipse managed projects select the
complete target layout for their configuration.

## Espressif ESP32-C

- `RISCV/Espressif/ESP32C/ldscript/gcc_esp32c3.ld`
- `RISCV/Espressif/ESP32C/ldscript/gcc_esp32c6.ld`

The ESP32-C3 Blinky, CryptoTest, and UartPrbsTxTest projects, and the ESP32-C6
Blinky and CryptoTest projects, select their respective GCC layouts in Debug
and Release. The ESP32-C6 `qemutest` project is still an incomplete skeleton
without a linker selection; this migration does not turn it into a buildable
target.

## Renesas R9A02G021

| Selected layout | GCC | LLVM |
| --- | --- | --- |
| Normal | `gcc_r9a02g021.ld` | `llvm_r9a02g021.ld` |
| DFU application | `gcc_r9a02g021_dfu.ld` | `llvm_r9a02g021_dfu.ld` |
| DFU stage-0 boot | `gcc_r9a02g021_dfu_boot.ld` | `llvm_r9a02g021_dfu_boot.ld` |

The scripts are in `RISCV/Renesas/R9A02/ldscript`. All 17 R9A02 example
projects use the GCC script in both Debug and Release; `DfuBoot` selects
the stage-0 boot script. The LLVM layouts are selectable alternatives, not
additional Eclipse build configurations.

Two fragments have intentionally different naming roles:

- `r9a02g021_head.ld`: compiler-neutral, included by the normal and DFU
  boot scripts to place reset, CLIC, option-setting words and OSIS.
- `gcc_r9a02g021_dfu_layout.ld`: the single shared DFU boundary definition
  included by **both** GNU and LLVM DFU boot/application scripts. It is
  named after the original GCC layout for compatibility; it is not a
  separately selectable application script.

The DFU boundaries are unchanged: stage-0 boot 0x000000–0x008FFF,
record 0x009000–0x009FFF, application slot 0x00A000–0x01FFFF, and
the recovery flag at 0x20006FF0. The R9A02G021 has no second update slot.
The naming migration does not enlarge the boot or application partitions
or resolve pre-existing memory-overflow failures.

The three R9A02 emulator build helpers under `tests/r9a02` use the renamed
GCC/LLVM normal layouts.

## Nordic nRF54H20 RISC-V library

`RISCV/Nordic/nRF54/nRF54H20/lib/ioc` is an archive/library project. It
does not link an executable and has no linker-script selection; inventing a
RISC-V linker layout is not part of this migration.

## Validation

From the IOsonata repository root:

```sh
python3 -m unittest discover -s tests -p 'test_riscv_ldscript_naming.py' -v
python3 -m unittest discover -s tests -p 'test_ldscript_reference_audit.py' -v
python3 tests/ldscript_reference_audit.py RISCV
python3 tests/ldscript_reference_audit.py RISCV --report-unreferenced
```

Static checks validate names, selected file paths, local `INCLUDE` references,
DFU boundaries, and the deliberately unconfigured qemutest. They are not
compiler/linker or hardware evidence. Full build results and existing build
failures are tracked separately in `tests/ldscript_build_validation.md`.
