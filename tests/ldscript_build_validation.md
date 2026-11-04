# Linker-script migration: build-agent validation

This procedure validates the linker-script naming and memory-layout
standardization on branch `fix_ldscript_naming_013`. It is **not** a report
of completed builds. The source-level linker reference audit and unit
tests are prerequisites, not substitutes for a compiler/linker run.

## Environment

Use Eclipse Embedded CDT **managed projects**, retaining the checked-in
`.project` and `.cproject` files. Install the cross-compilers and SDK
versions required by each in-scope port, including GNU Arm Embedded,
RISC-V, vendor SDKs, and libraries. IAR and Keil toolchains are not required. Build configurations should
be read from the checked-in projects, not guessed from directory names.

Do not introduce CMake, Ninja, VS Code/Theia, or GitHub Actions CI.
No push-triggered or pull-request-triggered build/test workflows.
The existing `.github/workflows/build-rag-index.yml` is the sole exception:
it maintains the RAG index on its own schedule or by manual dispatch.
Do not remove or repurpose it. Do not merge or retag release 0.13 until
build findings are reviewed.

## Scope

1. Enumerate every Eclipse CDT `.cproject` from the repository tree.
   Include libraries and examples across **all** MCU vendors and
   architectures, not only Nordic or the scripts modified most recently.
   Exclude Nordic Connect SDK (NCS), IAR (`.ewp`, `.eww`, `.icf`) and
   Keil (`.uvproj`, `.uvprojx`, `.uvopt`, `.uvoptx`, `.sct`) projects from
   compilation, the build matrix, acceptance totals, and BLOCKED counts.
   These project formats are outside the GNU Eclipse managed-project build
   scope. Do not require their proprietary toolchains or treat their
   configurations as failed, blocked or untested GNU builds.
   Also exclude the four Nordic Mesh reference examples, `MeshLgtSwClient`
   and `MeshLgtSwServer` for nRF52832 and nRF52840 (Debug and Release:
   eight configurations). Nordic Mesh is deferred; IOsonata will implement
   its own Mesh support in a future release. Preserve these project files
   and sources, but do not compile or count their configurations in 0.13.
   Exclude all five unfinished BleOta example projects (nRF52832,
   nRF52840, nRF5340_Net, nRF54L15, nRF54LM20x), Debug and Release:
   ten configurations. In the last measured matrix, eight were PASS and
   nRF5340_Net Debug and Release were FAIL. Exclude all ten from the
   denominator; preserve their source and project metadata unchanged.
   Do not spend 0.13 remediation effort on BleOta.
   Exclude the nRF52832 Thingy52 vendor firmware reference project (Debug and
   Release: two configurations). It depends on a different Nordic SDK
   generation and missing original application modules. Retain its files
   unchanged; do not port or build it for release 0.13.
2. Enumerate every enabled configuration in each in-scope project. Compile
   and link all supported GNU Eclipse managed-project configurations
   (including Debug and Release) using their own checked-in settings,
   toolchains, and linked resources.
3. Keep real SDK-dependent reference examples separate from IOsonata
   projects. In particular, the nRF52832 `UartPrbsTxnrfx` project
   intentionally uses a local Nordic `nrf_common.ld` and Nordic startup,
   while all IOsonata Nordic family linker layouts include
   `gcc_arm_flash.ld`.
4. For each ELF: inspect the generated map, entry symbol, interrupt
   vectors, `.text`, `.rodata`, `.data`, `.bss`, stack start,
   FLASH/RAM region usage, `NOLOAD` NVM regions, and any storage or
   bootloader settings boundaries. Confirm that no assigned section
   overlaps another region or crosses the linker-defined limit.
5. For Nordic DFU configurations: verify both boot and application
   images, their shared `*_dfu_layout.ld` definitions, relocated vector
   table, slot boundaries, NVM partitions, recovery flag, and any shared
   RAM/IPC reservation. Do not treat a successful static boundary check
   as proof of boot/update behavior.
6. For nRF5340 App/Net: verify IPC0 region, Network owner versus App
   peer HCI ring size, reserved recovery flag, and SPU permissions on
   hardware when available. For nRF54H20: treat both core partitions
   separately and check shared global RAM ownership.
7. Preserve potentially valid, unselected linker variants. If compilation
   fails, report the exact configuration and diagnose whether the
   compiler, SDK, project setting, linker include, startup file, or
   memory layout is responsible. Do **not** delete a script simply
   because it has no current `.cproject` reference.

## Preflight commands (from repository root)

```sh
git status --short
git branch --show-current
python3 -m unittest discover -s tests -p 'test_ldscript_reference_audit.py' -v
python3 tests/ldscript_reference_audit.py
python3 tests/ldscript_reference_audit.py --report-unreferenced
```

The inventory command reports selectable-but-unreferenced scripts;
its output does not authorize deletion.

## DFU bootloader Debug FLASH overflow policy

A DFU bootloader **Debug** configuration whose only failure is exceeding its
fixed FLASH region is an accepted size exception for release 0.13. Do not
optimize it further solely to fit, enlarge its partition, or mark it PASS.
Keep the attempted result and linker diagnostic in the detailed build record,
but report it separately as **WAIVED (Debug DFU FLASH overflow)** instead of
an actionable FAIL. A Debug compile error, undefined symbol, RAM overflow,
invalid section placement, or failing Release configuration remains a FAIL.
Release DfuBoot images must compile, link and pass the usual ELF and DFU checks.

In the last measured matrix, the affected Debug images are RE01, nRF5340_Net
and R9A02G021 DfuBoot (three WAIVED). nRF52840 DfuBoot Debug already PASSes.

## Required build result record

For **every in-scope** project/configuration, record:

- Repository-relative `.cproject` path and configuration name,
  selected target MCU, GNU toolchain and SDK version.
- Build command or managed-builder invocation, result (PASS, FAIL, BLOCKED, or
  WAIVED), complete compiler/linker diagnostics, and the referenced
  `-T`/`--config` linker file.
- ELF, map file, FLASH and RAM usage, key region-boundary checks, and
  whether the output ran on hardware.
- For BLOCKED: missing prerequisites and the exact reason the
  configuration could not run; do not count it as PASS.

Exclude NCS, IAR, Keil, Nordic Mesh, unfinished BleOta, and Nordic Thingy52 configurations
entirely from the denominator and from PASS, FAIL and BLOCKED totals. Record
out-of-scope configurations separately if needed. Exclusion does not authorize
deleting their source files, project metadata or linker scripts; the reference
audit may still inspect them.

Report totals separately for compiled, linked, hardware-tested, failed,
blocked, and WAIVED configurations. A WAIVED image is neither a PASS nor an
exclusion from the in-scope denominator. Do not combine static audit PASS with build
PASS, or claim that a skipped configuration compiled.

## Acceptance and release

The migration can be considered build-validated only after the full
managed-project matrix has been attempted and failures attributed.
Release 0.13 merge/retag decisions must follow review of those results.
Do not add GitHub Actions build/test workflows; retain only the RAG index updater.
