# Linker-script migration: build-agent validation

This procedure validates the linker-script naming and memory-layout
standardization on branch `fix_ldscript_naming_013`. It is **not** a report
of completed builds. The source-level linker reference audit and unit
tests are prerequisites, not substitutes for a compiler/linker run.

## Environment

Use Eclipse Embedded CDT **managed projects**, retaining the checked-in
`.project` and `.cproject` files. Install the cross-compilers and SDK
versions required by each port, including GNU Arm Embedded, RISC-V, IAR
where supported, vendor SDKs, and libraries. Build configurations should
be read from the checked-in projects, not guessed from directory names.

Do not introduce CMake, Ninja, VS Code/Theia, or an automatic GitHub
Actions workflow. No push-triggered CI. Do not merge or retag release
0.13 until build findings are reviewed.

## Scope

1. Enumerate **every** `.cproject` and `.ewp` from the repository tree.
   Include libraries and examples across **all** MCU vendors and
   architectures, not only Nordic or the scripts modified most recently.
   Exclude Nordic Connect SDK (NCS) projects from the build matrix,
   acceptance totals, and BLOCKED counts. NCS/CMake projects are outside
   the Eclipse managed-project validation scope; do not attempt to build them.
2. Enumerate every enabled configuration in each project. Compile and link
   all supported non-NCS configurations (including Debug and Release) using their
   own managed-project settings, toolchains, and linked resources.
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

## Required build result record

For **every** project/configuration, record:

- Repository-relative `.cproject` or `.ewp` path and configuration
  name, selected target MCU, toolchain and SDK version.
- Build command or managed-builder invocation, result (PASS, FAIL, or
  BLOCKED), complete compiler/linker diagnostics, and the referenced
  `-T`/`--config` linker file.
- ELF, map file, FLASH and RAM usage, key region-boundary checks, and
  whether the output ran on hardware.
- For BLOCKED: missing prerequisites and the exact reason the
  configuration could not run; do not count it as PASS.

Exclude NCS configurations entirely from the denominator and from PASS,
FAIL and BLOCKED totals. Record them separately as out of scope if needed.

Report totals separately for compiled, linked, hardware-tested, failed,
and blocked configurations. Do not combine static audit PASS with build
PASS, or claim that a skipped configuration compiled.

## Acceptance and release

The migration can be considered build-validated only after the full
managed-project matrix has been attempted and failures attributed.
Release 0.13 merge/retag decisions must follow review of those results.
Avoid enabling any automatic GitHub Actions workflow.
