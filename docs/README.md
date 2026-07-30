# IOsonata Documentation

This directory contains the current IOsonata architecture, development workflow and implementation references.

## Documentation precedence

Use the following order when documents differ:

1. current source and public headers;
2. current architecture documents under [`architecture/`](architecture/);
3. current workflow and usage documents in this directory;
4. the December 2025 *Beyond Blinky* publication snapshot.

The book remains useful for the object-design model and historical background, but it is not the current setup or project-workflow guide.

## Start here

1. [Project README](../README.md) — project scope, precompiled-library model, benchmarks and hardware references.
2. [Getting Started](getting-started.md) — install IOcomposer, build an MCU library, open an example, flash and debug.
3. [Quick Reference](quick-reference.md) — commands, project actions and common configuration examples.
4. [FAQ](FAQ.md) — concise answers about the architecture, target projects and build model.
5. [Supported Targets](supported-targets.md) — current hardware-validation baselines and target-status rules.
6. [Dependencies](dependencies.md) — tool, SDK and optional-library boundaries.

## Architecture

### [Architecture overview](architecture/README.md)

Defines:

- the MCU-centric precompiled-library model;
- application-owned board configuration;
- `Device` and `DeviceIntrf`;
- generic and target separation;
- linker-selected code;
- static-memory and real-time rules;
- bare-metal and RTOS independence.

### [IOcomposer workflow](architecture/iocomposer-workflow.md)

Describes how to build one MCU library, create or open an application, supply `board.h`, and link the selected Debug or Release library.

### [DeviceIntrf implementer notes](architecture/devintrf-implementer-notes.md)

Documents the C-level interface state, wrapper ownership, transfer sequencing, return values, events, enable reference counting and target-owned transfer limits.

### [Device inheritance and polymorphic composition](architecture/device-composition.md)

Documents behavioural inheritance, shared virtual `Device` state, multi-function devices, independent devices in one package, configuration-driven variation and object composition.

## Source references

Read architecture and implementation together:

- `include/device_intrf.h` and `src/device_intrf.cpp`;
- `include/device.h` and `src/device.cpp`;
- the complete generic subsystem;
- every affected target implementation;
- at least one working example and the relevant tests.

Reusable example source is under [`../exemples/`](../exemples/). Buildable target projects are under each MCU target and normally reference the shared source rather than copying it.

## Beyond Blinky

Human readers can [download the free edition or buy the complete book on Leanpub](https://leanpub.com/beyondblinky).

The repository keeps a [December 2025 publication notice](beyond_blinky_free_edition.md) and the [dated machine-readable extraction](beyond_blinky_free_edition_2025_raw.md) for search and RAG indexing.

Current architecture and workflow documents take precedence when procedures, tools or repository organization differ from the published edition.

## Documentation maintenance

- Do not create a second positioning document that repeats the root README.
- Keep tutorials, quick-reference material and architecture rules in separate files.
- Use real repository paths and current public APIs.
- IOcomposer is the official IDE. An `Eclipse/` path may be shown only when it is the actual directory name of a target project.
- Update target status only with a named board and recorded hardware result.
- Do not retain an implementation or validation matrix without clear ownership and current evidence.
- Do not embed proposed scripts, workflows or README fragments inside user documentation.
- Keep dated publications clearly separated from current repository instructions.
