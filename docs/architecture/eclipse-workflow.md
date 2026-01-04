# Eclipse Linked-Resource Workflow (Why It Exists)

IOsonata’s default workflow is optimized for one real-world constraint:

> Embedded teams rarely have “one firmware.” They have a family of firmware projects that share drivers, protocols, and platform code—and they need fixes to propagate safely.

This document explains why the Eclipse linked-resource approach exists and when you should use it.

---

## What the installer configures

The IOsonata `Installer/` scripts do more than “download Eclipse”:
- Install toolchains and (where applicable) add them to PATH
- Seed Eclipse Embedded CDT preferences so the toolchains are discovered
- Configure workspace defaults so examples import/build cleanly
- Optionally install IOsonata Eclipse templates/plugins (when present in the repo)

If you are troubleshooting path/toolchain issues, start by reviewing the installer script output and the documentation under `Installer/`.

---

## The problem IOsonata is solving

In multi-target embedded programs, teams often end up with:
- A “golden” project per board that slowly diverges
- Copied drivers per product line (“just tweak it here”)
- Multiple forked vendor SDK drops per product
- A growing tax of “BSP drift” that turns small fixes into multi-week cleanups

The cost is not only engineering time. It’s predictability:
- Bugs reappear because fixes do not propagate
- Behavior diverges across variants (often without intent)
- Porting becomes a rewrite instead of a bounded change

---

## IOsonata’s stance: keep one shared copy of the code

IOsonata prefers:
- **One physical IOsonata tree** (and shared modules) on disk
- **Many Eclipse projects** in a workspace that link to that tree
- Each firmware project contains only:
  - App-specific code
  - Board/target configuration (pin-map + board-level definitions)
  - Project metadata/build configuration

This keeps shared code genuinely shared.

---

## What linked resources do (in practical terms)

Eclipse linked resources let a project reference folders/files located elsewhere on disk.
IOsonata uses this to reference:
- The IOsonata framework directory
- Shared drivers/modules used by multiple projects
- Optional vendor SDK components (when required)

Benefits:
- **No duplication**: you do not copy drivers or framework folders into each project.
- **One fix propagates**: patch a driver once; every project picks it up.
- **Faster iteration**: switching targets is often a build-config change + `board.h` edit, not a new project fork.

---

## Recommended workspace layout

The installer scripts standardize three things:
1. A convenient working root folder (default: `~/IOcomposer` on macOS/Linux, `%USERPROFILE%\IOcomposer` on Windows)
2. Toolchains installed in OS-appropriate locations (e.g., `/opt/xPacks` on macOS/Linux; `%ProgramFiles%\xPacks` on Windows)
3. Eclipse Embedded CDT installed in OS-appropriate locations (e.g., `/Applications/Eclipse.app` on macOS; `/opt/eclipse` on Linux; `%ProgramFiles%\Eclipse Embedded CDT` on Windows)

Inside the working root, the typical layout is:

- `IOcomposer/`
  - `IOsonata/` (framework repo clone)
  - `external/` (vendor SDKs + third-party repos checked out by installer)
  - `workspaces/`
    - `myprojects/` (recommended Eclipse workspace folder)

The folder name is just a default; the important part is the structure.

---

## When you should not use the linked-resource workflow

Use a different workflow if:
- You cannot standardize paths/repository layout in your organization
- Your toolchain or policy forbids IDE-managed projects
- Your team already has a disciplined monorepo build stack with proven anti-duplication controls

In those cases, treat IOsonata as a library and integrate it into your build system.
The core requirement still applies: **do not duplicate shared source**.

---

## The design principle to enforce (regardless of tooling)

If IOsonata can give you one invariant, make it this:

> A board variant should be a pin-map/config change, not a fork.

Linked resources are one effective mechanism to enforce that invariant at scale.
