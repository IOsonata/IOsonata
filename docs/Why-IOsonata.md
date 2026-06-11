# File: docs/Why-IOsonata.md
# Purpose: concise, factual positioning you can link from README

# Why IOsonata

**Portable = one driver, many targets, no edits.**  
With IOsonata, the *same source* runs on MCUs **and** host (macOS/Linux/Windows). Per-board differences are **data-only** (`board.h`, `Init(cfg)`), not driver code.

## What this buys you
- **Lower risk:** tiny change surface → fewer conflicts, fewer bugs.
- **Faster hotfixes (MTTR):** host parity CI + small diffs → hours, not weeks.
- **Predictable ports:** same firmware app compiles for new MCU/board with just `board.h`.
- **Consistent device model:** UART/I²C/SPI/BLE/storage follow the same `Init(cfg)` + intrf pattern.

## Reality-based comparison

| Dimension | IOsonata | Zephyr | NuttX | RIOT | libopencm3 | Vendor HALs | ESP-IDF |
|---|---|---|---|---|---|---|---|
| **Portability (real)** | **One driver + `board.h` (data-only)** | Poly-porting (DTS/Kconfig variants) | Board/SoC ports | Board model | Low-level only | Per-vendor | Vendor-specific |
| **Driver matrix quality** | Focused, reusable | **Inflated by variants** | Broad, OS-centric | Broad IoT | Minimal | Broad per family | Broad (ESP) |
| **Operational risk** | **Low** | **High** | Med-High | Med | Low (you build more) | Med | Med |
| **Host parity (same sources)** | **Yes (macOS/Linux/Windows)** | Native_sim (heavy policy) | Simulator | Native | No | No | Limited |
| **Integration cost** | **Low (plain C/C++)** | High (west+DTS+Kconfig+kernel) | Med-High | Med | Low (DIY layers) | Med | Med |
| **Perf/latency** | **Lean; IRQ/DMA friendly** | Variable (layer cost) | OS overhead | OS overhead | Leanest | Varies | Good (ESP) |
| **LTS practicality** | **Simple: 2 branches + smoke** | Formal but friction | Formal releases | Regular | N/A | Vendor policy | Vendor LTS |

## Portable ≠ Poly-porting
- **Portable:** same driver compiles unmodified across MCUs/host; per-board is data only; single-commit backports.
- **Poly-porting:** per-SoC/board variants (DTS/Kconfig overlays, init quirks) → duplication, conflicts, longer MTTR.

**Litmus test**
1. Driver compiles unmodified for ≥3 MCUs **and** host.  
2. All per-board differences are in `board.h`/config structs.  
3. Fix backports are single cherry-picks.

## Proof over promises
- **Examples:** `exemples/` build and run across targets with the same sources.  
- **Portability Proof Matrix:** auto-generated from CI for Linux/macOS/Windows.  
- **Benches:** PRBS UART/BLE demos report goodput/BER; CSV + plots kept in repo.

> See: [`docs/PORTABILITY_PROOF_MATRIX.md`](../docs/PORTABILITY_PROOF_MATRIX.md)

## Migration in one hour
1) Map pins from your board to `board.h`.  
2) Build host → fix logic fast.  
3) Flash MCU. Same app, no driver edits.

---

# File: README snippet (paste into README.md)
<!-- Badges -->
<p align="center">
  <a href="docs/PORTABILITY_PROOF_MATRIX.md"><img alt="Portability Proof" src="https://img.shields.io/badge/portability-proof-green"></a>
  <a href="docs/Why-IOsonata.md"><img alt="Why IOsonata" src="https://img.shields.io/badge/why-IOsonata-blue"></a>
  <a href="exemples"><img alt="Examples" src="https://img.shields.io/badge/examples-%F0%9F%9A%80-brightgreen"></a>
</p>

## Portability Proof (Real, Not Claims)
The **same sources** in `exemples/` are built on **Linux**, **macOS**, and **Windows** in CI.  
See the live table: [`docs/PORTABILITY_PROOF_MATRIX.md`](docs/PORTABILITY_PROOF_MATRIX.md)

- ✅ = built successfully on that OS  
- ❌ = failed (see per-row logs; PRs welcome)

## Why IOsonata (60-second read)
- **Portable by construction:** one driver spans MCUs and host; per-board is `board.h`.  
- **Lower operational risk:** fewer moving parts, fewer conflicts.  
- **Faster hotfixes:** host-parity CI; minimal diffs.  
- **Uniform device model:** UART/I²C/SPI/BLE/storage share `Init(cfg)` + intrf.

Read the one-pager: [`docs/Why-IOsonata.md`](docs/Why-IOsonata.md)
