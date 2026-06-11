# File: docs/FAQ.md
# Purpose: concise answers you can link/hand to stakeholders

# IOsonata FAQ

## What do you mean by “portable”?
**Same driver source runs on MCUs and host** (macOS/Linux/Windows). Per-board differences are **data-only** (`board.h`, `Init(cfg)`)—no driver edits.

## How is that different from “big matrix portability”?
A large catalog with per-board/per-SoC variants is **poly-porting**, not portability. It multiplies code paths and conflicts. IOsonata keeps **one driver** and changes only data.

## Why is IOsonata “enterprise” for us?
- **Lower operational risk:** tiny change surface, fewer conflicts.
- **Faster hotfixes:** host-parity CI, small diffs → hours not weeks.
- **Predictable ports:** new MCU/board = update `board.h`, rebuild.

## Why not just use Zephyr?
Great breadth, but high ceremony (Devicetree/Kconfig/kernel) and frequent driver variants. If you value **speed, predictability, and true portability**, IOsonata fits better.

## Do we need an RTOS?
No. IOsonata works bare-metal or with FreeRTOS. Same APIs.

## How do we migrate quickly?
1) Map pins to `board.h`.  
2) Build and test on host.  
3) Flash MCU. Same sources, no driver edits.

## How do we prove portability?
See **Portability Proof Matrix**: CI builds `exemples/*` on Linux/macOS/Windows with the **same sources**, publishing ✅/❌ and logs.

---

# File: scripts/check_matrix_strict.py
#!/usr/bin/env python3
"""
Fail the build if any example failed on any OS, with optional allowlist regex.
Usage:
  python3 scripts/check_matrix_strict.py --docs docs --allow "exemples/legacy/.*"
"""
from __future__ import annotations
import argparse, csv, re
from pathlib import Path

def read_csvs(docdir: Path):
    rows = {}  # example -> {os: status}
    for osname in ("linux", "macos", "windows"):
        p = docdir / f"PORTABILITY_PROOF_MATRIX.{osname}.csv"
        if not p.exists(): continue
        with p.open() as f:
            r = csv.DictReader(f)
            for row in r:
                ex = row["example"].strip()
                rows.setdefault(ex, {})[osname] = row["status"].strip()
    return rows

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--docs", default="docs")
    ap.add_argument("--allow", action="append", default=[], help="Regex of examples allowed to fail")
    args = ap.parse_args()
    allow_rx = [re.compile(rx) for rx in args.allow]
    rows = read_csvs(Path(args.docs))
    failures = []
    for ex, statuses in sorted(rows.items()):
        if any(rx.search(ex) for rx in allow_rx):
            continue
        for osname in ("linux","macos","windows"):
            st = statuses.get(osname, "❌")
            if st != "✅":
                failures.append((ex, osname, st))
    if failures:
        print("Strict check: failures detected:")
        for ex, osname, st in failures:
            print(f"  {ex} @ {osname}: {st}")
        raise SystemExit(1)
    print("Strict check: all examples built ✅ across recorded OSes")

if __name__ == "__main__":
    main()

---

# Edit: .github/workflows/proof-matrix.yml (append strict gate job)
# Add this new job at the end of the workflow.

  strict-gate:
    needs: merge
    runs-on: ubuntu-latest
    if: ${{ vars.STRICT == 'true' || env.STRICT == 'true' }}
    steps:
      - uses: actions/checkout@v4
      - name: Download merged matrices
        uses: actions/download-artifact@v4
        with:
          pattern: matrix-*
          path: .
      - name: Rehydrate docs folder
        run: |
          mkdir -p docs
          cp matrix-ubuntu-latest/docs/PORTABILITY_PROOF_MATRIX.linux.csv docs/ || true
          cp matrix-macos-latest/docs/PORTABILITY_PROOF_MATRIX.macos.csv docs/ || true
          cp matrix-windows-latest/docs/PORTABILITY_PROOF_MATRIX.windows.csv docs/ || true
      - name: Run strict check
        run: |
          python3 scripts/check_matrix_strict.py --docs docs \
            --allow "exemples/legacy/.*" || (echo "::error ::Portability regression detected"; exit 1)

---

# README addition (toggle strict mode)
Add to your README or docs:

> **CI strict mode:** set `STRICT=true` (in repo Variables/Secrets or workflow_dispatch) to **fail** the build if any example stops building on Linux/macOS/Windows. Use `--allow` patterns in `check_matrix_strict.py` for known legacy paths.
