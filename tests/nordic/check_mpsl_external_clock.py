#!/usr/bin/env python3
"""Inspect installed Nordic MPSL support for external clock management.

This is a read-only capability check. No MCU code, binary or library is built.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SYMBOL = "mpsl_clock_ctrl_source_register"
UNREGISTER = "mpsl_clock_ctrl_source_unregister"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nrfxlib", type=Path,
                        default=ROOT / "external/sdk-nrfxlib")
    parser.add_argument("--nm", default=None,
                        help="Arm nm executable (optional)")
    parser.add_argument("--target", default="nrf52",
                        help="MPSL archive target directory (default: nrf52)")
    args = parser.parse_args()

    header = args.nrfxlib / "mpsl/include/mpsl_clock.h"
    if not header.is_file():
        print("MISSING MPSL header:", header)
        return 2

    source = header.read_text(encoding="utf-8")
    for name in (SYMBOL, UNREGISTER,
                 "mpsl_clock_lfclk_ctrl_source_t",
                 "mpsl_clock_hfclk_ctrl_source_t"):
        found = re.search(r"\b" + re.escape(name) + r"\b", source) is not None
        print("HEADER:", name, "PRESENT" if found else "MISSING")
        if not found:
            return 2

    directory = args.nrfxlib / "mpsl/lib" / args.target
    archives = sorted(directory.rglob("*.a"))
    if not archives:
        print("MISSING MPSL archives:", directory)
        return 2

    nm = args.nm or shutil.which("arm-none-eabi-nm")
    if not nm:
        for candidate in Path("/opt/xPacks").glob(
                "xpack-arm-none-eabi-*/bin/arm-none-eabi-nm"):
            nm = str(candidate)
            break
    if not nm:
        print("MISSING arm-none-eabi-nm; use --nm to specify a toolchain")
        return 2

    available = []
    for archive in archives:
        result = subprocess.run(
            [nm, "-g", "--defined-only", str(archive)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False)
        if result.returncode != 0:
            print("NM ERROR:", archive, result.stderr.strip())
            return 2
        if re.search(r"\b" + SYMBOL + r"\b", result.stdout):
            available.append(archive)

    if not available:
        print("MISSING exported registration symbol in", len(archives),
              "MPSL archives for", args.target)
        print("External clock-driver integration is not available in the "
              "inspected archives.")
        return 1

    print("EXPORTED external-clock registration:")
    for archive in available:
        print(" ", archive.relative_to(args.nrfxlib))
    print("CAPABILITY FOUND: source-compatible interface and binary export.")
    print("This does not establish correct clock ownership or hardware "
          "behavior. S132 requires separate review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
