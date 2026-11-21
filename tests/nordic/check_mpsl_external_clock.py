#!/usr/bin/env python3
"""Check external-clock integration in the *installed* Nordic MPSL SDK.

This is a read-only probe. It searches the installed SDK instead of
assuming a particular MPSL header name or release layout.
"""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NRFXLIB = ROOT.parent / "external/sdk-nrfxlib"
REGISTER = "mpsl_clock_ctrl_source_register"
UNREGISTER = "mpsl_clock_ctrl_source_unregister"
REQUIRED = (
    REGISTER,
    UNREGISTER,
    "mpsl_clock_lfclk_ctrl_source_t",
    "mpsl_clock_hfclk_ctrl_source_t",
)
FALLBACK = (
    "mpsl_clock_hfclk_src_request",
    "mpsl_clock_hfclk_src_release",
    "mpsl_clock_hfclk_src_is_running",
)


def find_nm(explicit):
    if explicit:
        return explicit
    command = shutil.which("arm-none-eabi-nm")
    if command:
        return command
    for base in (Path("/opt/xPacks"), Path.home() / ".local/xPacks"):
        for tool in sorted(base.glob(
                "xpack-arm-none-eabi-*/bin/arm-none-eabi-nm"), reverse=True):
            if tool.is_file():
                return str(tool)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nrfxlib", type=Path, default=DEFAULT_NRFXLIB)
    parser.add_argument("--target", default="nrf52")
    parser.add_argument("--nm", help="Path to arm-none-eabi-nm")
    args = parser.parse_args()

    sdk = args.nrfxlib.expanduser().resolve()
    include = sdk / "mpsl/include"
    if not include.is_dir():
        print("MPSL INCLUDE DIRECTORY NOT FOUND:", include)
        print("Specify the actual installed SDK with --nrfxlib PATH.")
        return 2

    headers = sorted(include.rglob("*.h"))
    if not headers:
        print("NO MPSL HEADERS found in", include)
        return 2

    print("MPSL include directory:", include)
    print("Headers scanned:", len(headers))
    found = {}
    for header in headers:
        source = header.read_text(encoding="utf-8", errors="replace")
        for symbol in REQUIRED + FALLBACK:
            if re.search(r"\b" + re.escape(symbol) + r"\b", source):
                found.setdefault(symbol, []).append(
                    str(header.relative_to(sdk)))

    for symbol in REQUIRED + FALLBACK:
        locations = found.get(symbol, [])
        print("HEADER", symbol, "FOUND" if locations else "NOT FOUND",
              ", ".join(locations))

    # Do not conflate a missing declaration with a missing .a symbol.
    archive_root = sdk / "mpsl/lib" / args.target
    archives = sorted(archive_root.rglob("*.a")) if archive_root.is_dir() else []
    if not archives:
        print("NO MPSL ARCHIVES found in", archive_root)
        return 2

    nm = find_nm(args.nm)
    if not nm:
        print("arm-none-eabi-nm not found. Set --nm to inspect binary symbols.")
        return 2

    exported = set()
    for archive in archives:
        result = subprocess.run(
            [nm, "--defined-only", "-g", str(archive)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False)
        if result.returncode != 0:
            print("NM FAILED:", archive, result.stderr.strip())
            return 2
        for symbol in (REGISTER, UNREGISTER):
            if re.search(r"\b" + symbol + r"\b", result.stdout):
                exported.add(symbol)
                print("ARCHIVE", symbol, archive.relative_to(sdk))

    if all(found.get(s) for s in REQUIRED) and all(
            s in exported for s in (REGISTER, UNREGISTER)):
        print("RESULT: External clock driver registration is available.")
        print("This does not validate the callbacks or HFXO handover.")
        return 0

    print("RESULT: External clock driver registration not demonstrated",
          "by the installed SDK headers and archives.")
    print("The generic IOsonata clock API is unchanged.")
    if any(found.get(s) for s in FALLBACK):
        print("Existing MPSL HF source-request API found;",
              "it is not equivalent to registering external clock control.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
