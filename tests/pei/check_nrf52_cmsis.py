#!/usr/bin/env python3
"""Syntax-check the nRF52 PEI driver against installed Nordic CMSIS headers.

This does not compile or link any nrfx HAL driver. It verifies actual nrf.h
and SoftDevice/SDC declarations rather than mock-register definitions.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def sdk_path(env_name, default):
    return Path(os.environ.get(env_name, default)).expanduser().resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nrfx", type=Path, help="installed nrfx directory")
    parser.add_argument("--nrf5-sdk", type=Path, help="installed nRF5_SDK directory")
    parser.add_argument("--nrfxlib", type=Path, help="installed sdk-nrfxlib directory")
    parser.add_argument("--compiler", default="arm-none-eabi-gcc", help="Arm GNU C compiler")
    args = parser.parse_args()

    ext = ROOT.parent / "external"
    nrfx = (args.nrfx or sdk_path("NRFX_HOME", ext / "nrfx")).resolve()
    sdk = (args.nrf5_sdk or sdk_path("NRF5_SDK_HOME", ext / "nRF5_SDK")).resolve()
    nrfxlib = (args.nrfxlib or sdk_path("NRFXLIB_HOME", ext / "sdk-nrfxlib")).resolve()
    mdk = nrfx / "bsp/stable/mdk"
    cmsis = ROOT / "ARM/CMSIS/Core/Include"
    driver = ROOT / "ARM/Nordic/nRF52/src/pei_nrf52.c"

    for required in (mdk / "nrf.h", cmsis / "cmsis_compiler.h", driver):
        if not required.is_file():
            parser.error("Missing required file: " + str(required))
    compiler = shutil.which(args.compiler)
    if compiler is None:
        parser.error("Arm compiler not found: " + args.compiler)

    common = [
        compiler, "-fsyntax-only", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
        "-mcpu=cortex-m4", "-mthumb", "-D__PROGRAM_START",
        "-I", str(ROOT / "include"),
        "-I", str(cmsis),
        "-I", str(mdk),
        "-I", str(mdk / "common"),
        "-I", str(mdk / "nrf52")
    ]
    variants = (
        ("nrf52805", "NRF52805_XXAA", "bare"),
        ("nrf52810", "NRF52810_XXAA", "bare"),
        ("nrf52832", "NRF52832_XXAA", "bare"),
        ("nrf52832_s132", "NRF52832_XXAA", "s132"),
        ("nrf52832_sdc", "NRF52832_XXAA", "sdc"),
        ("nrf52840", "NRF52840_XXAA", "bare"),
        ("nrf52840_s140", "NRF52840_XXAA", "s140"),
        ("nrf52840_sdc", "NRF52840_XXAA", "sdc")
    )
    for label, target, stack in variants:
        cmd = [*common, "-D" + target]
        if stack in ("s132", "s140"):
            softdevice = sdk / "components/softdevice" / stack / "headers"
            if not (softdevice / "nrf_soc.h").is_file():
                parser.error("Missing SoftDevice headers: " + str(softdevice))
            cmd += [
                "-DSOFTDEVICE_PRESENT", "-DSVC_INTERFACE_CALL_AS_NORMAL_FUNCTION",
                "-I", str(softdevice), "-I", str(sdk / "components/softdevice/common")
            ]
        elif stack == "sdc":
            sdc = nrfxlib / "softdevice_controller/include"
            mpsl = nrfxlib / "mpsl/include"
            if not (sdc / "sdc_soc.h").is_file() or not (mpsl / "mpsl_hwres.h").is_file():
                parser.error("Missing SDC or MPSL headers: " + str(nrfxlib))
            cmd += ["-DNRFXLIB_SDC", "-I", str(sdc), "-I", str(mpsl)]
        cmd.append(str(driver))
        result = subprocess.run(cmd, text=True, capture_output=True)
        if result.returncode != 0:
            print(label + ": FAIL", file=sys.stderr)
            print(result.stdout + result.stderr, file=sys.stderr)
            return 1
        print(label + ": PASS")
    print("PEI nRF52 CMSIS header compile check: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
