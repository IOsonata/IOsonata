#!/usr/bin/env python3
"""Compile the production nRF5340 PEI driver against host DPPI registers."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent
SOURCE = ROOT / "ARM/Nordic/nRF53/src/pei_nrf53.c"
COMMON = [
    "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
    "-DPEI_NRF53_TEST",
    "-I", str(TEST / "shim_nrf53"),
    "-I", str(ROOT / "include"),
]

variants = (
    ("app_secure", ["-DNRF5340_XXAA_APPLICATION"]),
    ("app_nonsecure", ["-DNRF5340_XXAA_APPLICATION", "-DNRF_TRUSTZONE_NONSECURE"]),
    ("network", ["-DNRF5340_XXAA_NETWORK"]),
)
with tempfile.TemporaryDirectory(prefix="pei-nrf53-") as dirname:
    for name, flags in variants:
        output = Path(dirname) / name
        cmd = ["gcc", *COMMON, *flags, str(SOURCE),
               str(TEST / "dppi_nrf53_test.c"), "-o", str(output)]
        subprocess.run(cmd, check=True)
        subprocess.run([str(output)], check=True)
        print(name + ": PASS")
print("PEI nRF53 host validation: PASS")
