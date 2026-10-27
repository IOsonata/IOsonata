#!/usr/bin/env python3
"""Run the production SPIM4 driver through generic SPI/DeviceIntrf on a register model."""
from pathlib import Path
import os
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="spim4-") as temp:
    for opt in ("-O0", "-O2"):
        exe = Path(temp) / "spim4"
        # Host addresses used by the 32-bit DMA registers must be below 4 GiB.
        # -fpermissive permits the target driver's existing pointer casts on x86-64.
        subprocess.run(["g++", "-std=gnu++23", opt, "-g", "-fno-pie", "-no-pie",
            "-fpermissive", "-fno-rtti", "-fno-exceptions", "-fsanitize=address,undefined",
            "-DNRF53_SERIES", "-DNRF5340_XXAA_APPLICATION",
            "-I"+str(HERE/"shim"), "-I"+str(ROOT/"include"),
            "-I"+str(ROOT/"include/coredev"), "-I"+str(ROOT/"ARM/Nordic/include"),
            *[str(ROOT/p) for p in ("ARM/Nordic/src/spi_nrfx.cpp", "src/device_intrf.cpp",
                "src/coredev/spi.cpp", "src/coredev/shared_intrf.cpp", "tests/nrf5340_spi/spim4_test.cpp")],
            "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True, env={**os.environ, "ASAN_OPTIONS":"detect_leaks=0"})
