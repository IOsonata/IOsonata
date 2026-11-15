#!/usr/bin/env python3
"""Run display SysLog tests with address and undefined-behavior sanitizers."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="display-intrf-") as directory:
    out = Path(directory)
    for opt in ("-O0", "-Os"):
        flags = [opt, "-ffunction-sections", "-fdata-sections",
                 "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                 "-I" + str(ROOT / "tests/display/hostport"),
                 "-I" + str(ROOT / "include")]
        objects = []
        for source in ("src/cfifo.c", "src/display/fonts/syst5x7.c"):
            obj = out / (Path(source).stem + ".o")
            subprocess.run([os.environ.get("CC", "gcc"), *flags, "-c",
                            str(ROOT / source), "-o", str(obj)], check=True)
            objects.append(str(obj))
        sources = ["tests/display/display_intrf_test.cpp", "src/display/display_intrf.cpp",
                   "src/display/display.cpp", "src/device.cpp", "src/device_intrf.cpp",
                   "src/syslog.cpp"]
        exe = out / "display_intrf_test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=gnu++23", "-fno-rtti", *flags,
                        "-Wall", "-Wextra", "-Werror", "-Wl,--gc-sections",
                        *[str(ROOT / source) for source in sources], *objects,
                        "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

        console = out / "samd11_console_test"
        sources[0] = "tests/display/samd11_console_test.cpp"
        sources.append("src/display/display_ssd1315.cpp")
        subprocess.run([os.environ.get("CXX", "g++"), "-std=gnu++23", "-fno-rtti",
                        *flags, "-Wall", "-Wextra", "-Werror",
                        "-Wno-missing-field-initializers", "-Wl,--gc-sections",
                        "-I" + str(ROOT / "ARM/Microchip/SAMD/SAMD11/exemples/LCDConsoleDisplayDemo/src"),
                        *[str(ROOT / source) for source in sources], *objects,
                        "-o", str(console)], check=True)
        subprocess.run([str(console)], check=True)
