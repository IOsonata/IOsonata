#!/usr/bin/env python3
"""Exercise the production SAMD low-frequency Standby clock request on host.

This checks register selection and preserved state. It does not validate
an RTC wake interrupt or sleep current on actual SAMD hardware.
"""
from pathlib import Path
import subprocess
import tempfile

from run_standby_source import body

ROOT = Path(__file__).resolve().parents[2]
clock = (ROOT / "ARM/Microchip/SAMD/src/system_samd.c").read_text(encoding="utf-8")
signature = "bool SamdLowFreqClockStandbyEnable(void)"
function = signature + "\n{" + body(clock, signature) + "}\n"

with tempfile.TemporaryDirectory(prefix="samd-lfclock-") as directory:
	out = Path(directory)
	(out / "lfclock_function.h").write_text(function, encoding="utf-8")
	for optimization in ("-O0", "-O2"):
		binary = out / ("lfclock" + optimization)
		subprocess.run([
			"gcc", "-std=c11", optimization, "-Wall", "-Wextra", "-Werror",
			"-fsanitize=undefined", "-fno-sanitize-recover=all",
			"-I" + str(out),
			str(ROOT / "tests/samd/lfclock_standby_test.c"),
			"-o", str(binary)
		], check=True)
		subprocess.run([str(binary)], check=True)
