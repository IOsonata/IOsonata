#!/usr/bin/env python3
"""Compile Nordic power and clock sources against installed Nordic MDK headers.

This is a focused production-source compilation matrix, not an Eclipse
library/application link or physical-power test. Uses the repository's
existing Arm GNU toolchain and external Nordic SDK layout.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
EXTERNAL = ROOT.parent / "external"

TARGETS = (
	("nrf52805", "NRF52805_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52810", "NRF52810_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52811", "NRF52811_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52820", "NRF52820_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52832", "NRF52832_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52832_s132", "NRF52832_XXAA", "cortex-m4", "nrf52", "s132"),
	("nrf52832_sdc", "NRF52832_XXAA", "cortex-m4", "nrf52", "sdc"),
	("nrf52833", "NRF52833_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52840", "NRF52840_XXAA", "cortex-m4", "nrf52", "bare"),
	("nrf52840_s140", "NRF52840_XXAA", "cortex-m4", "nrf52", "s140"),
	("nrf52840_sdc", "NRF52840_XXAA", "cortex-m4", "nrf52", "sdc"),
	("nrf5340_app", "NRF5340_XXAA_APPLICATION", "cortex-m33", "nrf53", "bare"),
	("nrf5340_net", "NRF5340_XXAA_NETWORK", "cortex-m33+nodsp", "nrf53", "sdc"),
	("nrf54l15", "NRF54L15_XXAA", "cortex-m33", "nrf54l", "bare"),
	("nrf54lm20a", "NRF54LM20A_XXAA", "cortex-m33", "nrf54l", "bare"),
	("nrf54lm20b", "NRF54LM20B_XXAA", "cortex-m33", "nrf54l", "bare"),
	("nrf9160", "NRF9160_XXAA", "cortex-m33", "nrf91", "bare"),
	("nrf9120", "NRF9120_XXAA", "cortex-m33", "nrf91", "bare"),
	("nrf7120", "NRF7120E_XXAA", "cortex-m33", "nrf71", "bare"),
)

def main():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--compiler", default=os.environ.get(
		"ARM_GXX", "arm-none-eabi-g++"))
	parser.add_argument("--nrfx", type=Path, default=Path(
		os.environ.get("NRFX_HOME", EXTERNAL / "nrfx")))
	parser.add_argument("--nrf5-sdk", type=Path, default=Path(
		os.environ.get("NRF5_SDK_HOME", EXTERNAL / "nRF5_SDK")))
	parser.add_argument("--nrfxlib", type=Path, default=Path(
		os.environ.get("NRFXLIB_HOME", EXTERNAL / "sdk-nrfxlib")))
	parser.add_argument("--sdk-config", type=Path,
		help="SoftDevice application sdk_config.h; default uses the SDK5 UART example per chip")
	parser.add_argument("--target", action="append",
		help="restrict to a target label (repeatable)")
	args = parser.parse_args()

	compiler = shutil.which(args.compiler)
	if compiler is None:
		parser.error("Arm compiler not found: " + args.compiler)
	mdk = args.nrfx.resolve() / "bsp/stable/mdk"
	required = [mdk / "nrf.h", ROOT / "ARM/CMSIS/Core/Include/cmsis_compiler.h"]
	for path in required:
		if not path.is_file():
			parser.error("Missing required header: " + str(path))
	known = {entry[0] for entry in TARGETS}
	if args.target and any(name not in known for name in args.target):
		parser.error("Unknown target. Available: " + ", ".join(sorted(known)))

	includes = [
		ROOT / "include",
		ROOT / "ARM/include",
		ROOT / "ARM/Nordic/include",
		ROOT / "ARM/CMSIS/Core/Include",
		mdk,
		mdk / "common",
		mdk / "nrf52",
		mdk / "nrf53",
		mdk / "nrf54",
		mdk / "nrf71",
		mdk / "nrf91",
		args.nrfx.resolve(),
		args.nrfx.resolve() / "hal",
		args.nrfxlib.resolve() / "mpsl/include",
		args.nrfxlib.resolve() / "softdevice_controller/include",
	]
	sources = {
		"nrf52": ("ARM/Nordic/nRF52/src/system_power_nrf52.cpp",
			"ARM/Nordic/src/system_clock_nrfx.cpp"),
		"nrf53": ("ARM/Nordic/src/system_power_nrfx.cpp",
			"ARM/Nordic/src/system_clock_nrfx.cpp"),
		"nrf54l": ("ARM/Nordic/src/system_power_nrfx.cpp",
			"ARM/Nordic/src/system_clock_nrfx.cpp"),
		"nrf91": ("ARM/Nordic/src/system_power_nrfx.cpp",
			"ARM/Nordic/src/system_clock_nrfx.cpp"),
		# nRF71 has no validated shared power backend yet. Compile the
		# initial port's actual sleep file and linked clock source.
		"nrf71": ("ARM/Nordic/nRF71/src/system_sleep_nrf71.c",
			"ARM/Nordic/src/system_clock_nrfx.cpp"),
	}
	errors = 0
	for label, chip, cpu, family, stack in TARGETS:
		if args.target and label not in args.target:
			continue
		for opt in ("-O0", "-Os"):
			base = [compiler, "-std=gnu++23", "-fno-exceptions",
				"-fno-rtti", "-fsyntax-only", "-Wall", "-Wextra",
				"-mcpu=" + cpu, "-mthumb", opt, "-D" + chip]
			if family == "nrf54l" or family == "nrf71":
				base.append("-DNRF_APPLICATION")
			if family == "nrf52":
				base.append("-DNRF52_SERIES")
			if family == "nrf53":
				base.append("-DNRF53_SERIES")
			if family == "nrf54l":
				base.append("-DNRF54L_SERIES")
			if family == "nrf91":
				base.append("-DNRF91_SERIES")
			if family == "nrf71":
				base.append("-DNRF71_SERIES")
			if stack.startswith("s1"):
				sd = args.nrf5_sdk.resolve() / (
					"components/softdevice/" + stack + "/headers")
				if not (sd / "nrf_soc.h").is_file():
					parser.error("Missing SoftDevice: " + str(sd))
				config = args.sdk_config
				if config is None:
					board = "nRF52832" if chip == "NRF52832_XXAA" else "nRF52840"
					config = ROOT / ("ARM/Nordic/nRF52/" + board +
						"/exemples/UartPrbsTxSdk5/src/sdk_config.h")
				config = config.resolve()
				if not config.is_file():
					parser.error("Missing SoftDevice SDK config: " + str(config))
				base += ["-I" + str(config.parent), "-DSOFTDEVICE_PRESENT=1",
					"-I" + str(sd), "-I" + str(sd / "nrf52"),
					"-I" + str(args.nrf5_sdk.resolve() /
						"components/softdevice/common"),
					"-I" + str(args.nrf5_sdk.resolve() /
						"components/softdevice/common/nrf_sdh")]
			if stack == "sdc":
				base.append("-DNRFXLIB_SDC")
			base += ["-I" + str(path) for path in includes if path.is_dir()]
			for source in sources[family]:
				file = ROOT / source
				flags = list(base)
				if file.suffix == ".c":
					ccompiler = str(Path(compiler).with_name("arm-none-eabi-gcc"))
					if not Path(ccompiler).is_file():
						parser.error("Arm C compiler not found: " + ccompiler)
					flags[0] = ccompiler
					flags[1] = "-std=gnu17"
					flags.remove("-fno-rtti")
					flags.remove("-fno-exceptions")
				cmd = flags + [str(file)]
				result = subprocess.run(cmd, capture_output=True, text=True)
				if result.returncode:
					errors += 1
					print(label + " " + opt + " " + file.name + ": FAIL",
						file=sys.stderr)
					print(result.stdout + result.stderr, file=sys.stderr)
				else:
					print(label + " " + opt + " " + file.name + ": PASS",
						flush=True)
	print("Nordic focused power/clock syntax matrix:",
		"PASS" if not errors else str(errors) + " FAIL")
	return 1 if errors else 0

if __name__ == "__main__":
	sys.exit(main())
