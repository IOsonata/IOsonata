#!/usr/bin/env python3
"""Compile and exercise the production SAM D11/D21 reset-reason source on host.

The six RCAUSE bit positions are extracted from each target's Microchip PM
component header rather than repeated as assumed values. This is a register
model check, not a target build or board validation.

Run: python3 tests/samd/run_power.py
"""
import os
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ARM/Microchip/SAMD/src/system_power_samd.cpp"
INCLUDE = ROOT / "include"
FIELDS = ("POR", "BOD12", "BOD33", "EXT", "WDT", "SYST")
TARGETS = ("SAMD11", "SAMD21")


def masks_from_header(target):
	path = ROOT / "ARM/Microchip/SAMD" / target / "include/component/pm.h"
	s = path.read_text(encoding="utf-8")
	masks = {}
	for field in FIELDS:
		pat = r"#define\\s+PM_RCAUSE_" + field + r"_Pos\\s+_UINT8_\\((\\d+)\\)"
		match = re.search(pat, s)
		if match is None:
			raise RuntimeError(f"{target}: no RCAUSE bit for {field}")
		masks[field] = 1 << int(match.group(1))
	if len(set(masks.values())) != len(masks):
		raise RuntimeError(f"{target}: duplicate RCAUSE positions")
	return masks


def check_target(target):
	masks = masks_from_header(target)
	with tempfile.TemporaryDirectory(prefix="iosonata_samd_power_") as temp:
		td = Path(temp)
		header = [
			"#ifndef SAMD_TEST_REGS_H",
			"#define SAMD_TEST_REGS_H",
			"#include <stdint.h>",
			"typedef struct { volatile uint8_t PM_RCAUSE; } pm_registers_t;",
			"extern pm_registers_t * const PM_REGS;",
		]
		for field in FIELDS:
			header.append(f"#define PM_RCAUSE_{field}_Msk 0x{masks[field]:02x}U")
		header.append("#endif")
		(td / "samdxxx.h").write_text("\n".join(header) + "\n", encoding="utf-8")

		fixture = r"""
#include <stdint.h>
#include "samdxxx.h"
#include "coredev/system_core_clock.h"

static pm_registers_t s_Pm;
pm_registers_t * const PM_REGS = &s_Pm;

int main(void)
{
	for (unsigned status = 0; status < 128; status++)
	{
		uint32_t expected = 0;
		s_Pm.PM_RCAUSE = (uint8_t)status;
		if (status & (PM_RCAUSE_POR_Msk |
			PM_RCAUSE_BOD12_Msk | PM_RCAUSE_BOD33_Msk))
		{
			expected |= SYSTEM_RESET_POWERON;
		}
		if (status & PM_RCAUSE_EXT_Msk)
		{
			expected |= SYSTEM_RESET_PIN;
		}
		if (status & PM_RCAUSE_WDT_Msk)
		{
			expected |= SYSTEM_RESET_WDT;
		}
		if (status & PM_RCAUSE_SYST_Msk)
		{
			expected |= SYSTEM_RESET_SOFT;
		}
		if (SystemResetReason() != expected ||
			s_Pm.PM_RCAUSE != (uint8_t)status)
		{
			return 1;
		}
	}
	return 0;
}
"""
		(td / "fixture.cpp").write_text(fixture, encoding="utf-8")
		out = td / "power_test"
		cmd = [
			os.environ.get("CXX", "g++"),
			"-std=c++17", "-Wall", "-Wextra", "-Werror",
			"-O2", "-I", str(td), "-I", str(INCLUDE),
			str(SOURCE), str(td / "fixture.cpp"), "-o", str(out)
		]
		subprocess.run(cmd, check=True)
		subprocess.run([str(out)], check=True)
	print(f"{target}: RCAUSE 128 combinations PASS")


def main():
	for target in TARGETS:
		check_target(target)
	print("SAM D11/D21 reset reason host tests PASS")


if __name__ == "__main__":
	main()
