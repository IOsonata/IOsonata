#!/usr/bin/env python3
"""Host checks of the production pin-assignment function and board mappings."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def run(*args):
	subprocess.run(args, check=True)


def function(path, name):
	text = (ROOT / path).read_text()
	start = text.index(name + "(")
	start = text.rfind("\n", 0, start) + 1
	brace = text.index("{", start)
	depth = 1
	end = brace + 1
	while depth:
		depth += (text[end] == "{") - (text[end] == "}")
		end += 1
	return text[start:end] + "\n"


def single_core_checks(build):
	# Use production validation helpers; only hardware addresses/counts are mocked.
	cases = [
		("ARM/Microchip/SAM4E/src/iopincfg_sam4e.c", "#define IOPIN_MAX_PORT 5\n", ""),
		("ARM/Microchip/SAM4L/src/iopincfg_sam4l.c", "#define IOPIN_MAX_PORT 3\n", ""),
		("ARM/NXP/LPC11xx/src/iopincfg_lpc11uxx.c",
		 function("ARM/NXP/LPC11xx/include/iopinctrl.h", "Lpc11GpioValidPin"),
		 "assert(!IOPinAssignCore(1, 30, 0));"),
		("ARM/NXP/LPC17xx/src/iopincfg.c", "", ""),
		("ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp",
		 "#define LPC546XX_GPIO_PORT_MAX 6U\n#define LPC546XX_GPIO_PIN_MAX 32U\n", ""),
		("ARM/Renesas/RE01/src/iopincfg_re01.c",
		 "#define RE01_1500KB_MAX_PORT 9\n" +
		 function("ARM/Renesas/RE01/src/iopincfg_re01.c", "IsValidIOPin"), ""),
		("ARM/Renesas/RA4M1/src/iopincfg_ra4m1.c",
		 "#define RA4M1_PACKAGE_PINS 100\n" +
		 function("ARM/Renesas/RA4M1/include/iopinctrl.h", "Ra4m1PortMask") +
		 function("ARM/Renesas/RA4M1/include/iopinctrl.h", "Ra4m1PinValid"), ""),
		("RISCV/Renesas/R9A02/src/iopincfg_r9a02.c",
		 "#define R9A02_PACKAGE_PINS 32\n#define R9A02_PORT_COUNT 5\n#define R9A02_PORT_PIN_COUNT 16\n" +
		 function("RISCV/Renesas/R9A02/include/iopinctrl.h", "R9a02PortMask") +
		 function("RISCV/Renesas/R9A02/include/iopinctrl.h", "R9a02PinValid"), ""),
		("ARM/ST/src/iopincfg_stm32.cpp",
		 "typedef int GPIO_TypeDef;\nstatic int port;\n#define GPIOA (&port)\n" +
		 function("ARM/ST/include/iopinctrl.h", "Stm32Gpio"), ""),
		("RISCV/Espressif/src/iopincfg_esp32.c",
		 "#define ESP32C3\n#define ESP32_GPIO_PIN_COUNT 22U\n" +
		 function("RISCV/Espressif/src/iopincfg_esp32.c", "ESP32IsValidGpioPin"), ""),
	]
	for path, helpers, extra in cases:
		check = build / "single.c"
		check.write_text('#include <assert.h>\n#include "coredev/iopincfg.h"\n' + helpers +
			function(path, "IOPinAssignCore") + '''
int main(void) {
	assert(IOPinAssignCore(0, 0, 0));
	assert(!IOPinAssignCore(0, 0, 1));
	assert(!IOPinAssignCore(0, 0, -1));
	assert(!IOPinAssignCore(-1, 0, 0));
	assert(!IOPinAssignCore(99, 0, 0));
	assert(!IOPinAssignCore(0, -1, 0));
	assert(!IOPinAssignCore(0, 32, 0));
''' + extra + "\nreturn 0;\n}\n")
		for compiler in ["gcc", "g++"]:
			binary = build / "single"
			run(compiler, "-Os", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
				f"-I{HERE / 'shim'}", f"-I{ROOT / 'include'}", str(check), "-o", str(binary))
			run(str(binary))
	print(f"PASS C/C++ core 0 and invalid arguments in {len(cases)} single-core ports")


def main():
	source = (ROOT / "ARM/Nordic/src/iopincfg_nrfx.c").read_text()
	function = source[source.index("bool IOPinAssignCore("):]
	variants = {
		"app": ["NRF5340_XXAA_APPLICATION"],
		"app_ns": ["NRF5340_XXAA_APPLICATION", "NRF_TRUSTZONE_NONSECURE"],
		"net": ["NRF5340_XXAA_NETWORK"],
		"nrf52": ["NRF52_SERIES"],
		"nrf54h": ["NRF54H20_XXAA"],
		"nrf54l": ["NRF54L15_XXAA"],
		"nrf54lm": ["NRF54LM20A_XXAA"],
	}
	with tempfile.TemporaryDirectory() as temporary:
		build = Path(temporary)
		single_core_checks(build)
		for compiler in ["gcc", "g++"]:
			run(compiler, "-Os", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
				f"-I{HERE / 'shim'}", f"-I{ROOT / 'include'}",
				str(HERE / "core_cfg_test.c"), "-o", str(build / "batch"))
			run(str(build / "batch"))
		print("PASS C/C++ batch order, empty/invalid input and failure propagation")
		(build / "assign_core.inc").write_text(function)
		for compiler, standard in [("gcc", "c11"), ("g++", "c++11")]:
			for optimization in ["-O0", "-Os"]:
				for name, defines in variants.items():
					binary = build / "test"
					run(compiler, f"-std={standard}", optimization, "-Wall", "-Wextra",
						"-Werror", "-fsanitize=undefined", "-fno-sanitize-recover=all",
						*[f"-D{d}" for d in defines], f"-I{HERE / 'shim'}",
						f"-I{ROOT / 'include'}", f"-I{build}",
						str(HERE / "assign_core_test.c"), "-o", str(binary))
					run(str(binary))
				print(f"PASS {compiler} {optimization}: {len(variants)} core configurations")
		# Compile and run the board tables through the generic batch helper.
		(build / "blueio_board.h").write_text("")
		for example in ["NetCoreStart", "BleAdvertiser", "BleSecurityDemo", "BleTestDut", "UartBleDemo"]:
			board = ROOT / f"ARM/Nordic/nRF53/nRF5340_App/exemples/{example}/src/board.h"
			for standard_dk in [False, True]:
				check = build / "board.c"
				check.write_text('#include <assert.h>\n#include "coredev/iopincfg.h"\n' +
					f'#include "{board}"\n' + '''
static const IOPinCoreCfg_t s_Pins[] = CORE_PINS;
static int s_Calls;
bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	assert(CoreNo == 1 && PortNo == 1);
	assert(PinNo == EXPECT_RX + s_Calls);
	s_Calls++;
	return true;
}
int main(void)
{
	assert(sizeof(s_Pins) / sizeof(s_Pins[0]) == 2);
	assert(IOPinCoreCfg(s_Pins, 2));
	assert(s_Calls == 2);
	return 0;
}
''')
				for compiler in ["gcc", "g++"]:
					run(compiler, "-Wall", "-Wextra", "-Werror", f"-I{build}",
						f"-I{HERE / 'shim'}", f"-I{ROOT / 'include'}",
						f"-DEXPECT_RX={0 if standard_dk else 8}",
						*(["-DNRF5340_DK_PCA10095"] if standard_dk else []),
						str(check), "-o", str(build / "board"))
					run(str(build / "board"))
		print("PASS C/C++ Audio DK and standard DK CORE_PINS in all five examples")


if __name__ == "__main__":
	main()
