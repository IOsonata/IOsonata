#!/usr/bin/env python3
"""Build the D11 OLED console, or a small status-logging resource probe."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PREFIX = os.environ.get("CROSS_COMPILE", "arm-none-eabi-")
TARGET = ROOT / "ARM/Microchip/SAMD/SAMD11"
SOURCES = [
    "src/CppRuntimeOverload.cpp", "ARM/src/ResetEntry.c", "src/cfifo.c",
    "src/device.cpp", "src/device_intrf.cpp", "ARM/src/iatomic.c",
    "src/syslog.cpp", "src/coredev/shared_intrf.cpp", "src/coredev/spi.cpp",
    "src/coredev/uart.cpp",
    "src/display/display.cpp", "src/display/display_ssd1315.cpp",
    "src/display/display_intrf.cpp", "src/display/fonts/syst5x7.c",
]
SOURCES += ["ARM/Microchip/SAMD/src/" + name for name in (
    "system_samd.c", "system_sleep_samd.c", "system_power_samd.cpp",
    "dmac_samd.c", "iopincfg_samd.c", "sercom_samd.c",
    "shared_intrf_samd.cpp", "spi_samd.cpp", "uart_samd.cpp",
)]
SOURCES += ["ARM/Microchip/SAMD/SAMD11/src/vectors_samd11.c"]
INCLUDES = [ROOT / "include", ROOT / "ARM/include",
            ROOT / "ARM/CMSIS/Core/Include", ROOT / "ARM/Microchip/SAMD/include",
            TARGET / "include", TARGET / "exemples/LCDConsoleDisplayDemo/src"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--syslog-only", action="store_true",
                        help="build status logging without a display or printf")
    parser.add_argument("--output", choices=("uart", "display"), default="display",
                        help="select the D11 console log output")
    args = parser.parse_args()
    app = ("tests/display/syslog_resource_probe.cpp" if args.syslog_only else
           "exemples/display/console_display_demo.cpp")
    name = "SysLogResourceProbe" if args.syslog_only else "LCDConsoleDisplayDemo"
    failures = 0
    for mode in ("Debug", "Release"):
        out = Path(tempfile.gettempdir()) / "iosonata-oled1-build" / name / args.output / mode
        out.mkdir(parents=True, exist_ok=True)
        flags = ["-mcpu=cortex-m0plus", "-mthumb", "-Os", "-fsigned-char",
                 "-ffunction-sections", "-fdata-sections", "-fstack-usage",
                 "-D__SAMD11D14AM__", "-D__PROGRAM_START"]
        flags += ["-DCONSOLE_LOG_OUTPUT=" + ("1" if args.output == "uart" else "2")]
        flags += ["-I" + str(path) for path in INCLUDES]
        flags += ["-DNDEBUG"] if mode == "Release" else ["-g3"]
        objects = []
        for index, source in enumerate(SOURCES + [app]):
            obj = out / (str(index) + ".o")
            cpp = source.endswith(".cpp")
            language = (["-std=gnu++23", "-fno-exceptions", "-fno-rtti"] if cpp
                        else ["-std=gnu17"])
            subprocess.run([PREFIX + ("g++" if cpp else "gcc"), *flags, *language,
                            "-c", str(ROOT / source), "-o", str(obj)], check=True)
            objects.append(str(obj))
        if not args.syslog_only:
            app_symbols = subprocess.check_output([PREFIX + "nm", objects[-1]], text=True)
            absent = ("g_Lcd", "g_Spi", "DisplayIntrf") if args.output == "uart" else ("s_Uart", "UARTInit")
            assert all(symbol not in app_symbols for symbol in absent), "Unselected output compiled"
        elf = out / (name + ".elf")
        result = subprocess.run([
            PREFIX + "g++", "-mcpu=cortex-m0plus", "-mthumb", "--specs=nano.specs",
            "-T", str(TARGET / "ldscript/gcc_samd11d14a.ld"),
            "-L" + str(ROOT / "ARM/ldscript"), "-Wl,--gc-sections",
            "-Wl,-Map," + str(out / (name + ".map")), *objects,
            "-Wl,--start-group", "-lgcc", "-lc", "-lm", "-Wl,--end-group",
            "-o", str(elf),
        ])
        if result.returncode:
            failures += 1
            print(mode + ": link failed", flush=True)
            continue
        print(mode + ": link passed", flush=True)
        subprocess.run([PREFIX + "size", str(elf)], check=True)
        symbols = subprocess.check_output([PREFIX + "nm", str(elf)], text=True)
        if not args.syslog_only:
            absent = ("LcdSSD", "SPIInit", "DisplayIntrf") if args.output == "uart" else ("UARTInit", "SamdUart")
            assert all(symbol not in symbols for symbol in absent), "Unselected output linked"
        if args.syslog_only:
            symbols = subprocess.check_output([PREFIX + "nm", str(elf)], text=True)
            assert "vsnprintf" not in symbols, "Status-only log linked printf"
            assert "SysLogFormatTime" not in symbols, "Unused timestamp code linked"
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
