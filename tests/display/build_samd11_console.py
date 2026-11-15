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
    parser.add_argument("--no-lto", action="store_true", help="compare a normal object build")
    parser.add_argument("--dma", action="store_true")
    parser.add_argument("--interrupt", action="store_true")
    args = parser.parse_args()
    app = ("tests/display/syslog_resource_probe.cpp" if args.syslog_only else
           "exemples/display/console_display_demo.cpp")
    name = "SysLogResourceProbe" if args.syslog_only else "LCDConsoleDisplayDemo"
    failures = 0
    for mode in ("Debug", "Release"):
        out = Path(tempfile.gettempdir()) / "iosonata-oled1-build" / name / args.output / mode
        if args.dma or args.interrupt:
            out = out / ("dma%d-int%d" % (args.dma, args.interrupt))
        if args.no_lto:
            out = out / "no-lto"
        out.mkdir(parents=True, exist_ok=True)
        flags = ["-mcpu=cortex-m0plus", "-mthumb", "-Os", "-fsigned-char",
                 "-ffunction-sections", "-fdata-sections", "-fstack-usage",
                 "-D__SAMD11D14AM__", "-D__PROGRAM_START"]
        flags += ["-DDISPLAY_SPI_DMA_ENABLE=" + str(int(args.dma)),
                  "-DDISPLAY_SPI_INT_ENABLE=" + str(int(args.interrupt))]
        flags += ["-DCONSOLE_LOG_OUTPUT=" + ("1" if args.output == "uart" else "2")]
        flags += ["-I" + str(path) for path in INCLUDES]
        flags += ["-g3"] if mode == "Debug" else []
        objects = []
        for index, source in enumerate(SOURCES + [app]):
            obj = out / (str(index) + ".o")
            cpp = source.endswith(".cpp")
            language = (["-std=gnu++23", "-fno-exceptions", "-fno-rtti"] if cpp
                        else ["-std=gnu17"])
            # GCC emits M0+ atomic helper references after LTO. Keep their
            # archive member native so the linker can resolve them then.
            lto = ([] if args.no_lto or source == "ARM/src/iatomic.c" else
                   ["-flto", "-ffat-lto-objects"])
            app_flags = ["-DNDEBUG"] if source == app and mode == "Release" else []
            subprocess.run([PREFIX + ("g++" if cpp else "gcc"), *flags, *language, *lto, *app_flags,
                            "-c", str(ROOT / source), "-o", str(obj)], check=True)
            objects.append(str(obj))
        if not args.syslog_only:
            app_symbols = subprocess.check_output([PREFIX + "nm", objects[-1]], text=True)
            absent = ("g_Lcd", "g_Spi", "DisplayIntrf") if args.output == "uart" else ("s_Uart", "UARTInit")
            assert all(symbol not in app_symbols for symbol in absent), "Unselected output compiled"
        archive = out / "libIOsonata_SAMD11D14AM.a"
        archive.unlink(missing_ok=True)
        subprocess.run([PREFIX + "ar", "rcs", str(archive), *objects[:-1]], check=True)
        elf = out / (name + ".elf")
        link_lto = [] if args.no_lto else ["-flto"]
        result = subprocess.run([
            PREFIX + "g++", "-mcpu=cortex-m0plus", "-mthumb", "-Os", *link_lto,
            "-ffunction-sections", "-fdata-sections", "--specs=nano.specs",
            "-T", str(TARGET / "ldscript/gcc_samd11d14a.ld"),
            "-L" + str(ROOT / "ARM/ldscript"), "-Wl,--gc-sections",
            "-Wl,-Map," + str(out / (name + ".map")), objects[-1], str(archive),
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
        demangled = subprocess.check_output([PREFIX + "nm", "-C", str(elf)], text=True)
        if not args.syslog_only:
            absent = ("LcdSSD", "SPIInit", "DisplayIntrf") if args.output == "uart" else ("UARTInit", "SamdUart")
            assert all(symbol not in symbols for symbol in absent), "Unselected output linked"
        assert "__Vectors" in symbols and "ResetEntry" in symbols, "Startup missing"
        if not (args.no_lto or args.syslog_only or args.dma or args.interrupt) and args.output == "display":
            absent = ("SamdSpimInt", "SamdSpis", "SamdSpimDma", "SamdDmacOpen",
                      "SamdDmacStart", "s_SamdSpiTxBuf")
            assert all(symbol not in symbols for symbol in absent), "Unused SPI mode linked"
        if args.dma and not args.no_lto and args.output == "display":
            absent = ("SamdSpimPolling", "SamdSpimStart", "SamdSpimIrq", "SamdSpis")
            assert all(symbol not in symbols for symbol in absent), "CPU SPI mode linked with DMA"
            assert "SamdSpiTxData(" not in demangled and "SamdSpiRxData(" not in demangled, "Register entry points linked with DMA"
        if args.syslog_only:
            symbols = subprocess.check_output([PREFIX + "nm", str(elf)], text=True)
            assert "vsnprintf" not in symbols, "Status-only log linked printf"
            assert "SysLogFormatTime" not in symbols, "Unused timestamp code linked"
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
