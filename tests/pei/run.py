#!/usr/bin/env python3
"""Compile and exercise the real nRF52 PEI source against mock PPI registers."""
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
TEST = pathlib.Path(__file__).resolve().parent
COMMON = [
    "-Wall", "-Wextra", "-Werror", "-pedantic", "-g", "-O1",
    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
    "-I", str(TEST / "shim"), "-I", str(ROOT / "include"),
]

subprocess.run([sys.executable, str(TEST / "test_check_no_pei_link.py")],
               check=True)
subprocess.run([sys.executable, str(TEST / "check_timer_cc_invariant.py")],
               check=True)
subprocess.run([sys.executable, str(TEST / "check_gpio_port_contract.py")],
               check=True)

with tempfile.TemporaryDirectory(prefix="pei-nrf52-") as temp:
    temp = pathlib.Path(temp)
    driver = ROOT / "ARM/Nordic/nRF52/src/pei_nrf52.cpp"
    matrix = ROOT / "src/coredev/pei.c"
    matrix_obj = temp / "pei_matrix.o"
    subprocess.run(["gcc", "-std=c11", *COMMON, "-c",
                    str(matrix), "-o", str(matrix_obj)], check=True)
    subprocess.run(["gcc", "-std=c11", *COMMON, "-fsyntax-only",
                    str(TEST / "pei_hal_c_api.c")], check=True)
    subprocess.run(["g++", "-std=c++11", *COMMON, "-fsyntax-only",
                    str(TEST / "pei_hal_cpp_api.cpp")], check=True)
    for name, flags in [
        ("bare", []),
        ("sdc", ["-DNRFXLIB_SDC"]),
        ("s132", ["-DSOFTDEVICE_PRESENT"]),
        ("s140", ["-DSOFTDEVICE_PRESENT", "-DS140", "-DNRF52840_XXAA"]),
        ("nrf52805", ["-DTEST_PPI_CHANNEL_COUNT=10", "-DTEST_EGU_COUNT=2"]),
    ]:
        output = temp / name
        driver_obj = temp / (name + "_pei.o")
        test_obj = temp / (name + "_test.o")
        subprocess.run(["g++", "-std=c++11", *COMMON, *flags,
                        "-c", str(driver), "-o", str(driver_obj)], check=True)
        subprocess.run(["gcc", "-std=c11", *COMMON, *flags,
                        "-c", str(TEST / "ppi_nrf52_test.c"),
                        "-o", str(test_obj)], check=True)
        subprocess.run(["g++", *COMMON, str(driver_obj), str(test_obj),
                        str(matrix_obj), "-o", str(output)], check=True)
        subprocess.run([str(output)], check=True)

    obj = temp / "pei_nrf52.o"
    subprocess.run(["g++", "-std=c++11", *COMMON,
                    "-c", str(driver), "-o", str(obj)], check=True)
    cpp = temp / "cpp"
    subprocess.run(["g++", "-std=c++11", *COMMON,
                    str(TEST / "ppi_cpp_test.cpp"), str(obj), str(matrix_obj),
                    "-o", str(cpp)], check=True)
    subprocess.run([str(cpp)], check=True)

print("PEI nRF52 host validation: PASS")
