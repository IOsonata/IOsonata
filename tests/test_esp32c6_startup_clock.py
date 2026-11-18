"""Execute the C3/C6 startup code against mapped host memory (Linux only)."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Esp32C6ClockTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform.startswith("linux") and shutil.which("cc"),
                         "requires Linux mmap and a host C compiler")
    def test_clock_decode_and_startup_registers(self):
        with tempfile.TemporaryDirectory(prefix="esp-clock-") as tmp:
            for chip, fast in (("ESP32C3", False), ("ESP32C3", True), ("ESP32C6", False)):
                for opt in ("-O0", "-Os"):
                    with self.subTest(chip=chip, optimization=opt, fast=fast):
                        binary = Path(tmp) / (chip + opt + str(fast))
                        source = "c6_startup_clock_test.c" if chip == "ESP32C6" else "startup_clock_test.c"
                        defines = [] if chip == "ESP32C6" else ["-D" + chip]
                        if fast:
                            defines.append("-DESP32C3_CPU_160MHZ")
                        subprocess.run([
                            "cc", opt, *defines, "-I.", "-Iinclude", "-ffunction-sections",
                            "-fdata-sections", "tests/esp32/" + source,
                            "-Wl,--gc-sections", "-o", str(binary),
                        ], cwd=ROOT, check=True)
                        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
