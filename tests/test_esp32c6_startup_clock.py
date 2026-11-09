"""Execute the C6 startup code against mapped host memory (Linux only)."""
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
        with tempfile.TemporaryDirectory(prefix="c6-clock-") as tmp:
            binary = Path(tmp) / "clock_test"
            subprocess.run([
                "cc", "-I.", "-Iinclude", "-ffunction-sections",
                "-fdata-sections", "tests/esp32/c6_startup_clock_test.c",
                "-Wl,--gc-sections", "-o", str(binary),
            ], cwd=ROOT, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
