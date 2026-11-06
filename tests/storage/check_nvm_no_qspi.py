#!/usr/bin/env python3
"""Check that the nRF52840 internal-flash MSC image does not link QSPI/SPI.

Usage:
    python3 tests/storage/check_nvm_no_qspi.py path/to/UsbMscFlashDisk.map

Run after rebuilding the Release HAL archive and the MSC application.
"""
import pathlib
import sys


def main():
    if len(sys.argv) != 2:
        print("usage: check_nvm_no_qspi.py UsbMscFlashDisk.map")
        return 2

    text = pathlib.Path(sys.argv[1]).read_text(errors="replace")
    start = text.find("Archive member included to satisfy reference")
    end = text.find("Discarded input sections", start)

    if start < 0 or end < 0:
        print("FAIL: GNU ld archive-extraction section not found")
        return 1

    included = text[start:end]
    unexpected = [
        member for member in ("qspi_nrfx.o", "spi_nrfx.o")
        if member in included
    ]
    if unexpected:
        print("FAIL: unused QSPI/SPI archive members linked: " +
              ", ".join(unexpected))
        return 1

    print("PASS: internal-flash MSC did not pull QSPI or SPI from the HAL")
    return 0


if __name__ == "__main__":
    sys.exit(main())
