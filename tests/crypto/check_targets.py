#!/usr/bin/env python3
"""Compile target crypto objects against real class headers, without MCU registers."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
VARIANTS = [
    ("nrf52805", ["NRF52805_XXAA"]),
    ("nrf52810", ["NRF52810_XXAA"]),
    ("nrf52832", ["NRF52832_XXAA", "NRF52_SERIES"]),
    ("nrf52840", ["NRF52840_XXAA", "NRF52_SERIES"]),
    ("nrf5340_net", ["NRF5340_XXAA_NETWORK"]),
    ("nrf5340_app", ["NRF5340_XXAA_APPLICATION"]),
    ("nrf91", ["NRF91_SERIES"]),
    ("nrf54l15", ["NRF54L15_XXAA"]),
    ("nrf54lm20", ["NRF54LM20A_XXAA"]),
    ("nrf54h20", ["NRF54H20_XXAA"]),
    ("stm32h753", ["STM32H753xx", "IOSONATA_STM32_HAS_RNG=1"]),
    ("stm32wba65", ["STM32WBA65xx", "IOSONATA_STM32_HAS_RNG=1"]),
    ("stm32l4s9", ["STM32L4S9xx", "IOSONATA_STM32_HAS_RNG=1"]),
    ("stm32l4", ["STM32L476xx", "IOSONATA_STM32_HAS_RNG=1"]),
    ("stm32g4", ["STM32G474xx", "IOSONATA_STM32_HAS_RNG=1"]),
    ("stm32f401", ["STM32F401xx"]),
]

with tempfile.TemporaryDirectory(prefix="crypto-targets-") as temp:
    out = Path(temp)
    (out / "nrf.h").write_text("")
    (out / "stm32.h").write_text(
        "#ifndef IOSONATA_STM32_HAS_RNG\n#define IOSONATA_STM32_HAS_RNG 0\n#endif\n")
    includes = ["-I" + str(p) for p in (
        out, ROOT / "include", ROOT / "ARM/include", ROOT / "ARM/Nordic/include",
        ROOT / "ARM/ST/include")]
    for name, definitions in VARIANTS:
        source = ROOT / "ARM" / ("Nordic/src/crypto_default_nrfx.cpp"
            if name.startswith("nrf") else "ST/src/crypto_default_stm32.cpp")
        obj = out / (name + ".o")
        subprocess.run(["g++", "-std=gnu++23", "-O2", "-Wall", "-Wextra", "-Werror",
            "-fno-rtti", "-fno-exceptions", *includes,
            *["-D" + define for define in definitions], "-c", str(source), "-o", str(obj)],
            check=True)
        symbols = subprocess.check_output(["nm", "-C", str(obj)], text=True)
        initializers = [line for line in symbols.splitlines() if " T Crypto::Init()" in line]
        assert len(initializers) == 1, (name, initializers)
        if name in ("nrf52840", "nrf5340_app", "nrf91"):
            assert "CryptoCc3xxSym::Cipher" in symbols
            assert "CryptoCc3xxSym::HashUpdate" in symbols
            assert "CryptoCc3xx::Agree" in symbols
        assert "CryptoCreate" not in symbols
        assert "CryptoCipherInstance" not in symbols
        print(name, "direct object and target Init compile")
