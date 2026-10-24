#!/usr/bin/env python3
"""Measure a minimal software Crypto image, excluding test/reference engines.

Requires host GCC/binutils. Values are host comparisons, not MCU flash or RAM.
Use --root to measure another checkout with the same compiler and fixture.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

SOURCES = ["src/device.cpp", "src/device_intrf.cpp", "src/crypto/crypto.cpp",
           "src/crypto/crypto_default.cpp", "src/crypto/crypto_aes_modes.cpp",
           "src/crypto/crypto_aes_soft.cpp", "src/crypto/crypto_softsha256.cpp",
           "src/crypto/crypto_softrng.cpp", "src/crypto/crypto_uecc.cpp",
           "src/crypto/crypto_p256.cpp", "micro-ecc/uECC.c"]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
args = parser.parse_args()
root = args.root.resolve()

with tempfile.TemporaryDirectory(prefix="crypto-size-") as temp:
    out = Path(temp)
    flags = ["-Os", "-ffunction-sections", "-fdata-sections", "-fstack-usage",
             "-DuECC_ENABLE_VLI_API=1", "-I" + str(root / "include"),
             "-I" + str(root / "micro-ecc")]
    objects = []
    for source in SOURCES:
        obj = out / (Path(source).stem + ".o")
        compiler = ["gcc", "-std=gnu17"] if source.endswith(".c") else [
            "g++", "-std=gnu++23", "-fno-rtti", "-fno-exceptions"]
        subprocess.run([*compiler, *flags, "-c", str(root / source), "-o", str(obj)], check=True)
        objects.append(str(obj))
    fixture = out / "main.cpp"
    fixture.write_text('#include "crypto/crypto.h"\n'
                       'int main() { Crypto c; return c.Init() ? c.SelfTest() : 1; }\n')
    exe = out / "minimal"
    subprocess.run(["g++", "-std=gnu++23", "-fno-rtti", "-fno-exceptions", *flags,
                    str(fixture), *objects, "-Wl,--gc-sections", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    subprocess.run(["size", str(exe)], check=True)
    for line in (out / "crypto_softsha256.su").read_text().splitlines():
        if "Sha256Block" in line or "CryptoHmacSha256" in line:
            print(line)
    symbols = subprocess.check_output(["nm", "-C", str(exe)], text=True)
    for name in ("CryptoSoftSha256::SelfTest", "vtable for CryptoSoftSha256"):
        print(name + (": retained" if any(line.endswith(name) for line in symbols.splitlines())
                      else ": discarded"))
