#!/usr/bin/env python3
"""Prepare pinned, unmodified ST sources for the isolated SPI reference."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

REVISION = '7e541d92019e18f98d211fc4ab9197ec8e8105f6'
BASE = f'https://raw.githubusercontent.com/STMicroelectronics/stm32h7xx-hal-driver/{REVISION}/'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, help='offline HAL driver directory at the pinned revision')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / 'hal_manifest.json').read_text())
    prepared = {}
    for name, digest in manifest.items():
        if args.source:
            data = (args.source / name).read_bytes()
        else:
            with urlopen(BASE + name, timeout=30) as response:
                data = response.read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise SystemExit(f'Checksum mismatch: {name}; no files installed')
        prepared[name] = data
    for name, data in prepared.items():
        path = root / 'vendor' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    print(f'Prepared {len(prepared)} verified ST files at {REVISION}')


if __name__ == '__main__':
    main()
