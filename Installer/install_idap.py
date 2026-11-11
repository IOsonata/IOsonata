#!/usr/bin/env python3
"""Install the published IDAP tools and download probe firmware, without flashing.

MIT License. Copyright (c) 2026 I-SYST inc.
Uses only the Python standard library available in the installer's managed Python.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import urllib.request
import zipfile

VERSION = '2_4_260521'
BASE_URL = 'https://downloads.sourceforge.net/project/idaplinkfirmware/'
TOOLS = {
    'macos': ('OSX', '_OSX', (
        '86d33dc934c15de528c51e89395973ebef52db177e91cb555138c863120479d5',
        'dedb865a32903428b33c50afb368f47e14ea64a2a2b157b09baeabd56ff48cdd')),
    'linux': ('Linux', '_Linux', (
        '5f7e800fd632f295f94afb024f842be4678ab39632fdd7b193b38c1f2d05f4b3',
        '5838a950f3ba3ff1baceb32a052c3c1b66d0b7ce333c2d6897d50779b937276e')),
    'windows': ('Windows', '', (
        '9dd9d1d8228fa2b01502dc115b8826dba540612865fa32307b0b62dac1f41da3',
        '6c2d6e7c01c4bbf65361adc7a8efc18e4bb339dfd838d2748620a42f032cd6d5')),
}
FIRMWARE = {
    'IDAP-Link_R2': 'ff263961c65d4bf43fa3cb94523bcabc172f5ad4624085170497e43fc93cdbd4',
    'IDAP-Link_R3': 'd1cb619bab1c9e116d2467c09d9477d3548ff3c8e2228d895324aeef75f09018',
    'IDAP-M': '8a0fea11175672b68f683105b5d1937485d3ec085c51f1d7d89ba43cc6dbf806',
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def download(path, expected):
    with urllib.request.urlopen(BASE_URL + path, timeout=60) as response:
        data = response.read()
    if digest(data) != expected:
        raise ValueError('IDAP checksum mismatch: ' + path)
    return data


def replace_file(path, data, executable=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Stage on the destination filesystem. A failed write cannot truncate the old file.
    fd, name = tempfile.mkstemp(prefix='.idap-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
        if executable:
            os.chmod(name, 0o755)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def install(root, platform, force=False):
    root = Path(root)
    receipt_path = root / 'Tools/IDAP/installed.json'
    try:
        receipt = json.loads(receipt_path.read_text())
    except (OSError, ValueError):
        receipt = {}
    if not isinstance(receipt, dict):
        receipt = {}
    folder, suffix, checksums = TOOLS[platform]
    pending = []
    records = {}
    for tool, checksum in zip(('IDAPnRFProg', 'IDAPSetTarget'), checksums):
        filename = tool + ('.exe' if platform == 'windows' else '')
        destination = root / 'Tools/bin' / filename
        record = receipt.get(filename, {})
        if (not force and isinstance(record, dict) and record.get('version') == VERSION
                and record.get('platform') == platform and destination.is_file()
                and (platform == 'windows' or os.access(destination, os.X_OK))
                and digest(destination.read_bytes()) == record.get('sha256')):
            records[filename] = record
            print(tool + ' ' + VERSION + ' already installed')
            continue
        archive = download(folder + '/' + tool + suffix + '_' + VERSION + '.zip', checksum)
        with zipfile.ZipFile(io.BytesIO(archive)) as package:
            # Read only the named executable; never extract arbitrary archive paths.
            data = package.read(filename)
        if not data:
            raise ValueError('Empty IDAP executable: ' + filename)
        pending.append((destination, data, True))
        records[filename] = {'version': VERSION, 'platform': platform, 'sha256': digest(data)}
    for board, checksum in FIRMWARE.items():
        relative = board + '/build_' + VERSION + '/firmware.bin'
        destination = root / 'Tools/IDAP/Firmware' / relative
        if not force and destination.is_file() and digest(destination.read_bytes()) == checksum:
            continue
        data = download('Firmware/' + relative, checksum)
        pending.append((destination, data, False))
    # All downloads and archives must validate before replacing any installed file.
    for destination, data, executable in pending:
        replace_file(destination, data, executable)
        print('Installed ' + str(destination))
    replace_file(receipt_path, (json.dumps(records, indent=2) + '\n').encode())
    print('IDAP firmware: ' + str(root / 'Tools/IDAP/Firmware'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--platform', choices=TOOLS, required=True)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    install(args.root, args.platform, args.force)


if __name__ == '__main__':
    main()
