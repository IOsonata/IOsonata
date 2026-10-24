#!/usr/bin/env python3
"""Execute the nRF52/5340 ECB driver, including SoftDevice and DMA failures."""
from pathlib import Path
import subprocess
import tempfile
import os
ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='nrf-ecb-') as temp:
    out = Path(temp)
    for name in ('nrf.h', 'nrf_soc.h', 'nrf_sdm.h'):
        (out/name).write_text('#include "nrf_ecb_model.h"\n')
    for opt in ('-O0', '-O2'):
        for defines in (['NRF52_SERIES'], ['NRF52_SERIES', 'SOFTDEVICE_PRESENT=1'], ['NRF5340_XXAA_NETWORK']):
            flags = [opt, '-std=gnu++23', '-fno-rtti', '-fno-exceptions', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=address,undefined', '-fno-pie', '-no-pie', '-DCRYPTO_ECB_NRF_TIMEOUT=8']
            inc = ['-I'+str(p) for p in (out, ROOT/'tests/crypto', ROOT/'include', ROOT/'ARM/Nordic/include')]
            sources = ['src/crypto/crypto_p256.cpp','src/device.cpp','src/device_intrf.cpp','src/crypto/crypto_aes_modes.cpp',
                'src/crypto/crypto_aes_soft.cpp','ARM/Nordic/src/crypto_ecb_nrf.cpp','tests/crypto/nrf_ecb_model.cpp']
            subprocess.run(['g++', *flags, *inc, *['-D'+d for d in defines],
                *[str(ROOT/s) for s in sources], '-lcrypto', '-o', str(out/'test')], check=True)
            subprocess.run([str(out/'test')], check=True, env={**os.environ, 'ASAN_OPTIONS':'detect_leaks=0'})

# The Eclipse linked source and header must both resolve in common Nordic code.
import xml.etree.ElementTree as ET
projects = [ROOT / 'ARM/Nordic/nRF52' / part / 'lib/ioc/.project'
            for part in ('nRF52805', 'nRF52810', 'nRF52832', 'nRF52840')]
projects.append(ROOT / 'ARM/Nordic/nRF53/nRF5340_Net/lib/ioc/.project')
for project in projects:
    links = ET.parse(project).findall('.//linkedResources/link')
    ecb = [link for link in links if 'crypto_ecb_' in link.findtext('name', '')]
    assert len(ecb) == 2, project
    for link in ecb:
        uri = link.findtext('locationURI')
        root, relative = uri.split('/', 1)
        assert root == 'PARENT-4-PROJECT_LOC', (project, uri)
        resolved = project.parent.parents[3] / relative
        assert resolved.is_file(), (project, resolved)
        assert resolved.parent == ROOT / 'ARM/Nordic' / ('include' if relative.endswith('.h') else 'src')
    print(project.parent.parent.parent.name, 'common Nordic ECB project links resolve')
