#!/usr/bin/env python3
"""Check RA4M1 IOC example projects and execute shared-example API models.

The host models use explicitly reduced test contracts, not production MCU
headers, GNU/newlib, or an IOC managed build. Hardware is not exercised.
--metadata-only needs Python only. --allow-partial-checkout is for the standalone
port package when shared, unmodified repository sources are not mounted.
"""
from pathlib import Path
import argparse
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TARGET = ROOT / 'ARM/Renesas/RA4M1'
BUILD = HERE / 'build/examples'
EXAMPLES = {
    'Blinky': ('exemples/misc/blinky.c', 0),
    'TimerDemo': ('exemples/timer/timer_demo.cpp', 1),
    'UartLoopback': ('exemples/uart/uart_loopback.cpp', 2),
    'UartPrbsTxTest': ('exemples/uart/uart_prbs_tx.cpp', 3),
}
PREFIX = 'ilg.gnuarmeclipse.managedbuild.cross.option.'


def metadata(partial=False):
    missing = set()
    example_dirs = {p.name for p in (TARGET/'exemples').iterdir() if p.is_dir()}
    assert example_dirs == set(EXAMPLES), 'Unexpected or hardware-specific example project'
    timer_source = (ROOT/EXAMPLES['TimerDemo'][0]).read_text()
    assert '#include "coredev/timer.h"' in timer_source
    assert '#include "timer_ra4m1.h"' not in timer_source
    assert not re.search(r'\b(?:AGT|GPT|SCT)\w*', timer_source, re.IGNORECASE)
    for name, (shared, _) in EXAMPLES.items():
        project = TARGET / 'exemples' / name / 'ioc'
        px = ET.parse(project / '.project').getroot()
        assert px.findtext('name') == name
        assert px.findtext('projects/project') == 'IOsonata_RA4M1'
        assert px.find("natures/nature[.='org.eclipse.cdt.core.ccnature']") is not None
        expected = {ROOT / shared}
        if name == 'UartPrbsTxTest':
            expected.add(ROOT / 'src/prbs.c')
        sources = set()
        for link in px.findall('linkedResources/link'):
            uri = link.findtext('locationURI')
            if uri == 'virtual:/virtual':
                continue
            match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
            assert match, uri
            path = (project.parents[int(match[1]) - 1] / match[2]).resolve()
            assert path.is_relative_to(ROOT), path
            assert 'tests' not in path.parts, path
            if path.suffix in ('.c', '.cpp'):
                sources.add(path)
            if not path.exists():
                missing.add(str(path.relative_to(ROOT)))
        assert sources == expected, (name, sources)
        # Only configuration lives under an MCU example: no duplicate mains.
        assert list((project.parent / 'src').iterdir()) == [project.parent / 'src/board.h']
        xml = ET.parse(project / '.cproject').getroot()
        ids = {e.get('id') for e in xml.iter() if e.get('id')}
        for scan in xml.findall('.//scannerConfigBuildInfo'):
            assert all(x in ids for x in scan.get('instanceId').split(';'))
        configs = xml.findall('.//cconfiguration')
        assert len(configs) == 2
        for cfg in configs:
            c = cfg.find("storageModule[@moduleId='cdtBuildSystem']/configuration")
            config = c.get('name')
            assert config in ('Debug', 'Release')
            assert c.get('artifactExtension') == 'elf'
            assert c.get('artifactName') == '${ProjName}'
            assert c.get('buildArtefactType').endswith('.exe')
            assert c.get('parent').endswith('.elf.' + config.lower())
            tc = c.find('folderInfo/toolChain')
            assert tc.get('superClass').endswith('.elf.' + config.lower())
            opts = {x.get('superClass'): x for x in tc.findall('.//option')}
            assert opts[PREFIX+'arm.target.family'].get('value').endswith('.cortex-m4')
            assert opts[PREFIX+'arm.target.fpu.abi'].get('value').endswith('.hard')
            assert opts[PREFIX+'arm.target.fpu.unit'].get('value').endswith('.fpv4spd16')
            assert opts[PREFIX+'addtools.createflash'].get('value') == 'true'
            for lang in ('c', 'cpp'):
                values = lambda key: [x.get('value').strip('"') for x in opts[PREFIX+lang+key].findall('listOptionValue')]
                inc = {(project/config/p).resolve() for p in values('.compiler.include.paths')}
                assert inc == {project.parent/'src', TARGET/'include', ROOT/'ARM/include',
                               ROOT/'ARM/CMSIS/Core/Include', ROOT/'include'}
                assert values('.linker.libs') == ['IOsonata_RA4M1']
                assert (project/config/values('.linker.paths')[0]).resolve() == TARGET/'lib/ioc'/config
                script = (project/config/values('.linker.scriptfile')[0]).resolve()
                assert script == TARGET/'ldscript/gcc_ra4m1.ld' and script.is_file()
                flags = opts[PREFIX+lang+'.linker.other'].get('value')
                assert '--specs=nosys.specs' in flags
                assert '-u,ResetEntry,-u,__Vectors' in flags
                assert 'rdimon' not in flags and 'nostartfiles' not in flags
        print('PASS: IOC metadata, shared source links, configuration paths:', name)
    if missing:
        if not partial:
            raise AssertionError('Missing repository files: ' + ', '.join(sorted(missing)))
        print('PARTIAL CHECKOUT: existence not checked locally:', ', '.join(sorted(missing)))


def run(args):
    result = subprocess.run([str(x) for x in args], cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'{args}\n{result.stdout}')
    return result.stdout


def models():
    compiler = os.environ.get('CLANGXX') or shutil.which('clang++')
    if not compiler:
        raise SystemExit('Clang++ is required for the host API models (or set CLANGXX).')
    BUILD.mkdir(parents=True, exist_ok=True)
    inc = ['-I'+str(HERE/'example_include'), '-I'+str(HERE/'include'), '-I'+str(TARGET/'include')]
    common = [compiler, '-std=c++23', '-Wall', '-Wextra', '-Werror',
              '-Wno-missing-field-initializers', '-DRA4M1_HOST_TEST', *inc]
    for name in EXAMPLES:
        for package in (40, 48, 64, 100):
            output = BUILD / f'{name}_pins_{package}'
            run([*common, '-O2', '-DRA4M1_PACKAGE_PINS='+str(package),
                 '-I'+str(TARGET/'exemples'/name/'src'), HERE/'example_pin_probe.cpp',
                 '-o', output])
            print(name, run([output]).strip())
    count = 0
    for name, (source, kind) in EXAMPLES.items():
        if not kind:
            continue
        # Exercise the same timer project for every logical DevNo. Do not
        # create separate application projects or board headers per backend.
        devices = range(10) if kind == 1 else (None,)
        for device in devices:
            for api in ('cpp', 'c'):
                for opt in ('-O0', '-Os', '-O2', 'sanitized'):
                    output = BUILD / f'{name}_{device}_{api}_{opt}'
                    flags = [opt] if opt != 'sanitized' else ['-O1', '-g', '-fsanitize=address,undefined',
                               '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
                    defs = ['-DDEMO_C'] if api == 'c' else []
                    if device is not None:
                        defs.append('-DTIMER_DEVNO='+str(device))
                    run([*common, *flags, *defs, '-DEXAMPLE_KIND='+str(kind),
                         '-DEXAMPLE_SOURCE="'+str(ROOT/source)+'"',
                         '-I'+str(TARGET/'exemples'/name/'src'), HERE/'example_model.cpp', '-o', output])
                    log = run([output])
                    output.with_suffix('.log').write_text(log)
                    print(name, device, api, opt, log.splitlines()[-1])
                    count += 1
    print(f'PASS: {count} host model builds/runs with reduced example API contracts')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata-only', action='store_true')
    parser.add_argument('--allow-partial-checkout', action='store_true')
    args = parser.parse_args()
    metadata(args.allow_partial_checkout)
    if not args.metadata_only:
        models()
    print('NOT TESTED: IOC import/managed build, production GNU/newlib firmware linking, hardware.')


if __name__ == '__main__':
    main()
