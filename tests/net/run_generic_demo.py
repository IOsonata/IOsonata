#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Host checks for the new generic MQTT and CoAP examples.

Compile the actual new applications with each selected board header and run
their real shared protocol clients through two different DeviceIntrf ports.
Also exercise LTE, Ethernet and custom setup with hardware substitutes.
Requires g++; no SDK, radio, network connection or Python packages are needed.
These checks do not replace ARM compile/link or board testing.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
	UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
TARGETS = {
	'nrf9160': 'ARM/Nordic/nRF91/nRF9160',
	'nrf91x1': 'ARM/Nordic/nRF91/nRF91x1',
	'stm32h753': 'ARM/ST/STM32H7xx/STM32H753',
}
BASE_FLAGS = [
	'-std=gnu++17', '-Wall', '-Wextra', '-Werror',
	'-Wno-missing-field-initializers', '-fno-exceptions', '-fno-rtti',
]
TEST_FLAGS = [
	'-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
	'-fno-pie', '-no-pie', '-ffunction-sections', '-fdata-sections',
]


def run(command, **kwargs):
	subprocess.run(command, check=True, **kwargs)


def check_project(project, protocol):
	"""Fail before compilation if a project points back at the old application."""
	ioc = project / 'ioc'
	tree = ET.parse(ioc / '.project')
	expected = {
		'src/' + protocol + '_generic.cpp': ROOT / 'exemples/net' / (protocol + '_generic.cpp'),
		'src/net_demo.cpp': ROOT / 'exemples/net/net_demo.cpp',
		'src/net_demo.h': ROOT / 'exemples/net/net_demo.h',
	}
	links = {}
	for link in tree.findall('./linkedResources/link'):
		name = link.findtext('name')
		uri = link.findtext('locationURI', '')
		if uri.startswith('PARENT-'):
			prefix, rest = uri.split('/', 1)
			count = int(prefix.removeprefix('PARENT-').removesuffix('-PROJECT_LOC'))
			links[name] = ioc.parents[count - 1].joinpath(rest).resolve()
	for name, path in expected.items():
		if links.get(name) != path or not path.is_file():
			raise RuntimeError(str(ioc) + ': wrong or missing link ' + name)
	ET.parse(ioc / '.cproject')


def build_and_run(tmp, name, flags, sources, opt):
	binary = tmp / name
	run(['g++', *BASE_FLAGS, *flags, opt, *TEST_FLAGS,
		*[str(ROOT / source) for source in sources],
		'-Wl,--gc-sections', '-o', str(binary)])
	run([str(binary)], env=ENV, timeout=30)


def board_flags(tmp, board_dir, target):
	flags = ['-I' + str(board_dir), '-I' + str(tmp), '-I' + str(ROOT / 'include')]
	if target == 'stm32h753':
		flags += ['-DSTM32H753xx=1', '-I' + str(ROOT / 'ARM/ST/include')]
	else:
		flags += ['-DNRF91_SERIES=1', '-I' + str(ROOT / 'ARM/Nordic/include')]
	return flags


def profile_board(tmp, board, name, overrides):
	directory = tmp / name
	directory.mkdir()
	lines = ['#include "' + board.as_posix() + '"']
	for macro, value in overrides.items():
		lines += ['#undef ' + macro, '#define ' + macro + ' ' + value]
	(directory / 'board.h').write_text('\n'.join(lines) + '\n', encoding='ascii')
	return directory


def test_application(tmp, target, protocol):
	project = ROOT / TARGETS[target] / 'exemples' / ('MqttDemo' if protocol == 'mqtt' else 'CoapDemo')
	board = project / 'src/board.h'
	check_project(project, protocol)
	flags = board_flags(tmp, board.parent, target)
	for source in ('exemples/net/' + protocol + '_generic.cpp', 'exemples/net/net_demo.cpp'):
		run(['g++', *BASE_FLAGS, *flags, '-fsyntax-only', str(ROOT / source)])
	print(target + ' ' + protocol + ': project links and actual board/setup host syntax PASS', flush=True)
	profiles = [('default', {})]
	if protocol == 'coap':
		profiles += [('helper', {'COAP_DEMO_PUBLIC_TEST': '0'})]
	sources = [
		'tests/net/' + protocol + '_generic_test.cpp', 'src/net/' + protocol + '.cpp',
		'src/device_intrf.cpp', 'src/device.cpp',
	]
	for profile, overrides in profiles:
		name = target + '-' + protocol + '-' + profile
		directory = profile_board(tmp, board, name, overrides)
		flags = board_flags(tmp, directory, target)
		for opt in ('-O0', '-O2'):
			build_and_run(tmp, name + opt, flags, sources, opt)
			print(name + ' ' + opt + ': ASan/UBSan PASS', flush=True)
	# The empty-host startup guard is independent of the target pin map.
	# Run it once per protocol, against the first requested target.
	return board, sources


def main():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument('--protocol', choices=('mqtt', 'coap', 'all'), default='all')
	parser.add_argument('--target', choices=(*TARGETS, 'all'), default='all')
	args = parser.parse_args()
	targets = tuple(TARGETS) if args.target == 'all' else (args.target,)
	protocols = ('mqtt', 'coap') if args.protocol == 'all' else (args.protocol,)
	with tempfile.TemporaryDirectory(prefix='iosonata-generic-net-') as directory:
		tmp = Path(directory)
		(tmp / 'nrf.h').write_text('/* Host: no Nordic registers are used. */\n', encoding='ascii')
		for target in targets:
			for protocol in protocols:
				board, sources = test_application(tmp, target, protocol)
				if target == targets[0]:
					name = protocol + '-empty-host'
					overrides = {protocol.upper() + '_DEMO_HOST': '""'}
					profile = profile_board(tmp, board, name, overrides)
					flags = board_flags(tmp, profile, target)
					for opt in ('-O0', '-O2'):
						build_and_run(tmp, name + opt, flags, sources, opt)
			# Source/header checks select real project settings; hardware substitute
			# setup tests below select all media independently of the MCU label.
		for medium, label in ((1, 'LTE'), (2, 'Ethernet'), (3, 'CUSTOM')):
			flags = ['-I' + str(ROOT / 'tests/net/generic_stubs'),
				'-I' + str(ROOT / 'include'), '-DNET_DEMO_TEST_MEDIUM=' + str(medium)]
			for opt in ('-O0', '-O2'):
				build_and_run(tmp, 'setup-' + label + opt, flags,
					['tests/net/net_demo_setup_test.cpp', 'src/device_intrf.cpp', 'src/device.cpp'], opt)
				print(label + ' setup ' + opt + ': ASan/UBSan PASS', flush=True)
	print('Generic network examples: selected host checks PASS; ARM/hardware not exercised.', flush=True)


if __name__ == '__main__':
	main()
