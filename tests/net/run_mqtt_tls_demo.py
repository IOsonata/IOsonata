#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Host checks for the nRF91 generic MQTT modem-TLS example profiles.

Build the real example setup with simulated modem/credential operations and
the real MQTT application/client with an injected stream. These checks verify
configuration, sequencing, failure handling, the optional LTE radio-off test
and fresh MQTT sessions. They do
not perform TLS cryptography, validate a remote certificate, use modem firmware
or establish ARM compile/link or hardware interoperability.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

from run_generic_demo import (
	BASE_FLAGS, ENV, ROOT, TARGETS, TEST_FLAGS, board_flags, build_and_run,
	check_project, profile_board, run,
)


def unsupported_media():
	# Both modem-specific options must reject selection on another medium.
	for option, define in (('NET_DEMO_MODEM_TLS', 'NET_DEMO_TEST_TLS'),
		('NET_DEMO_LTE_RECOVERY_TEST', 'NET_DEMO_LTE_RECOVERY_TEST')):
		for medium, label in ((2, 'Ethernet'), (3, 'CUSTOM')):
			result = subprocess.run(['g++', *BASE_FLAGS,
				'-I' + str(ROOT / 'tests/net/generic_stubs'),
				'-I' + str(ROOT / 'include'), '-D' + define + '=1',
				'-DNET_DEMO_TEST_MEDIUM=' + str(medium), '-fsyntax-only',
				str(ROOT / 'exemples/net/net_demo.cpp')], capture_output=True, text=True)
			if result.returncode == 0 or option + ' requires the LTE medium' not in result.stderr:
				raise RuntimeError(label + ': missing ' + option + ' compile guard\n' + result.stderr)
	print('Non-LTE modem-TLS and recovery selections: compile rejection PASS', flush=True)


def application(tmp, target):
	project = ROOT / TARGETS[target] / 'exemples/MqttTlsDemo'
	check_project(project, 'mqtt', modem_tls=True)
	flags = board_flags(tmp, project / 'src', target)
	flags += ['-I' + str(ROOT / 'exemples/net')]
	for source in ('mqtt_generic.cpp', 'net_demo.cpp', 'net_demo_tls_ca.cpp'):
		run(['g++', *BASE_FLAGS, *flags, '-fsyntax-only',
			str(ROOT / 'exemples/net' / source)])
	if target == 'nrf91x1':
		directory = profile_board(tmp, project / 'src/board.h', target + '-recovery-config',
			{'NET_DEMO_LTE_RECOVERY_TEST': '1'})
		recovery_flags = board_flags(tmp, directory, target)
		recovery_flags += ['-I' + str(ROOT / 'exemples/net')]
		run(['g++', *BASE_FLAGS, *recovery_flags, '-fsyntax-only',
			str(ROOT / 'exemples/net/net_demo.cpp')])
	print(target + ': secure project links and actual board/setup host syntax PASS', flush=True)
	sources = ['tests/net/mqtt_generic_test.cpp', 'src/net/mqtt.cpp',
		'src/device_intrf.cpp', 'src/device.cpp']
	for opt in ('-O0', '-O2'):
		build_and_run(tmp, target + '-mqtt-tls' + opt, flags, sources, opt)
		print(target + ' MQTT application ' + opt + ': ASan/UBSan PASS', flush=True)


def setup(tmp):
	flags = ['-I' + str(ROOT / 'tests/net/generic_stubs'),
		'-I' + str(ROOT / 'include'), '-DNET_DEMO_TEST_MEDIUM=1',
		'-DNET_DEMO_TEST_TLS=1']
	variants = (
		('provision', (), ('missing', 'stored', 'init-failure', 'disconnect-failure',
			'write-failure', 'check-failure', 'reconnect-failure')),
		('existing-only', ('-DNET_DEMO_TEST_NO_CA=1',), ('missing', 'stored')),
		('replace', ('-DNET_DEMO_TEST_REPLACE_CA=1',), ('missing', 'stored', 'write-failure')),
	)
	for name, extra, scenarios in variants:
		for opt in ('-O0', '-O2'):
			binary = tmp / ('setup-tls-' + name + opt)
			run(['g++', *BASE_FLAGS, *flags, *extra, opt, *TEST_FLAGS,
				str(ROOT / 'tests/net/net_demo_setup_test.cpp'),
				str(ROOT / 'src/device_intrf.cpp'), str(ROOT / 'src/device.cpp'),
				'-Wl,--gc-sections', '-o', str(binary)])
			for scenario in scenarios:
				run([str(binary), scenario], env=ENV, timeout=30)
			print(name + ' TLS setup ' + opt + ': all scenarios ASan/UBSan PASS', flush=True)


def recovery_setup(tmp):
	flags = ['-I' + str(ROOT / 'tests/net/generic_stubs'),
		'-I' + str(ROOT / 'include'), '-DNET_DEMO_TEST_MEDIUM=1',
		'-DNET_DEMO_LTE_RECOVERY_TEST=1']
	variants = (
		('plain', (), ('normal',)),
		('tls', ('-DNET_DEMO_TEST_TLS=1',), ('normal', 'off-wrap',
			'disconnect-failure', 'connect-failure', 'no-off-event',
			'no-registration', 'fault-off', 'fault-retry', 'fault-registration')),
	)
	for name, extra, scenarios in variants:
		for opt in ('-O0', '-O2'):
			binary = tmp / ('setup-recovery-' + name + opt)
			run(['g++', *BASE_FLAGS, *flags, *extra, opt, *TEST_FLAGS,
				str(ROOT / 'tests/net/net_demo_setup_test.cpp'),
				str(ROOT / 'src/device_intrf.cpp'), str(ROOT / 'src/device.cpp'),
				'-Wl,--gc-sections', '-o', str(binary)])
			for scenario in scenarios:
				run([str(binary), scenario], env=ENV, timeout=30)
			print(name + ' LTE recovery setup ' + opt + ': all scenarios ASan/UBSan PASS', flush=True)


def main():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument('--target', choices=('nrf9160', 'nrf91x1', 'all'), default='all')
	args = parser.parse_args()
	targets = ('nrf9160', 'nrf91x1') if args.target == 'all' else (args.target,)
	unsupported_media()
	with tempfile.TemporaryDirectory(prefix='iosonata-mqtt-modem-tls-') as directory:
		tmp = Path(directory)
		(tmp / 'nrf.h').write_text('/* Host: no Nordic registers are used. */\n', encoding='ascii')
		for target in targets:
			application(tmp, target)
		setup(tmp)
		recovery_setup(tmp)
	run([sys.executable, str(ROOT / 'tests/net/run_nrf91_socket.py')])
	print('MQTT modem-TLS examples: host checks PASS; ARM, modem TLS and hardware not exercised.',
		flush=True)


if __name__ == '__main__':
	main()
