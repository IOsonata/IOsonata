#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Host checks for MqttCertDemo provisioning, setup, and MQTT startup ordering.

Uses the real shared example sources with hardware and TLS-state substitutes.
Cryptographic interoperability/error tests are in run_tls_cert.py. No ARM
compile/link, board, radio or external network access is performed here.
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import run_generic_demo as generic


ROOT = generic.ROOT
UECC = Path(os.environ.get('UECC_ROOT', ROOT / 'micro-ecc'))
PROJECT = ROOT / 'ARM/ST/STM32H7xx/STM32H753/exemples/MqttCertDemo'


def run(command, **kwargs):
	subprocess.run(command, check=True, **kwargs)


def source_links():
	generic.check_project(PROJECT, 'mqtt')
	for family in generic.TARGETS.values():
		ioc = ROOT / family / 'lib/ioc'
		links = ET.parse(ioc / '.project').findall('./linkedResources/link')
		for name in ('include/net/tls.h', 'src/net/tls.cpp', 'include/crypto/x509.h', 'src/crypto/x509.cpp'):
			found = [link for link in links if link.findtext('name') == name]
			assert len(found) == 1
			assert found[0].findtext('locationURI') == 'PARENT-6-PROJECT_LOC/' + name
			assert (ioc.parents[5] / name).resolve() == ROOT / name
	links = ET.parse(PROJECT / 'ioc/.project').findall('./linkedResources/link')
	for name in ('net_demo_tls_trust.cpp', 'net_demo_tls_trust.h'):
		found = [link for link in links if link.findtext('name') == 'src/' + name]
		assert len(found) == 1
		assert found[0].findtext('locationURI') == 'PARENT-7-PROJECT_LOC/exemples/net/' + name
	print('Native TLS demo and all three library source links PASS', flush=True)


def provision(tmp):
	output = tmp / 'credentials'
	header = tmp / 'net_demo_tls_trust_generated.h'
	run(['python3', str(ROOT / 'Python/mqtt_cert_demo.py'), 'prepare',
		'--output', str(output), '--header', str(header)])
	assert 'PRIVATE KEY' not in header.read_text()
	assert 'g_NetDemoTlsTrustAnchorCount = 1' in header.read_text()
	assert (output / 'ca.key').stat().st_mode & 0o077 == 0
	assert (output / 'server.key').stat().st_mode & 0o077 == 0
	assert 'tls_version tlsv1.3' in (output / 'mosquitto.conf').read_text()
	assert 'ciphers_tls1.3 TLS_AES_128_GCM_SHA256' in (output / 'mosquitto.conf').read_text()
	run(['openssl', 'verify', '-CAfile', str(output / 'ca.pem'), '-purpose', 'sslserver',
		'-verify_hostname', 'iosonata.test', str(output / 'server.pem')])
	bad = subprocess.run(['openssl', 'verify', '-CAfile', str(output / 'ca.pem'),
		'-verify_hostname', 'wrong.test', str(output / 'server.pem')],
		stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	assert bad.returncode != 0
	for args in (['--hostname', '127.0.0.1'], ['--hostname', 'bad/name.test'],
		['--hostname', 'trailing.test.'], ['--port', '0'], ['--days', '0']):
		failed = subprocess.run(['python3', str(ROOT / 'Python/mqtt_cert_demo.py'), 'prepare',
			'--output', str(tmp / 'bad'), '--header', str(tmp / 'bad.h'), *args],
			stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		assert failed.returncode != 0 and not (tmp / 'bad').exists()
	before = header.read_bytes()
	failed = subprocess.run(['python3', str(ROOT / 'Python/mqtt_cert_demo.py'), 'prepare',
		'--output', str(tmp / 'second'), '--header', str(header)],
		stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	assert failed.returncode != 0 and header.read_bytes() == before
	assert not (tmp / 'second').exists()
	print('P-256 provisioning, hostname verification, private file permissions and input checks PASS', flush=True)


def console_helper():
	spec = importlib.util.spec_from_file_location('mqtt_cert_demo', ROOT / 'Python/mqtt_cert_demo.py')
	helper = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(helper)
	clock = [0.0]
	instances = []
	previous = sys.modules.get('serial')

	class Serial:
		def __init__(self, **kwargs):
			assert kwargs == dict(port=None, baudrate=1000000, timeout=0.5, write_timeout=2)
			self.writes = []
			self.closed = False
			instances.append(self)

		def open(self):
			assert not self.dtr and not self.rts and self.port == 'test-port'

		def write(self, value):
			self.writes.append(value)

		def flush(self):
			pass

		def read(self, count):
			assert count == 256
			try:
				clock[0], chunk = next(steps)
			except StopIteration as error:
				raise KeyboardInterrupt from error
			return chunk

		def close(self):
			self.closed = True

	helper.time = SimpleNamespace(time=lambda: 1700000000 + clock[0], monotonic=lambda: clock[0])
	sys.modules['serial'] = SimpleNamespace(Serial=Serial)
	try:
		for initial in (
			b'Native TLS waiting for UTC: time <Unix seconds>\r\nTLS UTC set from console\r\n',
			b'UTC command rejected; zero time or active connection\r\n'):
			clock[0] = 0.0
			steps = iter([(1.0, initial), (10.0, b''), (20.0, b'ordinary MQTT log\r\n'),
				(21.0, b'Native TLS waiting for UTC: time <Unix seconds>\r\n'),
				(22.0, b''), (23.1, b''), (24.0, b'TLS UTC set from console\r\n')])
			try:
				helper.console(SimpleNamespace(port='test-port', baud=1000000))
			except KeyboardInterrupt:
				pass
			assert instances[-1].closed
			assert instances[-1].writes == [b'time 1700000000\n', b'time 1700000021\n', b'time 1700000023\n']
		assert len(instances) == 2
	finally:
		if previous is None:
			del sys.modules['serial']
		else:
			sys.modules['serial'] = previous
	print('Console helper UTC, active-session refusal, boot retry and serial close checks PASS', flush=True)


def main():
	source_links()
	console_helper()
	with tempfile.TemporaryDirectory(prefix='iosonata-native-demo-') as directory:
		tmp = Path(directory)
		(tmp / 'nrf.h').write_text('/* Host: no Nordic registers are used. */\n', encoding='ascii')
		flags = generic.board_flags(tmp, PROJECT / 'src', 'stm32h753')
		for source in ('exemples/net/net_demo.cpp', 'exemples/net/mqtt_generic.cpp',
			'exemples/net/net_demo_tls_trust.cpp'):
			run(['g++', *generic.BASE_FLAGS, *flags, '-fsyntax-only', str(ROOT / source)])
		provision(tmp)
		run(['g++', *generic.BASE_FLAGS, *flags, '-fsyntax-only',
			str(ROOT / 'exemples/net/net_demo_tls_trust.cpp')])
		print('MqttCertDemo actual board and setup host syntax, with and without trust header PASS', flush=True)
		for target in ('nrf9160', 'nrf91x1'):
			board = ROOT / generic.TARGETS[target] / 'exemples/MqttDemo/src'
			flags = generic.board_flags(tmp, board, target)
			run(['g++', *generic.BASE_FLAGS, *flags, '-DNET_DEMO_NATIVE_TLS=1',
				'-DNET_DEMO_TLS_SERVER_NAME="server.example"', '-fsyntax-only',
				str(ROOT / 'exemples/net/net_demo.cpp')])
		flags = generic.board_flags(tmp, PROJECT / 'src', 'stm32h753')
		bad = subprocess.run(['g++', *generic.BASE_FLAGS, *flags, '-DNET_DEMO_MODEM_TLS=1',
			'-fsyntax-only', str(ROOT / 'exemples/net/net_demo.cpp')],
			stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		assert bad.returncode != 0 and b'Select either native TLS or modem TLS' in bad.stderr
		print('Native TLS with actual Nordic board settings and conflicting TLS selection rejection PASS', flush=True)
		for opt in ('-O0', '-O2'):
			objects = []
			for source in ('src/crypto/crypto_softaes.cpp', 'src/crypto/crypto_aes_soft_block.cpp', 'src/crypto/crypto_softsha256.cpp',
				'src/crypto/crypto_p256.cpp', 'src/crypto/crypto_uecc.cpp', 'src/device.cpp',
				'src/device_intrf.cpp'):
				obj = tmp / (Path(source).stem + opt + '.o')
				run(['g++', *generic.BASE_FLAGS, *generic.TEST_FLAGS, opt,
					'-DuECC_ENABLE_VLI_API=1', '-I' + str(ROOT / 'include'), '-I' + str(UECC),
					'-c', str(ROOT / source), '-o', str(obj)])
				objects.append(str(obj))
			ecc = tmp / ('uecc' + opt + '.o')
			run(['gcc', opt, '-g', '-fsanitize=address,undefined', '-fno-pie',
				'-DuECC_ENABLE_VLI_API=1', '-I' + str(UECC), '-c', str(UECC / 'uECC.c'), '-o', str(ecc)])
			objects.append(str(ecc))
			for medium in (1, 2, 3):
				for missing in (False, True):
					binary = tmp / ('setup-' + str(medium) + ('-missing' if missing else '') + opt)
					flags = ['-DNET_DEMO_TEST_MEDIUM=' + str(medium),
						'-I' + str(ROOT / 'tests/net/generic_stubs'), '-I' + str(ROOT / 'include')]
					if missing:
						flags.append('-DNET_DEMO_TEST_NO_CA=1')
					run(['g++', *generic.BASE_FLAGS, *generic.TEST_FLAGS, opt, *flags,
						str(ROOT / 'tests/net/net_demo_native_tls_test.cpp'), *objects,
						'-Wl,--gc-sections', '-o', str(binary)])
					run([str(binary)], env=generic.ENV, timeout=30)
			flags = generic.board_flags(tmp, PROJECT / 'src', 'stm32h753')
			generic.build_and_run(tmp, 'mqtt-opening' + opt, flags,
				['tests/net/mqtt_generic_test.cpp', 'src/net/mqtt.cpp',
				'src/device_intrf.cpp', 'src/device.cpp'], opt)
			print(opt + ': native setup across LTE/Ethernet/CUSTOM and MQTT opening tests ASan/UBSan PASS', flush=True)
	print('Native TLS example host checks PASS; ARM and hardware not exercised.', flush=True)


if __name__ == '__main__':
	main()
