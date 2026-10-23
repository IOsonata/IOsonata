#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Prepare a local P-256 MQTT TLS server and provision the demo's UTC clock.

prepare generates private credentials in --output and a public DER trust header
beside the firmware board.h. It does not contact a public CA or start a server.
console sends the commissioning PC's current UTC time and displays board logs.
Requires OpenSSL for prepare; console additionally requires pyserial.
"""
import argparse
import ipaddress
import os
from pathlib import Path
import re
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HEADER = ROOT / 'ARM/ST/STM32H7xx/STM32H753/exemples/MqttCertDemo/src/net_demo_tls_trust_generated.h'


def openssl(*arguments):
	result = subprocess.run(['openssl', *map(str, arguments)], check=True,
		stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	return result.stdout


def dns_name(value):
	try:
		ipaddress.ip_address(value)
	except ValueError:
		pass
	else:
		raise argparse.ArgumentTypeError('use a DNS name in the server certificate, not an IP literal')
	if len(value) > 253 or not all(re.fullmatch(r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?', label)
		for label in value.split('.')):
		raise argparse.ArgumentTypeError('expected an ASCII DNS name without a trailing dot')
	return value


def listen_address(value):
	try:
		return str(ipaddress.ip_address(value))
	except ValueError as error:
		raise argparse.ArgumentTypeError('expected a listener IP address') from error


def public_header(der):
	lines = ['/* Generated public CA certificate. No private key. */',
		'#ifndef __NET_DEMO_TLS_TRUST_GENERATED_H__',
		'#define __NET_DEMO_TLS_TRUST_GENERATED_H__', '',
		'static const uint8_t s_NetDemoCaDer[] = {']
	for start in range(0, len(der), 12):
		lines.append('\t' + ', '.join('0x%02x' % value for value in der[start:start + 12]) + ',')
	lines += ['};', '', 'const X509Cert_t g_NetDemoTlsTrustAnchors[] = {',
		'\t{s_NetDemoCaDer, sizeof(s_NetDemoCaDer)},', '};',
		'const uint32_t g_NetDemoTlsTrustAnchorCount = 1;', '', '#endif', '']
	return '\n'.join(lines)


def prepare(args):
	output = args.output.expanduser().resolve()
	header = args.header.expanduser().resolve()
	if output.exists() and any(output.iterdir()):
		raise ValueError('output directory is not empty; select a new directory')
	if header.exists() and not args.replace_header:
		raise ValueError('public header already exists; use --replace-header to replace it')
	if not header.parent.is_dir():
		raise ValueError('header parent directory does not exist: ' + str(header.parent))
	# Mosquitto's configuration uses unquoted paths. Reject newline characters
	# and spaces instead of generating a configuration that points elsewhere.
	if any(character.isspace() for character in str(output)):
		raise ValueError('output path must not contain whitespace')
	mask = os.umask(0o077)
	try:
		output.mkdir(parents=True, exist_ok=True, mode=0o700)
		root_config = output / 'ca.cnf'
		leaf_config = output / 'server.cnf'
		root_config.write_text('[req]\ndistinguished_name = dn\n[dn]\n[v3]\n'
			'basicConstraints = critical,CA:TRUE,pathlen:0\n'
			'keyUsage = critical,keyCertSign,cRLSign\n'
			'subjectKeyIdentifier = hash\n'
			'authorityKeyIdentifier = keyid:always\n', encoding='ascii')
		leaf_config.write_text('[v3]\nbasicConstraints = critical,CA:FALSE\n'
			'keyUsage = critical,digitalSignature\n'
			'extendedKeyUsage = serverAuth\n'
			'subjectAltName = DNS:' + args.hostname + '\n'
			'subjectKeyIdentifier = hash\n'
			'authorityKeyIdentifier = keyid:always\n', encoding='ascii')
		for name in ('ca', 'server'):
			openssl('ecparam', '-name', 'prime256v1', '-genkey', '-noout',
				'-out', output / (name + '.key'))
		openssl('req', '-new', '-x509', '-sha256', '-key', output / 'ca.key',
			'-out', output / 'ca.pem', '-subj', '/CN=IOsonata local MQTT test CA',
			'-days', '3650', '-set_serial', '1', '-config', root_config, '-extensions', 'v3')
		openssl('req', '-new', '-key', output / 'server.key', '-out', output / 'server.csr',
			'-subj', '/CN=' + args.hostname)
		openssl('x509', '-req', '-in', output / 'server.csr', '-CA', output / 'ca.pem',
			'-CAkey', output / 'ca.key', '-out', output / 'server.pem', '-sha256',
			'-days', str(args.days), '-set_serial', '2', '-extfile', leaf_config, '-extensions', 'v3')
		der = openssl('x509', '-in', output / 'ca.pem', '-outform', 'DER')
		(output / 'ca.der').write_bytes(der)
		(output / 'server-chain.pem').write_bytes((output / 'server.pem').read_bytes() +
			(output / 'ca.pem').read_bytes())
		openssl('verify', '-CAfile', output / 'ca.pem', '-purpose', 'sslserver',
			'-verify_hostname', args.hostname, output / 'server.pem')
		config = ('listener ' + str(args.port) + ' ' + args.listen + '\n'
			'allow_anonymous true\npersistence false\n'
			'cafile ' + str(output / 'ca.pem') + '\n'
			'certfile ' + str(output / 'server-chain.pem') + '\n'
			'keyfile ' + str(output / 'server.key') + '\n'
			'require_certificate false\ntls_version tlsv1.3\n'
			'ciphers_tls1.3 TLS_AES_128_GCM_SHA256\n')
		(output / 'mosquitto.conf').write_text(config, encoding='ascii')
		header.write_text(public_header(der), encoding='ascii')
	finally:
		os.umask(mask)
	print('Created local P-256 CA and server credentials in ' + str(output))
	print('Public firmware trust header: ' + str(header))
	print('Set NET_DEMO_TLS_SERVER_NAME to "' + args.hostname + '" and MQTT_DEMO_HOST to the PC LAN address.')
	print('Build the library, then clean/rebuild MqttCertDemo after generating the header.')
	print('Start as the same ordinary user: mosquitto -c ' + str(output / 'mosquitto.conf') + ' -v')


def console(args):
	try:
		import serial
	except ImportError as error:
		raise ValueError('console requires pyserial: python3 -m pip install pyserial') from error
	connection = serial.Serial(port=None, baudrate=args.baud, timeout=0.5, write_timeout=2)
	connection.dtr = False
	connection.rts = False
	connection.port = args.port
	connection.open()
	try:
		acknowledged = False
		last_sent = 0.0

		def set_time():
			nonlocal last_sent
			seconds = int(time.time())
			if not 0 < seconds <= 253402300799:
				raise ValueError('PC UTC time is outside the supported range')
			connection.write(('time ' + str(seconds) + '\n').encode('ascii'))
			connection.flush()
			last_sent = time.monotonic()
			print('Sent PC UTC ' + str(seconds) + '; waiting for board acknowledgment.', flush=True)

		set_time()
		print('Monitoring board console; Ctrl-C closes it. Keep the PC clock correct.', flush=True)
		pending = bytearray()
		while True:
			chunk = connection.read(256)
			if not chunk:
				if not acknowledged and time.monotonic() - last_sent >= 2.0:
					set_time()
				continue
			sys.stdout.buffer.write(chunk)
			sys.stdout.buffer.flush()
			pending.extend(chunk)
			while b'\n' in pending:
				line, _, pending = pending.partition(b'\n')
				if b'Native TLS waiting for UTC:' in line:
					acknowledged = False
				elif b'TLS UTC set from console' in line:
					acknowledged = True
				elif b'UTC command rejected; zero time or active connection' in line:
					# This helper never sends zero. Preserve an active session's
					# clock and resume provisioning only after a new boot prompt.
					acknowledged = True
			if len(pending) > 1024:
				del pending[:-1024]
			if not acknowledged and time.monotonic() - last_sent >= 2.0:
				set_time()
	finally:
		connection.close()


def main():
	parser = argparse.ArgumentParser(description=__doc__)
	commands = parser.add_subparsers(dest='command', required=True)
	setup = commands.add_parser('prepare', help='generate credentials and the public firmware trust header')
	setup.add_argument('--output', type=Path, required=True, help='new local directory for private credentials')
	setup.add_argument('--header', type=Path, default=DEFAULT_HEADER)
	setup.add_argument('--replace-header', action='store_true')
	setup.add_argument('--hostname', type=dns_name, default='iosonata.test')
	setup.add_argument('--listen', type=listen_address, default='0.0.0.0')
	setup.add_argument('--port', type=int, default=8883)
	setup.add_argument('--days', type=int, default=30, help='server certificate validity in days (1..3650)')
	terminal = commands.add_parser('console', help='send current UTC and display board logs')
	terminal.add_argument('--port', required=True, help='serial device name')
	terminal.add_argument('--baud', type=int, default=1000000)
	args = parser.parse_args()
	try:
		if args.command == 'prepare':
			if not 1 <= args.port <= 65535 or not 1 <= args.days <= 3650:
				raise ValueError('port must be 1..65535 and days must be 1..3650')
			prepare(args)
		else:
			console(args)
	except KeyboardInterrupt:
		return 0
	except (OSError, ValueError, subprocess.CalledProcessError) as error:
		print(str(error), file=sys.stderr)
		if isinstance(error, subprocess.CalledProcessError) and error.stderr:
			print(error.stderr.decode('utf-8', 'replace'), file=sys.stderr)
		return 1
	return 0


if __name__ == '__main__':
	raise SystemExit(main())
