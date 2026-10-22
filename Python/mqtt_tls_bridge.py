#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Single-board TLS 1.3 PSK test listener forwarding to a local MQTT broker.

Requires OpenSSL 3. Uses a PUBLIC TEST KEY, not production credentials.
OpenSSL handles TLS; Python forwards only application bytes. Each TLS session
gets a fresh broker connection. No MQTT parsing or TLS implementation here.
"""
import argparse
import asyncio
import contextlib


async def copy_stream(reader, writer, idle):
	while True:
		data = await asyncio.wait_for(reader.read(4096), idle)
		if not data:
			return
		writer.write(data)
		await writer.drain()


async def session(args):
	command = [args.openssl, 's_server', '-quiet', '-ign_eof', '-naccept', '1',
		'-accept', f'{args.host}:{args.port}', '-nocert', '-tls1_3',
		'-groups', 'P-256', '-ciphersuites', 'TLS_AES_128_GCM_SHA256',
		'-psk_identity', 'IOsonata', '-psk', args.psk]
	process = await asyncio.create_subprocess_exec(*command,
		stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
	peer = None
	tasks = []
	try:
		# Connect to the broker only after authenticated application data arrives.
		data = await asyncio.wait_for(process.stdout.read(4096), args.idle)
		if not data:
			return await process.wait()
		reader, peer = await asyncio.wait_for(
			asyncio.open_connection('127.0.0.1', args.broker_port), 5)
		print('TLS application stream connected to MQTT broker', flush=True)
		peer.write(data)
		await peer.drain()
		tasks = [asyncio.create_task(copy_stream(process.stdout, peer, args.idle)),
			asyncio.create_task(copy_stream(reader, process.stdin, args.idle))]
		done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
		for task in done:
			task.result()
		return 0
	finally:
		for task in tasks:
			task.cancel()
		await asyncio.gather(*tasks, return_exceptions=True)
		if peer is not None:
			peer.close()
			with contextlib.suppress(OSError, asyncio.TimeoutError):
				await asyncio.wait_for(peer.wait_closed(), 2)
		if process.returncode is None:
			process.terminate()
			try:
				await asyncio.wait_for(process.wait(), 2)
			except asyncio.TimeoutError:
				process.kill()
				await process.wait()
		process.stdin.close()


async def main(args):
	print(f'TLS 1.3 PSK test endpoint {args.host}:{args.port}; '
		f'broker 127.0.0.1:{args.broker_port}', flush=True)
	print('PUBLIC TEST KEY; one board at a time; Ctrl+C stops', flush=True)
	while True:
		try:
			status = await session(args)
			if status:
				print(f'OpenSSL exited with status {status}; retrying', flush=True)
		except (OSError, asyncio.TimeoutError) as error:
			print(f'Session ended: {type(error).__name__}: {error}', flush=True)
		print('Waiting for next TLS session', flush=True)
		await asyncio.sleep(0.25)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument('--openssl', default='openssl', help='OpenSSL 3 executable')
	parser.add_argument('--host', default='0.0.0.0')
	parser.add_argument('--port', type=int, default=8883)
	parser.add_argument('--broker-port', type=int, default=1883)
	parser.add_argument('--idle', type=float, default=60,
		help='seconds without application data before recycling the session')
	parser.add_argument('--psk', default=bytes(range(32)).hex(), help='32-byte test key in hex')
	args = parser.parse_args()
	try:
		key = bytes.fromhex(args.psk)
	except ValueError:
		parser.error('--psk must be hexadecimal')
	if len(key) != 32 or len(args.psk) != 64:
		parser.error('--psk must contain exactly 64 hex digits')
	if not 1 <= args.port <= 65535 or not 1 <= args.broker_port <= 65535 or args.idle <= 0:
		parser.error('ports must be 1..65535 and idle must be positive')
	try:
		asyncio.run(main(args))
	except KeyboardInterrupt:
		pass
