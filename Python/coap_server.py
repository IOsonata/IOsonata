#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Independent aiocoap UDP peer for the IOsonata CoAP demo and host tests.

Install with: python3 -m pip install aiocoap==0.4.17
Run on the board's LAN: python3 Python/coap_server.py --host 10.5.5.51
Plain CoAP only, without DTLS or authentication. One shared /value resource.
"""

import argparse
import asyncio
import ipaddress
import logging

try:
    import aiocoap
    import aiocoap.resource as resource
except ImportError:
    raise SystemExit('Install the test dependency: python3 -m pip install aiocoap==0.4.17')


class Hello(resource.Resource):
    async def render_get(self, request):
        return aiocoap.Message(code=aiocoap.CONTENT, payload=b'IOsonata CoAP\n',
                               content_format=0)


class Echo(resource.Resource):
    async def render_post(self, request):
        if len(request.payload) > 1024:
            return aiocoap.Message(code=aiocoap.REQUEST_ENTITY_TOO_LARGE)
        return aiocoap.Message(code=aiocoap.CONTENT, payload=request.payload,
                               content_format=42)


class Value(resource.Resource):
    def __init__(self):
        super().__init__()
        self.value = None

    async def render_get(self, request):
        if self.value is None:
            return aiocoap.Message(code=aiocoap.NOT_FOUND)
        return aiocoap.Message(code=aiocoap.CONTENT, payload=self.value,
                               content_format=42)

    async def render_put(self, request):
        if len(request.payload) > 1024:
            return aiocoap.Message(code=aiocoap.REQUEST_ENTITY_TOO_LARGE)
        self.value = bytes(request.payload)
        return aiocoap.Message(code=aiocoap.CHANGED)

    async def render_delete(self, request):
        if self.value is None:
            return aiocoap.Message(code=aiocoap.NOT_FOUND)
        self.value = None
        return aiocoap.Message(code=aiocoap.DELETED)


class Slow(resource.Resource):
    def __init__(self, delay):
        super().__init__()
        self.delay = delay

    async def render_get(self, request):
        # Delay beyond aiocoap's piggyback window so CON receives empty ACK
        # followed by a separate response from the independent implementation.
        await asyncio.sleep(self.delay)
        return aiocoap.Message(code=aiocoap.CONTENT, payload=b'separate response\n',
                               content_format=0)


async def serve(args):
    site = resource.Site()
    site.add_resource(['hello'], Hello())
    site.add_resource(['echo'], Echo())
    site.add_resource(['value'], Value())
    site.add_resource(['slow'], Slow(args.delay))
    # This transport supports Linux/macOS without optional WebSocket/DTLS
    # packages. It binds one explicit address, not a wildcard or multicast.
    context = await aiocoap.Context.create_server_context(
        site, bind=(args.host, args.port), transports=['simplesocketserver'])
    print(f'CoAP server ready: {args.host}:{args.port} (plaintext UDP)', flush=True)
    print('GET /hello; POST /echo; PUT/GET/DELETE /value; GET /slow', flush=True)
    try:
        await asyncio.get_running_loop().create_future()
    finally:
        await context.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1',
                        help='explicit local IP address; use the PC LAN address for hardware')
    parser.add_argument('--port', type=int, default=5683)
    parser.add_argument('--delay', type=float, default=0.3,
                        help='seconds before /slow replies (default: 0.3)')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()
    try:
        address = ipaddress.ip_address(args.host)
    except ValueError:
        parser.error('--host must be a numeric local IP address')
    if address.is_unspecified or address.is_multicast:
        parser.error('--host must be one local unicast address, not a wildcard or multicast')
    if not 1 <= args.port <= 65535:
        parser.error('--port must be 1..65535')
    if not 0.2 <= args.delay <= 30:
        parser.error('--delay must be 0.2..30 seconds')
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
