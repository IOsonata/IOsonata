#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026, I-SYST inc., all rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""UDP and TCP echo server for the Ethernet socket examples."""
import argparse
import asyncio


class UdpEcho(asyncio.DatagramProtocol):
    def __init__(self, quiet=False):
        self.quiet = quiet

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        self.transport.sendto(data, addr)
        if not self.quiet:
            print('UDP', addr, len(data), repr(data), flush=True)


async def tcp_echo(reader, writer, quiet=False):
    addr = writer.get_extra_info('peername')
    try:
        while data := await reader.read(4096):
            writer.write(data)
            await writer.drain()
            if not quiet:
                print('TCP', addr, len(data), repr(data), flush=True)
    except (ConnectionError, OSError):
        pass
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except (ConnectionError, OSError):
            pass


async def main(host, port, quiet=False):
    loop = asyncio.get_running_loop()
    udp, _ = await loop.create_datagram_endpoint(lambda: UdpEcho(quiet), local_addr=(host, port))
    try:
        server = await asyncio.start_server(lambda r, w: tcp_echo(r, w, quiet), host, port)
        print(f'UDP and TCP echo listening on {host}:{port}', flush=True)
        async with server:
            await server.serve_forever()
    finally:
        udp.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=2469)
    parser.add_argument('--quiet', action='store_true', help='suppress per-packet output for binary echo tests')
    args = parser.parse_args()
    try:
        asyncio.run(main(args.host, args.port, args.quiet))
    except KeyboardInterrupt:
        pass
