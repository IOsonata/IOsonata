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

"""Check the Ethernet echo example and host server without an MCU or lwIP."""
import asyncio
import contextlib
import importlib.util
import io
import os
import socket
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def run(args):
    subprocess.run([str(a) for a in args], cwd=ROOT, check=True)


async def server_checks():
    spec = importlib.util.spec_from_file_location("net_echo", ROOT / "Python/net_echo.py")
    echo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(echo)
    loop = asyncio.get_running_loop()
    for quiet in (False, True):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            udp, _ = await loop.create_datagram_endpoint(
                lambda: echo.UdpEcho(quiet), local_addr=("127.0.0.1", 0))
            server = await asyncio.start_server(
                lambda r, w: echo.tcp_echo(r, w, quiet), "127.0.0.1", 0)
            client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client.setblocking(False)
            writer = None
            try:
                for size in (1, 15, 16, 17, 255, 256, 257, 1024, 1472):
                    data = bytes(i % 256 for i in range(size))
                    await loop.sock_sendto(client, data, udp.get_extra_info("sockname"))
                    reply, _ = await asyncio.wait_for(loop.sock_recvfrom(client, 4096), 2)
                    assert reply == data
                reader, writer = await asyncio.open_connection(
                    "127.0.0.1", server.sockets[0].getsockname()[1])
                for size in (1, 1472, 4096, 8193):
                    data = bytes(i % 256 for i in range(size))
                    # Exercise a stream across several client writes.
                    for start in range(0, size, 71):
                        writer.write(data[start:start + 71])
                    await writer.drain()
                    assert await asyncio.wait_for(reader.readexactly(size), 2) == data
            finally:
                if writer is not None:
                    writer.close()
                    await writer.wait_closed()
                client.close()
                udp.close()
                server.close()
                await server.wait_closed()
        assert bool(output.getvalue()) == (not quiet)
    print("Host UDP/TCP binary echo, normal/quiet output: PASS")


def main():
    os.environ.setdefault('ASAN_OPTIONS', 'detect_leaks=0')
    inc = ["-I" + str(ROOT / p) for p in (
        "include", "ARM/ST/include", "ARM/ST/STM32H7xx/include",
        "ARM/CMSIS/Core/Include")]
    with tempfile.TemporaryDirectory(prefix="iosonata-echo-") as td:
        for proto in ("Udp", "Tcp"):
            board = ROOT / f"ARM/ST/STM32H7xx/STM32H753/exemples/Ethernet{proto}Demo/src"
            for count in (0, 6000):
                run(["g++", "-std=gnu++23", "-Wall", "-Werror", "-fsyntax-only",
                     *inc, "-I" + str(board), "-DSTM32H753xx",
                     f"-DNET_DEMO_TEST_COUNT={count}", ROOT / "exemples/net/ethernet_demo.cpp"])
            for opt in ("O0", "O2"):
                exe = Path(td) / (proto + opt)
                run(["g++", "-std=gnu++23", "-Wall", "-Werror", "-" + opt,
                     "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                     "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                     "-fno-pie", "-no-pie", *inc, "-I" + str(board), "-DSTM32H753xx",
                     "-DNET_DEMO_TEST_COUNT=100",
                     ROOT / "tests/net/echo_demo_test.cpp", "-o", exe])
                run([exe])
    asyncio.run(server_checks())


if __name__ == "__main__":
    main()
