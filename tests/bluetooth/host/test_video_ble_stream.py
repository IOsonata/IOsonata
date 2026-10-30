#!/usr/bin/env python3
"""Exercise production video refill and Nordic DMA chunk boundaries."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


def function(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class VideoBleStream(unittest.TestCase):
    def test_refill_and_dma_boundaries(self):
        demo = (ROOT / "exemples/video/video_ble_demo.cpp").read_text()
        spi = (ROOT / "ARM/Nordic/src/spi_nrfx.cpp").read_text()
        source = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <algorithm>
using std::min;
#define NRF52_SERIES
#define SPI_PRESENT
#define NRFX_SPI_DMA_MAXCNT 255
bool errata = true;
bool nrf52_errata_58() { return errata; }
''' + function(spi, "static int nRFxSPIRxBlockSize(") + r'''
bool connected = true, events = false;
bool s_bNotif = true, s_bRun = true, s_bSession = true, s_bFrame = true;
int s_Pending = 50, s_Offset = 0;
uint8_t s_Buff[50];
struct { void Stop() {} } s_Video;
// Only the public transport operations are available to the production loop.
struct {
    bool ready = false;
    int calls = 0, accepted = 0;
    bool RequestToSend(int n) { assert(n == 1); return ready; }
    int Tx(int selector, const uint8_t *p, int n) {
        assert(selector == 0);
        calls++;
        int count = min(n, 17);
        for (int i = 0; i < count; i++) assert(p[i] == accepted + i);
        accepted += count;
        return count;
    }
} g_BtIntrf;
void BtAppCheckStatus() {}
bool isConnected() { return connected; }
bool AppEvtHandlerPending() { return events; }
void VideoOpen() { s_bSession = true; }
bool VideoFill() { return false; }
''' + function(demo, "bool AppCheckStatus(void)") + r'''
int main() {
    for (int i = 0; i < 50; i++) s_Buff[i] = i;
    assert(AppCheckStatus()); // Backpressure permits sleep, retains staging data.
    assert(g_BtIntrf.calls == 0 && s_Pending == 50 && s_Offset == 0);
    events = true;
    assert(!AppCheckStatus()); // Never sleep with a queued application event.
    events = false;
    g_BtIntrf.ready = true;
    assert(!AppCheckStatus()); // Keep polling the camera after draining staging.
    assert(g_BtIntrf.accepted == 50 && s_Pending == 0 && s_Offset == 50);
    connected = false;
    assert(AppCheckStatus());
    assert(!s_bRun && !s_bSession && !s_bFrame);
    for (int size = 2; size <= 65535; size++) {
        int remaining = size;
        while (remaining) {
            int count = nRFxSPIRxBlockSize(remaining);
            assert(count >= 2 && count <= 255 && count <= remaining);
            remaining -= count;
        }
    }
    errata = false;
    assert(nRFxSPIRxBlockSize(256) == 255);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp, exe = Path(tmp) / "stream.cpp", Path(tmp) / "stream"
            cpp.write_text(source)
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-O2",
                            "-fsanitize=address,undefined", str(cpp), "-o", str(exe)],
                           check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
