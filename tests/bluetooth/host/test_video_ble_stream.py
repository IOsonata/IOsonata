#!/usr/bin/env python3
"""Exercise production video double buffering and Nordic DMA boundaries."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class VideoBleStream(unittest.TestCase):
    def test_refill_and_dma_boundaries(self):
        spi = (ROOT / 'ARM/Nordic/src/spi_nrfx.cpp').read_text()
        for demo_name, profiling in (('video_ble_demo.cpp', 1), ('video_ble_demo.cpp', 0), ('video_usb_demo.cpp', 0)):
            with self.subTest(demo=demo_name, profiling=profiling):
                demo = (ROOT / 'exemples/video' / demo_name).read_text()
                declarations = demo[demo.index('typedef struct {\n\tuint8_t Data'):]
                declarations = declarations[:declarations.index('static int s_ReadHeader = 0;')]
                declarations += 'static int s_ReadHeader = 0;\n'
                defines = '\n'.join(re.findall(r'^#define (?:PACKET_SIZE|VIDEO_DEMO_BUFSIZE)\s+[^\n]+', demo, re.M))
                profile = ''
                if demo_name == 'video_ble_demo.cpp':
                    profile = '#define VIDEO_DEMO_PROFILE %d\n' % profiling + demo[demo.index('#ifndef VIDEO_DEMO_PROFILE'):demo.index('void ReadCharSetNotif(', demo.index('#ifndef VIDEO_DEMO_PROFILE'))]
                source = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
using std::min;
#define NRF52_SERIES
#define SPI_PRESENT
#define NRFX_SPI_DMA_MAXCNT 255
bool errata = true;
bool nrf52_errata_58() { return errata; }
''' + function(spi, 'static int nRFxSPIRxBlockSize(') + '\n' + defines + r'''
uint32_t clockMs = 0;
uint32_t BtGattMsTick() { return clockMs; }
bool connected = true, events = false;
bool s_bNotif = true, s_bRun = true, s_bSession = false, s_bFrame = false;
const uint8_t s_FrameTag[4] = {'V','F','R','M'};
''' + declarations + r'''
struct Camera {
    bool pending = false, done = false, inlineRead = false;
    int remaining = 0, frames = 0, offset = 0, length = 0, starts = 0;
    bool refuse = true;
    uint8_t *target = nullptr;
    void Stop() {}
    void Start() { assert(!pending); remaining = frames = offset = 0; refuse = true; }
    bool UpdateData() {
        if (remaining || frames == 2) return false;
        remaining = 2500; offset = 0; frames++; return true;
    }
    uint32_t FrameLength() { return remaining ? 2500 : 0; }
    void Complete() {
        assert(pending && !done);
        for (int i = 0; i < length; i++) target[i] = (uint8_t)(offset + i);
        done = true;
    }
    int ReadFrameComplete() {
        if (!pending) return 0;
        if (!done) return -1;
        pending = false; remaining -= length; offset += length;
        return length;
    }
    int ReadFrameAsync(uint8_t *p, int n) {
        assert(!pending && remaining > 0);
        if (refuse) { refuse = false; return 0; }
        pending = true; done = false; target = p;
        length = min(n, remaining); starts++;
        if (inlineRead) { Complete(); return ReadFrameComplete(); }
        return -1;
    }
} s_Video;
std::vector<uint8_t> sent;
bool overlap = false;
struct Transport {
    bool ready = true;
    bool IsPortOpen() { return connected; }
    bool RequestToSend(int n) { assert(n == 1); return ready; }
    int Tx(int, const uint8_t *p, int n) {
        if (!ready) return 0;
        overlap |= s_Video.pending && sent.size() > 1;
        int count = min(n, 17);
        sent.insert(sent.end(), p, p + count);
        return count;
    }
} g_BtIntrf;
Transport &g_Cdc = g_BtIntrf;
void BtAppCheckStatus() {}
void UsbCheckStatus() {}
bool isConnected() { return connected; }
bool AppEvtHandlerPending() { return events; }
''' + profile + r'''
void VideoOpen() {
    assert(!s_Video.pending);
    for (auto &b : s_Buff) { b.Len = 0; b.Offset = 0; }
    s_TxIdx = 0; s_FillIdx = 1; s_ReadHeader = 0;
    s_Buff[0].Data[0] = 'S'; s_Buff[0].Len = 1;
    s_bSession = s_bRun = true; s_bFrame = false;
    s_Video.Start();
#if VIDEO_DEMO_PROFILE
    VideoProfileReset();
#endif
}
''' + function(demo, 'static void VideoFill()') + '\n' + function(demo, 'bool AppCheckStatus(void)') + r'''
int main() {
    std::vector<uint8_t> expected = {'S'};
    for (int frame = 0; frame < 2; frame++) {
        const uint8_t header[] = {'V','F','R','M',0xc4,9,0,0};
        expected.insert(expected.end(), header, header + 8);
        for (int i = 0; i < 2500; i++) expected.push_back((uint8_t)i);
    }
    for (bool inlineRead : {false, true}) {
        s_Video.inlineRead = inlineRead;
        s_bSession = false;
        sent.clear(); overlap = false;
        for (int step = 0; step < 2000 && sent.size() < expected.size(); step++) {
            if (s_Video.pending && !s_Video.done) s_Video.Complete();
            g_BtIntrf.ready = step % 7 != 0;
            AppCheckStatus();
        }
        assert(sent == expected);
        if (!inlineRead) assert(overlap);
        assert(!s_Video.pending);
    }
#if VIDEO_DEMO_PROFILE
    assert(s_Profile.Frames == 2 && s_Profile.Bytes == 5000);
    // Report insertion preserves JPEG ordering and is emitted as plain text.
    clockMs = 6000;
    AppCheckStatus();
    std::string report(sent.begin() + expected.size(), sent.end());
    assert(report.find("Video profile ms=6000 frames=2 bytes=5000 ") == 0);
    assert(report.substr(report.size() - 2) == "\r\n");
    // Millisecond wrap and repeated failed attempts count one stall interval.
    VideoProfileReset();
    clockMs = 0xfffffff0U;
    VideoProfileBle(true);
    VideoProfileCapture(false);
    clockMs = 4;
    VideoProfileBle(true);
    VideoProfileBle(false);
    VideoProfileCapture(true);
    assert(s_Profile.BleWait == 20 && s_Profile.Stalls == 1);
    assert(s_Profile.CaptureWait == 20);
    s_Profile.SpiStart = clockMs;
    clockMs += 7;
    VideoProfileRead(0);
    assert(s_Profile.SpiWait == 7);
    clockMs = 0;
#endif
    // Disconnect/reconnect cannot reuse an in-flight DMA destination.
    s_Video.inlineRead = false;
    s_bSession = false;
    g_BtIntrf.ready = false;
    AppCheckStatus();
    AppCheckStatus(); // First read attempt is deliberately refused.
    assert(s_Video.pending && s_ReadIdx >= 0);
    int starts = s_Video.starts;
    uint8_t saved[sizeof(s_Buff)];
    memcpy(saved, s_Buff, sizeof(s_Buff));
    connected = false;
    assert(AppCheckStatus());
    assert(s_Video.pending && s_ReadIdx >= 0);
    assert(memcmp(saved, s_Buff, sizeof(s_Buff)) == 0);
    connected = true;
    assert(AppCheckStatus());
    assert(!s_bSession && s_Video.starts == starts);
    s_Video.Complete();
    AppCheckStatus();
    AppCheckStatus();
    assert(s_bSession && s_Video.starts == starts + 1);
    events = true;
    assert(!AppCheckStatus());
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
                    cpp, exe = Path(tmp) / 'stream.cpp', Path(tmp) / 'stream'
                    cpp.write_text(source)
                    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                                    str(cpp), '-o', str(exe)], check=True)
                    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
