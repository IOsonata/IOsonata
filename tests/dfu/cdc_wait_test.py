"""Inject USB interrupts between the last empty poll and the stage 0 idle wait."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
boot = (root / "exemples/dfu/dfu_boot_main.cpp").read_text()
start = boot.index("static void DfuBootLinkWait(void)")
wait = boot[start:boot.index("\n}", start) + 2]

model = r"""
#include <cassert>
#include <cstdint>

enum Arrival { BeforeMask, DuringFifoCheck, DuringEventCheck, AtWait, None };
enum Work { OutPacket, DeferredEvent };
static Arrival arrival;
static Work work;
static bool masked, pending, rx, event, waited, slept;

struct UsbData { int hRxFifo; };
struct UsbdCdcDev_t { UsbData *pData; };
static UsbData data;
static UsbdCdcDev_t cdc = { &data };
static UsbdCdcDev_t *s_Link = &cdc;

static void dispatch()
{
    if (pending && !masked)
    {
        pending = false;
        if (work == OutPacket)
            rx = true;
        else
            event = true;
    }
}

static void inject(Arrival at)
{
    if (arrival == at)
    {
        pending = true;
        dispatch();
    }
}

static uint32_t DisableInterrupt()
{
    inject(BeforeMask);
    masked = true;
    return 0;
}

static void EnableInterrupt(uint32_t state)
{
    assert(state == 0);
    masked = false;
    dispatch();
}

static void *CFifoPeek(int)
{
    inject(DuringFifoCheck);
    return rx ? &data : nullptr;
}

static bool AppEvtHandlerPending()
{
    inject(DuringEventCheck);
    return event;
}

static void __DSB() {}
static void __WFI()
{
    inject(AtWait);
    waited = true;
    slept = !pending;
}
"""

cases = r"""
int main()
{
    for (int w = OutPacket; w <= DeferredEvent; ++w)
    {
        for (int a = BeforeMask; a <= None; ++a)
        {
            work = Work(w);
            arrival = Arrival(a);
            masked = pending = rx = event = waited = slept = false;
            DfuBootLinkWait();
            assert(!masked && !pending);
            assert(slept == (arrival == None));
            assert(waited == (arrival != BeforeMask));
            assert(rx == (arrival != None && work == OutPacket));
            assert(event == (arrival != None && work == DeferredEvent));
        }
    }
}
"""

with tempfile.TemporaryDirectory() as tmp:
    source = Path(tmp) / "cdc_wait.cpp"
    binary = Path(tmp) / "cdc_wait"
    source.write_text(model + wait + cases)
    subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall",
                    "-Wextra", "-Werror", str(source), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

print("PASS: CDC recovery wait handles queued work and interrupts at the sleep boundary")
