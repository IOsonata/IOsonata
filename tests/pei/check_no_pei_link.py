#!/usr/bin/env python3
"""Fail if a non-PEI firmware pulls PEI archive members or PEI code.

Usage:
  python3 tests/pei/check_no_pei_link.py path/to/TimerDemo.map

Run against a fresh GNU ld map generated with --gc-sections. Do not
interpret PEI entries in 'Discarded input sections' as linked code.
"""
import argparse
import pathlib
import re
import sys

MEMBER = re.compile(r"\.a\((?:[^()\n]*/)?pei(?:_[a-zA-Z0-9]+)?(?:\.(?:c|cpp))?\.o\)")
CODE = re.compile(r"^\s*\.text\.(?:Pei[A-Za-z0-9_]*|[^\s]*Pei(?:Trigger|Event|Install)[^\s]*|IOPin(?:Enable|Disable)PortEvent|AdcNrf52(?:PreparePeiSampling|Pei[A-Za-z0-9_]*)|AnalogCompPei[A-Za-z0-9_]*|PWM(?:EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|Pdm(?:PreparePeiStart|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|I2S(?:PreparePeiStart|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|SPI(?:PreparePeiTransfer|ReleasePeiTransfer|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|I2C(?:PreparePeiTransfer|ReleasePeiTransfer|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|UART(?:PreparePeiTx|ReleasePeiTx|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask)|Qdec(?:PreparePeiStart|EnablePeiEvent|DisablePeiEvent|ConnectPeiTask|DisconnectPeiTask))(?=\s|$)", re.M)
HF_STATE = re.compile(r"^\s*\.bss\.[^\s]*s_nRFxTimer[0-9]+Pei\b", re.M)
RTC_STATE = re.compile(r"^\s*\.bss\.[^\s]*(?:s_nRfxRtc(?:[0-9]+Pei|Raw)|g_PeiNrf52RtcRaw)\b", re.M)


def check(path):
    data = path.read_text(encoding="utf-8", errors="replace")
    if "Archive member included to satisfy reference" not in data:
        raise ValueError("Not a GNU ld archive extraction map")
    if "Linker script and memory map" not in data:
        raise ValueError("GNU ld memory map section is missing")
    archive = data.split("Discarded input sections", 1)[0]
    linked = data.split("Linker script and memory map", 1)[1]

    pulled = [line.strip() for line in archive.splitlines()
              if MEMBER.search(line)]
    code = CODE.findall(linked)
    state = HF_STATE.findall(linked)
    rtc_state = RTC_STATE.findall(linked)

    if pulled:
        print("FAIL: PEI archive members were extracted:")
        for line in pulled:
            print(" ", line)
    if code:
        print("FAIL: PEI-specific text sections remain in the firmware:")
        for line in code:
            print(" ", line.strip())
    if state:
        print("FAIL: obsolete HF Timer PEI state arrays remain:")
        for line in state:
            print(" ", line.strip())
    if rtc_state:
        print("FAIL: RTC PEI state is retained by an ordinary Timer:")
        for line in rtc_state:
            print(" ", line.strip())
    if pulled or code or state or rtc_state:
        return 1

    print("PASS: no PEI archive members or PEI code retained")
    print("PASS: no PEI-only HF Timer or RTC bookkeeping arrays")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=pathlib.Path,
                        help="GNU ld map for an application not using PEI")
    args = parser.parse_args()
    if not args.map.is_file():
        parser.error("Missing linker map: " + str(args.map))
    try:
        return check(args.map)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
