# LPC1769 UsbCdcPrbsTx

Import `ioc` and `../../lib/ioc` into IOcomposer. Build `IOsonata_LPC1769`
first, then `UsbCdcPrbsTx`, using matching configurations. Debug defaults to
rdimon semihosting; enable it in the IDAP-Link debugger. Release uses nosys.

The project links `exemples/usb/usb_cdc_prbs_tx.cpp`. It uses the generic CDC
class, a 2048-byte TX FIFO and an explicitly initialized 16-entry event queue.
AppRun services queued events; AppCheckStatus generates PRBS bytes while the
host asserts DTR. Partial writes retain unsent bytes for the next iteration.
The application stays active while the port is open to fill the TX FIFO.

## Board setup

The board settings match UsbCdcLoopback: a 16 MHz external crystal and
32.768 kHz RTC crystal, VBUS P1.30 FUNC2, D+ P0.29 FUNC1, D- P0.30 FUNC1 and
active-low SoftConnect P2.9 FUNC1. The latter requires an external D+ pull-up
switch. Verify these settings against the board schematic. The system clock
must provide a valid 48 MHz USB clock.

Connect USB before reset. Automatic attachment after startup and reattachment
after unplugging are not implemented; reset with USB attached. Remote wake
and low-power suspend are disabled. One CDC instance uses interrupt IN EP1
and bulk IN/OUT EP2. Arbitrary composite layouts are not covered.

## Host check

With pyserial installed, run from the repository root:

```sh
python3 Python/uartprbs_rx.py --baud 1000000 --port /dev/cu.usbmodemDEVICE
```

Replace the port with the enumerated CDC port. The receiver asserts DTR and
checks the PRBS sequence while reporting throughput and sequence errors.
Baud is CDC line coding; it does not set the USB bus rate. Stop with Ctrl-C.
Run Debug and Release separately. Close/reopen the port as a separate run;
each run establishes a new sequence baseline. After cable removal, reset the
MCU with USB attached before testing again.

VID/PID 1209:0002 and the product string `IOsonata CDC PRBS Tx` are shared demo
values. Select appropriate IDs before distribution. The serial is the ROM UID.

## Validation

```sh
python3 tests/lpc/usb/link17.py --prefix arm-none-eabi-
```

The script links EP0, CDC loopback, HID and CDC PRBS examples in both modes,
checks project links, vector checksum, the USB IRQ, PRBS/CDC symbol extraction
and semihosting selection. This is a command-line build rather than a complete
IOcomposer build. Physical enumeration, throughput and PRBS integrity remain
pending hardware tests; no throughput result is claimed.

GNU Arm 13.2.1 text/data/bss: Debug 51892/460/6656 bytes;
Release 23272/156/6168 bytes. Both links pass.
