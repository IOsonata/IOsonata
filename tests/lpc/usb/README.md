# LPC11Uxx USB checks

Run `python3 tests/lpc/usb/run.py`.

- C and C++ include the production controller header through `usb/usb.h`.
- The production `usb_lpc11uxx.c` runs against mapped vendor CMSIS peripheral
  declarations at -O0 and -O2 with undefined-behavior checking. Tests cover
  invalid clocks without hardware changes, dedicated/main PLL clock paths,
  minimum AHB frequency, priority clamping, refusal of a live controller,
  endpoint-list initialization, W1C write masks and bounded EPSKIP shutdown.
- An EP0 model exercises multi-packet IN/OUT, copied IN sources, short OUT,
  zero-length packets/status, SET_ADDRESS timing and cancellation, stalls,
  superseding SETUP, reset, suspend/resume, early status, callback shutdown
  and restart, malformed lengths and a stuck endpoint skip.
- Lifecycle tests cover validation, GPIO interrupt allocation failure, attach
  while USB is stopped, repeated start/stop, priority clamping, separate NVIC
  and soft-connect control, SOF events, UID formatting/truncation and failed
  retirement followed by retry. Unmapped USB registers catch access before
  initialization or while starting without VBUS.
- Data tests cover all eight non-control endpoint directions, unaligned caller
  buffers, short packets, ZLPs, direct callback resubmission, close/reopen,
  cancellation, malformed lengths, halt/clear and failed retirement. IN tests
  cover multi-packet transfers through 65535 bytes, exact packet multiples
  without an implicit ZLP, partial progress, callback resubmission and
  cancellation or halt between packets.
- A host integration fixture links the production generic USB core, UsbIntrf,
  CFifo and DeviceIntrf to the LPC C driver. It exercises SET_ADDRESS, an
  80-byte TX, a two-packet RX, FIFO backpressure and rearming after reads.
  It also unplugs during EP0/IN/OUT traffic through both GPIO processing and
  the USB device interrupt, checks state before the application detach event,
  reconnects without reinitialization, and services a latched disconnect after
  VBUS has already returned.
- A real CDC class fixture reads device/configuration/string descriptors over
  EP0, including 64-byte replies needing a ZLP and multi-packet descriptors.
  It checks SET_ADDRESS, SET_CONFIGURATION, GET/SET_LINE_CODING and DTR/RTS,
  with line coding and port-open state committed only after the status ACK.
  An 80-byte OUT/IN loopback uses the public UsbdCdc Rx/Tx methods and a
  blocking packet FIFO, matching the example's backpressure behavior.
  Serial-state notifications use 8+2 bytes, retain an older notification while
  a new state is pending, and start the new notification from completion.
  Detach during the second notification packet closes the port; a GPIO VBUS
  interrupt then drives re-enumeration, reopening and another loopback without
  reinitializing the stack. This fixture uses the production application event
  queue and descriptor builder, with no direct UsbProcess call from the test.
- Remote-wakeup tests cover the three-frame delay, stale tick rejection,
  duplicate requests, host resume/reset/disconnect cancellation, SOF enable
  changes and preservation of pre-existing clock control. The generic-core
  fixture also checks SET/CLEAR_FEATURE and refusal without host permission.
- Host GNU binutils exercise the production linker script's USB SRAM placement,
  NOLOAD behavior, removal when unused, and failure on USB SRAM overflow.

The preparation fixture records W1C write values; ordinary mapped RAM does not
simulate W1C register semantics. For EP0 tests, the runner instruments only
DEVCMDSTAT, INTSTAT, INTSETSTAT and EPSKIP assignments in a temporary source
copy. Test write functions implement W1C, set-interrupt and skip completion;
register reads, vendor types, DMA memory and driver control flow are unchanged.
The EP0-only fixture models the core's packet continuation/status calls. The
integration fixture instead uses the real core and minimal static device and
configuration descriptor overrides, with hardware/ROM access and the application
queue mocked. It includes the public UsbRemoteWakeup path. The separate CDC
fixture retains the real descriptors and application queue; only hardware/ROM
and interrupt delivery are modeled there. These host tests
do not simulate a USB bus or validate PLL accuracy, NVIC timing or ARM relocations.

## Current implementation

`usb_lpc11uxx.c` now supplies private hardware preparation and release helpers.
It uses the existing vendor `LPC_USB_Type` and `LPC_SYSCON_Type` declarations.
The library project already compiles this source and now links its private
header. Applications continue to use the generic USB API.

Preparation requires a working external system oscillator and clock state
maintained by the LPC11Uxx SystemInit/clock setters. It checks a nominal 48 MHz
USB clock, PLL lock/power and at least 16 MHz AHB before accessing USB registers.
The physical oscillator's USB accuracy remains a board requirement. It does
not reprogram PLLs, configure board pins or connect to the host.

The linker places 832 bytes in the 2 KB USB SRAM at 0x20004000: the 80-byte
endpoint list, alignment padding, SETUP storage and ten 64-byte packet buffers.
The list is 256-byte aligned; buffers are 64-byte aligned. NOLOAD keeps this
storage outside normal startup clearing, which runs before the USB SRAM clock
is enabled. The driver initializes it after enabling the clock. Programs which
do not reference this driver discard the section with --gc-sections.

Release disconnects and requests hardware retirement of active single-buffer
endpoints. A timeout retains the buffers and clocks; it never returns their
ownership on an unconfirmed stop. Only clocks added by preparation are removed
on success. The status acknowledgment helper writes only requested W1C bits
and cannot accidentally generate remote wakeup.

EP0 now supplies UsbCtrlrEp0Send, UsbCtrlrEp0Status, UsbCtrlrSetAddress and
EP0 stall handling. USB_IRQHandler reports SETUP, EP0 completions, reset and
suspend/resume to the existing generic UsbDevProcessEvent. A cable change
queues the generic work pass. Descriptor parsing and Chapter 9 remain generic.
The private Lpc11UsbEp0Enable initializes EP0 and interrupt sources but leaves
the pull-up and NVIC disabled for the public lifecycle to enable separately.

UsbCtrlrInit requires one pin-map entry: P0.3, IOPINOP_FUNC1, input, no pull,
normal type. It allocates a GPIO interrupt for both VBUS edges. This interrupt
stays allocated across Stop so cable insertion can queue the generic process
pass while USB clocks are off. Reinitialization while stopped releases the old
pin interrupt; exhaustion of GPIO interrupt slots fails initialization.
UsbCtrlrStart requires VBUS and valid clocks, prepares EP0, and selects the fixed
P0.6 FUNC1 USB_CONNECT output. The generic core then enables NVIC and connects
after its own initialization. Boards need VBUS sensing and the external
soft-connect switch; configurations without these are not supported yet.
Stop uses the bounded retirement helper and can be retried after a timeout.
Low-power suspend requests fail initialization; normal suspend keeps clocks on.
SOF events carry the hardware frame number. The serial string uses all four
IAP UID words and is available before initialization, as the generic core needs.

EP0 OUT and SETUP share an interrupt. The handler acknowledges that interrupt
before checking SETUP and clears old EP0 ACTIVE/STALL state before acknowledging
the SETUP status. Generation checks prevent callback stop/restart from applying
old completions to new state. Buffer cancellation uses bounded EPSKIP waits.
A failed skip disconnects and disables interrupts while retaining DMA storage.

Bulk and interrupt endpoints use independent single-buffer command words and
the existing aligned SRAM packets. OUT stays NAKed until a caller supplies a
destination. Completion copies received bytes before calling the endpoint
handler directly; IN stages at most one MPS-sized packet in SRAM and advances
from its interrupt until the entire submitted buffer is sent. The caller must
keep the source valid and unchanged until completion, failure or cancellation.
Completion reports the total length once; no trailing ZLP is added implicitly.
Failure reports only bytes confirmed by earlier packet completions. Initial OUT readiness is requested from IRQ after
class setup; later completions request the next destination directly.
Closing, stopping and bus reset retire hardware before cancellation callbacks,
and block reentrant submissions during cancellation. Bindings survive reset and
Stop/Start. Halting an active transfer reports XFER_FAILED after retirement,
avoiding replay when an ACK races EPSKIP. A completion already observed before
halt advances the transfer normally; any remaining packets wait for clear. New submissions while halted wait for clear, which
resets the hardware data toggle.

Cable loss uses the same retirement sequence before resetting the generic USB
configuration and notifying application code of detach. The GPIO process pass
handles loss before the debounced device interrupt arrives. Repeated low-level
checks do not reset classes again. A latched DCON_C still invalidates old
transfers when VBUS is already high by the time the interrupt is serviced.
The controller stays prepared for reconnection; no application restart is
required. Failed retirement follows the existing disconnected fault path and
retains DMA ownership.

Remote wakeup keeps the normal clock-on suspend mode. DSUS indicates at least
3 ms of bus idle. A wake request discards the old local FRAME_INT and waits for
three millisecond timer edges, guaranteeing at least another 2 ms before
signalling. This timer is described in UM10462 table 205; it is not a count of
SOF packets received from the suspended host. No busy wait or application timer
is used. Resume, reset, disconnect and stop cancel pending wake requests.
The driver forces AP_CLK before the explicit DSUS=0 write and restores only
the bit it added on resume/cancellation. It leaves the resume waveform to the
controller and never reports a host resume merely because a request was made.
Internal timing interrupts do not produce generic SOF callbacks unless enabled.

Low-power suspend remains unfinished. ISO calls return false
and the header advertises no ISO support; the 1023-byte constant records only
the silicon limit. No application enumeration or hardware throughput result is
claimed. The host model checks register sequencing and tick counts, not physical
resume pulse duration, bus timing or suspended current consumption.

The implementation follows the existing Nordic/SAM4L separation: generic USB
owns descriptors and control requests, while the controller handles hardware.
The built-in ROM DFU bootloader remains independent; no software DFU is added.

## References and validation

- NXP LPC11U3x data sheet, section 7.9.1.1 (endpoint capabilities):
  https://www.nxp.com/docs/en/data-sheet/LPC11U3X.pdf
- NXP UM10462 revision 2.1, chapters 3 and 11 (clock gates, command/status,
  endpoint memory and skip semantics):
  https://strawberry-linux.com/pub/UM10462.pdf

The hardware source was also compiled to Cortex-M0 Thumb assembly with
arm-none-eabi-gcc. ARM link results are recorded below; board USB testing is pending.

## ARM firmware link

With Arm GNU tools on PATH, run:

```
python3 tests/lpc/usb/link.py
python3 tests/lpc/usb/link.py --prefix /path/to/bin/arm-none-eabi- --output /tmp/lpc-usb-link
```

This separate check needs newlib-nano and C++23 support. It compiles the real
the shared CDC and HID loopback examples, generic USB classes, event queue, GPIO, IAP,
Cortex-M0 atomic helpers and startup. Library objects are archived before the
application is linked against the production LPC11U35 linker script. Required
sources must also be linked into the LPC11U35 IOcomposer library project.
It checks the CDC and HID firmware dependencies, not compilation of every optional
peripheral and device driver in that library project.

Debug uses -O0 and rdimon semihosting; Release uses -Os and nosys. Both use
newlib-nano, section garbage collection, no exceptions and no RTTI. The check
verifies the first eight vector words sum to zero, the initial stack respects
the 32-byte IAP reservation, the real USB IRQ is linked, and 832 bytes of USB
DMA storage remain NOLOAD at 0x20004000. Debug must retain monitor initialization;
Release must omit it. ELF, map and binary files are retained only with --output.

Measured with Arm GNU GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 49360 | 4476 | 832 | 1224 |
| Release | 21596 | 3680 | 832 | 1492 |

Main RAM includes initialized data and BSS. The remaining main RAM is divided
by the existing linker script between heap and stack. The stack allowance is
not a measured runtime high-water mark. Sizes may change with compiler version.
The check compiles each example board.h and checks its
IOcomposer source links, C++23 setting and Debug/Release semihosting options.
See that example's README for the documented module clock and wiring requirements;
the carrier schematic does not verify the module-internal circuit.

A successful link does not establish USB enumeration or CDC operation on
hardware. The host CDC fixture checks the 10-byte serial-state notification
on its 8-byte interrupt endpoint, but physical bus validation is still pending.

The CDC fixture also runs with the shared HID class registered. It verifies
three interfaces and five unique endpoint addresses, HID report descriptor
routing and SET_IDLE status-stage commit, 64-byte HID reports while CDC data
and notifications are active, and both classes after detach/re-enumeration.
Both CDC-only and composite variants pass at -O0 and -O2 with UBSan.

The library project now links `usbd_hid.cpp` and `usb_int.cpp` and their headers.
The ARM check now links a separate UsbHidLoopback firmware that retains HID:
Debug uses 51136 bytes flash, 3420 bytes main RAM and 1576 bytes linker stack;
Release uses 22236 bytes flash, 2624 bytes main RAM and 1844 bytes linker stack.
Both use 832 bytes USB SRAM. These are standalone HID measurements, not
composite firmware measurements. Composite operation has been checked in the
host model only. Retained files are under UsbCdcLoopback/{Debug,Release} and
UsbHidLoopback/{Debug,Release} within the --output directory.


## Composite ARM firmware

The same `link.py` command also builds `firmware/composite_firmware.cpp` in
Debug and Release. This fixture registers CDC, HID and vendor bulk together, uses the CDC
example board configuration and services both through DeviceIntrf. It keeps
separate pending packets when a transport is busy. All three class initializers
and the interrupt transport must be present in the linked image.

CDC and vendor bulk each use two 64-byte receive slots and a 128-byte transmit
FIFO. HID uses one
caller-owned report slot per direction. The application event queue holds
16 events. These smaller CDC buffers explain why composite RAM is lower than
the standalone CDC example with its 1024-byte transmit FIFO.

Measured with the same GCC 13.2.1/binutils 2.42 toolchain:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM / 2048 bytes | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 55356 | 4292 | 832 | 1288 |
| Release | 24420 | 3496 | 832 | 1552 |

Debug retains rdimon semihosting; Release uses nosys. Both pass the production
linker, vector checksum, USB IRQ and IAP/USB RAM checks. With --output, the
images are retained under Composite/{Debug,Release}. This is a link fixture
with demo USB identifiers, not a hardware-validated example. Stack usage,
physical enumeration and concurrent report/serial traffic remain unmeasured.


EP0 status completion is delivered before a new SETUP when both share an IRQ.
The regression reproduces an acknowledged CDC SET_LINE_CODING immediately
followed by GET_LINE_CODING, and HID SET_IDLE followed by GET_IDLE. Previously,
only SET_ADDRESS preserved this completion, so class requests could lose an
acknowledged update. Transport tests also check that an active status packet
or a control-read data ZLP does not falsely commit the old request, and that
an early-terminated control write remains cancelled.

Control-read OUT status uses the same ordering when it completes before the
next SETUP. A HID class callback regression checks COMPLETE rather than ABORT
for the finished read. Transport regressions cover a still-active OUT status
and an early-terminated read so neither becomes a false successful completion.


The LPC11U35 library also links the shared `usbd_bulk.cpp` and its public header.
The host fixture runs a third configuration with CDC, HID and vendor bulk:
four interfaces, seven unique data endpoint addresses, and all four available
non-control IN endpoints allocated. It transfers a 64-byte vendor packet on
EP4 OUT/IN while a HID IN report remains pending, and repeats after reconnect.
CDC-only and CDC/HID configurations remain separate regression runs. All three
run at -O0 and -O2 with UBSan. The composite ARM measurements above now include
vendor bulk; the standalone CDC and HID firmware sizes are unchanged.


The ARM runner also builds the standalone UsbCustomBulkLoopback IOcomposer
example using its board header and the shared vendor-bulk application. Debug
uses 47580 bytes flash, 4448 bytes main RAM and 1236 bytes linker stack;
Release uses 20964 bytes flash, 3652 bytes main RAM and 1500 bytes linker stack.
Both use 832 bytes USB SRAM. Project links and semihosting options receive the
same checks as CDC and HID. The runner now checks eight firmware builds;
retained vendor-bulk images are under UsbCustomBulkLoopback/{Debug,Release}.
See the example README for the existing PyUSB loopback and reconfiguration
commands. These host hardware commands have not been run against a board.


The CDC, CDC/HID and CDC/HID/vendor-bulk fixtures each repeat four configuration
cycles while OUT destinations and IN transfers are active. They alternate
SET_CONFIGURATION(0) followed by (1), and repeated SET_CONFIGURATION(1).
Checks cover endpoint retirement, unchanged device address, CDC close/reopen
notifications, and clean payloads from all enabled classes afterward.
FIFO transports can accept data while unconfigured; the tests verify that no
hardware bank is armed and that these bytes are flushed on reconfiguration.
The direct HID transport refuses a report while unconfigured. Both -O0 and
-O2 UBSan runs pass; these checks required no production driver change.

## LPC54628 USB0 hardware preparation

`python3 tests/lpc/usb/run546.py` tests the private USB0 preparation and
release functions with the repository's vendor CMSIS register types. It is
also run by the USB suite. Both optimization levels exercise the 48/96 MHz
FRO divider selection, device-mode mux, SRAM layout, interrupt-priority
clamping, refusal to take over active USB clocks, divider timeout cleanup,
and refusal to release active endpoint buffers. ROM calls are modeled to
check normal-boot FRO startup, prior ROM/OTP power preservation, calibration
readback failure, and cleanup after a subsequent divider timeout. Invalid SRAM placement is
checked separately.

`usb_hw_lpc546xx.c` calibrates and starts the FRO HF output through the NXP
ROM entry when it is off. The existing 48/96 MHz selection is preserved,
and an already-running HF output is reused. ROM/OTP power and the ROM clock
are enabled only around calibration and restored afterward. It selects a
48 MHz USB0 clock and enables SOF trim without switching the CPU clock or
PLL. It leaves DEV_EN, DCON and interrupts disabled. The system clock
owner must keep that source stable until release. USB0 host and USB1 must
remain unused while this helper owns the shared USB resources.

The shared `LpcUsbFsMemory_t` layout is used by LPC11U35 and LPC54628.
LPC54628 places its 832-byte allocation in the 8 KB dedicated USB RAM using
NOLOAD, outside startup .data/.bss. Only the owned allocation is initialized;
unused USB storage is discarded by the linker.

The LPC54628 port now integrates with the generic USB core and provides EP0,
four bulk/interrupt endpoint pairs, direct endpoint callbacks in the USB0 ISR,
SOF events, and VBUS-driven start/stop and reconnect. `usb_ep0_lpc_fs.c` and
`usb_data_lpc_fs.c` are compiled by both LPC11U35 and LPC54628. Their clocking,
pin setup and IRQ entry points remain family-specific. Public controller APIs
are unchanged. Cancellation callbacks run only after DMA retirement; a failed
skip keeps memory and clocks owned until Stop can safely retry.

Init requires P0.22 FUNC7 VBUS input, no pull, normal type. A GPIO edge queues
ordinary USB processing even while USB clocks are stopped. The PHY provides
its pull-up and dedicated DM/DP pads. Other VBUS routes are not supported yet.
The ROM UID call temporarily powers ROM/OTP and restores their prior state.
USB1 high-speed, ISO, remote wakeup and low-power suspend are not implemented.
Configure `bRemoteWakeup=false` and `bLowPowerSuspend=false`.

The LPC54628 host fixtures cover EP0 ordering, all eight data endpoint
directions, unaligned buffers, multi-packet IN through 65535 bytes, halt/clear,
reset, reentrant callbacks, bounded skip failure and retained ownership.
The real generic core, AppEvt queue and CDC/HID/vendor-bulk classes also run
against LPC54628 registers. Tests cover startup without VBUS, later attachment,
enumeration, CDC line coding and notifications, payloads, cable removal and
reconnect, and repeated configuration while transfers are active. The same
class tests run against LPC11U35. Both use -O0/-O2 and UBSan.

`ARM/NXP/LPC546xx/LPC54628/exemples/UsbCdcLoopback` links the shared CDC loopback
application. Its board header selects P0.22 and disables remote wakeup in the
descriptors. The application continues to use the generic USB interfaces and
the 16-entry AppEvt queue; it does not call the controller.

`python3 tests/lpc/usb/link546.py --prefix arm-none-eabi-` checks the managed
project links, then builds the CDC application and EP0 fixture with real
startup. It verifies the strong USB0 vector, LPC checksum and 832-byte NOLOAD
USB RAM allocation. Debug retains semihosting; Release uses nosys. GNU Arm
13.2.1 produces a CDC image of 48076 bytes flash in Debug and 21628 in Release.
The ten LPC11U35 firmware links also pass after sharing data endpoints; the
largest composite Debug image uses 56428 of 65536 flash bytes. Earlier size
measurements above describe earlier builds.

Physical enumeration, cable behavior, throughput and suspend/resume still
require board testing. Host register models do not establish hardware results.

Reference sequences: NXP's LPC54628 `CLOCK_EnableUsbfs0DeviceClock` in
[MCUXpresso SDK](https://github.com/nxp-mcuxpresso/legacy-mcux-sdk/blob/main/devices/LPC54628/drivers/fsl_clock.c),
and the USB register definitions in `LPC54628.h` / `LPC54628_features.h`.
