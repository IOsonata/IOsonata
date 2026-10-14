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
USB1 high-speed, ISO and low-power suspend are not implemented.
Configure `bLowPowerSuspend=false`. Remote wakeup supports normal clock-on
suspend and retains the generic core's host-permission checks. A request
discards stale FRAME_INT and waits for three local millisecond edges after
DSUS, ensuring at least 5 ms of bus idle before requesting resume. It sets
FORCE_NEEDCLK before clearing DSUS and leaves pulse generation to hardware.
Resume, reset, cable loss, disconnect, stop and controller faults cancel the
request and release only clock forcing added by that request. Internal timing
does not generate application SOF callbacks unless they are enabled.

The LPC54628 host fixtures cover EP0 ordering, all eight data endpoint
directions, unaligned buffers, multi-packet IN through 65535 bytes, halt/clear,
reset, reentrant callbacks, bounded skip failure and retained ownership.
The real generic core, AppEvt queue and CDC/HID/vendor-bulk classes also run
against LPC54628 registers. Tests cover startup without VBUS, later attachment,
enumeration, CDC line coding and notifications, payloads, cable removal and
reconnect, and repeated configuration while transfers are active. The same
class tests run against LPC11U35. Both use -O0/-O2 and UBSan.
Remote-wakeup fixtures check delayed signalling, duplicate requests, stale
interrupts, clock ownership, cancellation and user SOF changes. The real
generic-core fixture checks host permission through SET/CLEAR_FEATURE and
the public UsbRemoteWakeup call for both controllers.

`ARM/NXP/LPC546xx/LPC54628/exemples/UsbCdcLoopback` links the shared CDC loopback
application. Its board header selects P0.22 and advertises remote wakeup in
the descriptors. The loopback does not itself generate wake requests. The application continues to use the generic USB interfaces and
the 16-entry AppEvt queue; it does not call the controller.

`python3 tests/lpc/usb/link546.py --prefix arm-none-eabi-` checks the managed
project links, then builds the CDC application and EP0 fixture with real
startup. It verifies the strong USB0 vector, LPC checksum and 832-byte NOLOAD
USB RAM allocation. Debug retains semihosting; Release uses nosys. GNU Arm
13.2.1 produces a CDC image of 48396 bytes flash in Debug and 21860 in Release.
The ten LPC11U35 firmware links also pass after sharing data endpoints; the
largest composite Debug image uses 56424 of 65536 flash bytes. Earlier size
measurements above describe earlier builds.

Physical enumeration, cable behavior, throughput and suspend/resume still
require board testing. Host register models do not establish hardware results.

Reference sequences: NXP's LPC54628 `CLOCK_EnableUsbfs0DeviceClock` in
[MCUXpresso SDK](https://github.com/nxp-mcuxpresso/legacy-mcux-sdk/blob/main/devices/LPC54628/drivers/fsl_clock.c),
and the USB register definitions in `LPC54628.h` / `LPC54628_features.h`.

## LPC54628 HID application

`ARM/NXP/LPC546xx/LPC54628/exemples/UsbHidLoopback` links the shared HID
loopback application with the same USB0 wiring as the CDC project. It echoes
64-byte reports through the generic HID class and interrupt transport. Its
README documents the existing `Python/usb_hid_loopback.py` hardware check.
Remote wakeup and low-power suspend are disabled in this application.

`link546.py` now builds EP0, CDC and HID in Debug and Release, checks both
example project source links and settings, and verifies that the HID class
and interrupt transport remain linked. All images use the production startup
and linker script, including the USB0 vector, checksum and 832-byte NOLOAD
USB SRAM checks. Debug keeps semihosting and Release uses nosys.
Hardware enumeration and report traffic still require board testing.

## LPC54628 custom bulk application

`ARM/NXP/LPC546xx/LPC54628/exemples/UsbCustomBulkLoopback` links the shared
vendor-bulk loopback application with USB0 VBUS on P0.22. Its README documents
the existing PyUSB payload and reconfiguration checks. Remote wakeup and
low-power suspend are disabled in this application.

`link546.py` also builds this example in Debug and Release and checks that
UsbdBulk remains linked, bringing the runner to eight firmware links across
EP0, CDC, HID and custom bulk. Project source links, C++23, semihosting,
USB0 vector, checksum and 832-byte NOLOAD USB SRAM checks apply to each
application. Physical enumeration and bulk traffic remain untested.

## LPC54628 composite ARM firmware

`link546.py` now also builds the shared CDC/HID/vendor-bulk composite fixture
in Debug and Release, using the LPC54628 CDC board header. The fixture uses
the board oscillator configuration only when one is supplied, so the same
source still builds for LPC11U35. No separate application implementation or
controller changes are needed.

The runner checks ten LPC54628 firmware links. The composite must retain all
three class initializers and the interrupt transport. Each image checks main
RAM use, positive linker stack allowance and the 32-byte IAP reservation in
addition to USB SRAM, vectors, checksum and semihosting. The linker stack
allowance is not a measured runtime high-water mark.

This extends firmware link coverage; the existing host model already checks
the three classes together, seven data endpoint addresses, transfers and
reconfiguration. Physical simultaneous traffic remains pending board testing.

With Arm GNU GCC 13.2.1, LPC54628 composite Debug uses 53784 bytes flash,
4316 bytes main RAM and 832 bytes USB SRAM; Release uses 24484, 3520 and
832 bytes respectively. Linker stack allowances are 20392 and 20660 bytes.
The shared LPC11U35 composite also passes both links: Debug uses 56424 bytes
flash and Release uses 24884 bytes. Debug retains semihosting on both MCUs;
Release omits it.

## LPC54628 CDC PRBS transmitter

`ARM/NXP/LPC546xx/LPC54628/exemples/UsbCdcPrbsTx` links the shared CDC PRBS
application with USB0 VBUS on P0.22. It uses a 2048-byte transmit FIFO and
16-entry AppEvt queue. Its README documents the existing serial PRBS receiver,
DTR requirement and separate close/reopen and cable-reconnect runs.

`link546.py` builds this application in Debug and Release and checks that the
PRBS generator is linked. The LPC54628 library project already includes
`prbs.c`; the firmware runner now compiles it too. This brings the runner to
twelve links across EP0, CDC loopback, CDC PRBS, HID, custom bulk and composite
firmware. Debug retains semihosting and Release uses nosys. Physical PRBS
integrity, throughput and reconnect behavior remain pending board testing.

## LPC1769 USB hardware preparation

`python3 tests/lpc/usb/run17.py` tests the new private clock/power helper,
`usb_hw_lpc17xx.c`, using vendor `LPC_SC_TypeDef` and `LPC_USB_TypeDef`
register declarations. The main USB host suite also invokes it. This is the
initial LPC1769 USB support, not a usable generic USB device driver. Pins,
EP0 IRQ/control stages, data endpoints and lifecycle integration remain to be added.

Preparation accepts the board's main oscillator frequency and validates the
existing PLL0 or PLL1 status for exactly 48 MHz. It requires a ready main
oscillator and SystemCoreClock of at least 18 MHz; the clock owner must keep
these settings stable. It does not reprogram PLLs. An already-powered USB
block is refused to avoid taking over host, OTG or another device user.
It powers USB, enables device/AHB clocks and polls their ready bits with a
signed bounded counter. Timeout removes its clock/power request. Interrupt
mask and unrelated PCONP bits are preserved. Release refuses enabled endpoint,
DMA or interrupt state; it does not cancel transfers or disconnect a device.

Host tests pass at -O0/-O2 with UBSan for PLL selection, exact rate, PLL lock,
PLL1 CCO range, oscillator readiness, low core speed, clock timeout, exclusive
ownership and guarded release. The production helper compiles for Cortex-M3
at -O0/-Os with Arm GNU 13.2.1. These are not firmware links or board tests.
The LPC1769 library project includes the helper and its private header.

Reference: NXP UM10360 revision 2, chapters 4 and 11, particularly the USB
clock control and initialization sequence:
https://www.pololu.com/file/0J431/user.manual.lpc17xx.pdf

The private LPC1769 helper now supplies command-only, one-byte write and
one-byte read SIE operations using UM10360 section 11.12. The complete
command/data sequence preserves PRIMASK, clears only CCEMPTY/CDFULL, and
uses signed bounded waits. Failed reads leave the caller's byte unchanged.
A timeout latches failure until release and successful preparation; callers
must treat device state as unknown and reinitialize it, not replay a failed
command automatically. These helpers do not implement two-byte SIE reads.

`run17.py` adds -O0/-O2 UBSan checks for phase encoding, command-only/write/read
ordering, stale flags, unrelated interrupt preservation, missing clocks,
null output, command/data timeouts, late completion rejection and interrupt
mask restoration. Temporary source instrumentation models only W1C and
USBCmdCode writes; vendor register reads and driver control flow are retained.
The updated source also compiles for Cortex-M3 at -O0/-Os. Enumeration and
hardware SIE timing remain untested.

LPC1769 now has private 64-byte EP0 OUT/IN realization and CPU FIFO helpers.
Realization must run while disconnected, before enabling endpoint interrupts;
it cannot be used to resize a live controller. Reads stage at most 64 bytes
and publish them only after Clear Buffer confirms that no SETUP overwrote
the packet. Return -2 tells the future IRQ layer to clear PO and read the new
SETUP; no unbounded retry loop runs here. Insufficient destination capacity
leaves the packet queued. Writes refuse full/stalled IN buffers and validate
a single packet after copying it. Validation does not mean host acknowledgement;
a later SETUP may invalidate the packet.

Packet helpers preserve PRIMASK, explicitly clear FIFO access mode (including
zero-length packets), and handle unaligned buffers and final partial words
without word overreads. Realization and packet-ready timeouts latch controller
failure through the existing SIE fault state. The caller must retire endpoints
before the existing guarded power release.

The packet model checks lengths 0 through 64, ZLP, buffer guards, insufficient
capacity, SETUP overwrite, busy IN, invalid lengths and realization/FIFO/SIE
timeouts at -O0/-O2 with UBSan. Cortex-M3 -O0/-Os compilation passes. No generic
USB controller entry points or enumeration result are claimed yet.

`Lpc17UsbEp0Ack` now performs the endpoint-interrupt select/clear sequence
from UM10360 section 11.10.3.3. It clears one physical EP0 direction at a time,
waits for fresh CDFULL, and returns the raw SIE status byte. Other endpoint
interrupts and unrelated device flags are preserved. It also allows OUT
STP/PO clearing during overwrite recovery without requiring a pending or
enabled interrupt. The caller must interpret the returned status before
processing a packet or reporting completion.

The operation preserves PRIMASK and leaves output unchanged on failure.
Timeouts use the existing latched SIE fault. The packet model covers both
directions pending together, stale CDFULL, raw status, null output, unavailable
power/clocks, timeout, late completion and release/prepare recovery at -O0/-O2
with UBSan. Cortex-M3 -O0/-Os compilation passes. IRQ dispatch and control
transfer integration remain to be implemented; no board validation was run.

`Lpc17UsbEp0Collect` captures enabled, pending EP0 directions once and obtains
their SIE status before the future dispatcher invokes callbacks. It captures
IN before OUT so the dispatcher can consider the observed SETUP status before
reporting IN completion. It does not itself interpret a captured IN interrupt
as successful transmission: NAK and SETUP invalidation still require handling.
Directions absent from the initial snapshot are left pending, and disabled
or non-control endpoints are untouched. Device summary flags and bus reset
remain the responsibility of the future top-level ISR.

Both status bytes are published together only after successful collection.
If the second acknowledgement fails, the first may already have cleared its
hardware interrupt, but output remains unchanged and the latched SIE fault
prevents replay. Tests cover all 16 pending/enabled EP0 combinations, IN/OUT
ordering, an OUT arrival during IN acknowledgement, mask restoration, and
failure at either acknowledgement. Host -O0/-O2 UBSan tests and Cortex-M3
-O0/-Os compilation pass. Generic control-transfer dispatch remains unfinished.


LPC1769 private control event dispatch
-------------------------------------

`usb_control_lpc17xx.c` uses the shared LPC `UsbCtrlrEvt_t` definitions and
calls Event directly in interrupt context, following the existing LPC ports.
The generic USB core still owns Chapter 9 requests, descriptors and classes.
The private module handles SETUP, bounded 64-byte IN submissions, OUT data,
status stages, control stalls and software reset. Its fixed OUT packet buffer
is valid only during the callback. The LPC1769 library project links the new
source/header and the existing shared event header.

SETUP replaces the old transfer, including an unacknowledged control-read ZLP.
An acknowledged status IN completes the preceding request first, even if a
new SETUP arrives after collection. NAK or full-buffer indications never
complete IN. FIFO overwrite defers delivery until the next interrupt handles
the newer SETUP. A callback that resets/disables the controller prevents
remaining events from the old generation. Failed SIE/FIFO operations disable
software control state and call Fault; the owning controller must stop and
reinitialize hardware. Software reset alone does not retire hardware buffers.

The controller initializer must disable control NAK interrupts. This module
does not yet implement early control-write termination through an IN NAK.
Controller initialization, bus events, address handling and the top-level IRQ
entry still need integration before enumeration can be tested. The event
module is not a complete `UsbCtrlr*` implementation.

`control17_test.c` checks SETUP replacement, packet-size limiting, callback
resubmission, multi-packet OUT, simultaneous IN/OUT, coalesced status/SETUP,
SETUP arriving after capture, data ZLP cancellation, malformed SETUP stalls,
overwrite recovery, callback reset, NAK/full filtering and fault suppression.
It uses real shared event types and vendor CMSIS registers, mocking only the
private FIFO/SIE operations. `run17.py` runs it at -O0/-O2 with UBSan alongside
the existing hardware-operation models. The new module also compiles for
Cortex-M3 at -O0/-Os. These checks are not a full firmware link or board test.

The private control module now provides `Lpc17UsbControlSetAddress`. Following
UM10360 section 11.12.1, it writes SIE command 0xD0 before status IN, with DEV_EN
set even for address zero. The hardware defers the address change until status;
software confirms the address before delivering the acknowledged status event.
A new SETUP or stall restores the previously confirmed address if the status
was not acknowledged. A coalesced status acknowledgement and SETUP preserves
the newly confirmed address. Software bus reset returns address tracking to
zero and must accompany hardware reset; ordinary SETUP retains the address.

Address tests cover 0/127, rejection of 128, DEV_EN encoding, pending status,
NAK rejection, coalesced SETUP, interrupted status, stall restoration, software
reset, interrupt-mask preservation and SIE failure during programming or
restoration. `run17.py` passes at -O0/-O2 with UBSan; Cortex-M3 compilation
passes at -O0/-Os. Public controller wrappers, initialization and bus IRQ
handling remain to be connected. Address timing still needs board validation.

`Lpc17UsbDeviceInit` now initializes a prepared, exclusively owned block:
SoftConnect inactive, 64-byte EP0, always-on USB clock mode with NAK interrupts
disabled, unconfigured device, address zero with DEV_EN set, and cleared EP0
stall/interrupt state. It refuses prior EP0 realization, enabled interrupts,
DMA or realized data endpoints. It leaves all interrupt enables unchanged at
zero. Board pin configuration, NVIC setup and connection remain the caller's
responsibility. A partial initialization failure latches the existing fault
and prevents packet operations until release/preparation and reinitialization.

`Lpc17UsbDeviceStatus` captures a pending bus-status byte using the UM10360
11.12.7 ordering: clear DEV_STAT first, then read SIE command 0xFE. It leaves
output unchanged on no event or failure, preserves unrelated flags, and does
not clear a newly arriving DEV_STAT after the read. The caller must handle bus
reset before endpoint traffic; this helper does not itself dispatch bus events.

The register model validates initialization command order/data, refusal and
repeat guards, failure at all twelve command/data phases and both endpoint
acknowledgements, bus-status ordering, an event arriving during the read,
unrelated-flag preservation and timeout handling. `run17.py` passes at -O0/-O2
with UBSan; the hardware source compiles for Cortex-M3 at -O0/-Os. Full IRQ
wiring, reset recovery and firmware enumeration remain unfinished and untested
on hardware.

`Lpc17UsbBusReset` restores EP0 only after the caller has captured an RST bus
event. It temporarily masks device/endpoint interrupts, retires stale EP0
interrupts (which hardware bus reset does not clear), realizes 64-byte EP0,
restores the no-NAK clock mode and clears control stalls. On success it restores
the previous interrupt enables without clearing events arriving during
restoration. This CPU-only path requires DMA off and no enabled data endpoints.
If the first SETUP is already buffered when recovery begins, it faults instead
of resizing live RAM and silently losing that request. Fault recovery still
requires the owning controller to disconnect and reinitialize.

`Lpc17UsbControlBusReset` first invalidates the old software transfer and
address state, then performs hardware recovery. It emits the shared RESET
event only on success. Failure calls Fault with software state disabled.
The top-level bus IRQ must invoke it before dispatching endpoint interrupts.

Tests cover repeated reset, mask restoration, stale completion retirement,
events during endpoint realization, SIE/realization timeout, delayed SETUP,
address reset, callback disable and failure without a RESET notification.
Host -O0/-O2 UBSan checks and both sources' Cortex-M3 -O0/-Os compilation pass.
Top-level IRQ wiring and hardware enumeration remain outstanding.

`usb_irq_lpc17xx.c` now supplies the strong `USB_IRQHandler` referenced by
the LPC1769 vector table. `Lpc17UsbIrqBind` installs the initialized control
object before IRQ enable; unbind only after disabling hardware/NVIC interrupts.
Referencing the bind function also brings this IRQ object into a static-library
link. The library project includes the source.

The handler services DEV_STAT first. Reset recovers EP0 and ends the current
IRQ without dispatching the pre-reset endpoint snapshot. Suspend/resume use
the existing direct event callback. Callback disable, unbind or generation
change prevents further dispatch. VBUS loss and status-read failure mask
device/endpoint interrupts, disable software state and call Fault; the owner
must disconnect/stop and arrange reinitialization. The controller uses slow interrupt routing without DMA/SOF support.

Endpoint summary is cleared only after all endpoint flags are clear, followed
by a recheck that restores the summary if an endpoint arrived across the W1C
write. There is no drain loop. CCEMPTY/CDFULL stay owned by the SIE polling
operations. Enabled unsupported device sources cause a stop instead of an
unhandled interrupt storm.

`irq17_test.c` covers reset priority, suspend/resume, callback shutdown,
summary-clear arrival, remaining endpoints, VBUS loss, read failure, unsupported
sources, power-off and PRIMASK restoration. All host tests pass at -O0/-O2
with UBSan; the new IRQ source compiles for Cortex-M3 at -O0/-Os and exports
the strong handler symbol. Public controller lifecycle wrappers, pin/NVIC
configuration and a complete firmware link still remain before board testing.

LPC1769 public EP0 lifecycle and firmware link
--------------------------------------------

`usb_lpc17xx.c` now implements the public controller lifecycle and EP0 wrappers.
The target `usb_ctrlr.h` advertises the hardware endpoint-number range;
bulk and interrupt directions are now connected as described below. ISO is unsupported. Init validates the fixed pin map and clamps IRQ
priority to the supported range. Start validates the existing USB clock using
the board oscillator descriptor, initializes EP0, binds the IRQ and sets device
interrupt masks. Connect enables SoftConnect only with VBUS present. Stop
disables IRQs, disconnects using the GPIO latch, unbinds and releases the owned
CPU-mode endpoint resources. Fault handling queues generic USB processing;
the application still uses AppRun and does not call controller operations.

This stage requires VBUS P1.30 FUNC2, DP P0.29 FUNC1, DM P0.30 FUNC1, and an
external active-low pull-up switch on SoftConnect P2.9 FUNC1. The pin order is
documented in `usb_ctrlr.h`. VBUS must be present at startup: P1.30 has no GPIO
edge interrupt and automatic reattachment is not implemented. The existing
system clock must supply a valid 48 MHz USB clock; the controller does not
retune it. Remote wake, SOF and low-power suspend are unsupported.
The default serial string comes from the ROM unique ID through shared LPC IAP.

`ARM/NXP/LPC17xx/LPC1769/exemples/UsbEp0Demo/src/usb_ep0_demo.cpp` registers a vendor interface with no data
endpoints, using the real generic USB core and a 16-entry AppEvt queue. Vendor
request 0xC0/1 returns an eight-byte identity. Its 16 MHz oscillator descriptor
and pin map are build-fixture assumptions to verify against the actual board.

Run `python3 tests/lpc/usb/link17.py --prefix arm-none-eabi-` to link both modes.
The script checks library source links, LPC vector checksum, USB IRQ vector,
archive extraction and semihosting selection. Debug uses rdimon; Release uses
nosys. Measured text/data/bss: Debug 37896/476/4060, Release 16952/88/3560 bytes.
`run17.py` also checks public lifecycle start/stop, missing VBUS, priority clamp,
pin ownership and initialization/SIE failure cleanup at -O0/-O2 with UBSan.
These are successful firmware links and host tests, not hardware enumeration.

LPC1769 ROM UID serial
---------------------

`UsbCtrlrGetSerial` follows the LPC11U35 format: four ROM UID words rendered
as 32 uppercase hexadecimal digits. It works before controller initialization,
terminates truncated output, and returns an empty string on ROM failure or an
invalid controller number. LPC1769 selects the shared IAP entry at 0x1FFF1FF1;
the shared helper masks interrupts and restores the caller's prior state.
Both library configurations now compile their existing linked `lpc_iap.c`.

The UsbEp0Demo example leaves `pSerial` null to use generic USB serial generation.
The lifecycle test links the actual shared IAP implementation, replaces only
its ROM entry, and verifies command 58, word ordering, all buffer capacities
from zero through 39, invalid arguments, ROM failure and interrupt restoration.
Serial tests run before peripheral memory is mapped. Full Debug/Release links
also require the IAP symbols. These checks do not validate ROM execution on a
physical LPC1769 board or extend support to other IAP commands.

LPC1769 importable EP0 example
-----------------------------

`UsbEp0Demo` replaces the private `firmware/ep017_firmware.cpp` link fixture.
Its IOC project follows TimerDemo, links the MCU library, and preserves Debug
semihosting and Release nosys. Oscillator and USB pin definitions are in the
example's `board.h`. The example README gives the wiring assumptions and a
PyUSB control-request check. `link17.py` now compiles the actual example and
checks its project source links and linker options. Hardware validation is
still pending; this example exercises EP0 only.

LPC1769 bulk and interrupt endpoint realization
----------------------------------------------

The private `Lpc17UsbDataRealize` helper follows UM10360 Table 185 and section
11.10.4. Interrupt endpoint numbers are 1, 4, 7, 10 and 13, with packet sizes
1..64. Bulk endpoint numbers are 2, 5, 8, 11, 14 and 15, with packet sizes
8, 16, 32 or 64. Each direction maps to its own physical endpoint; EP15 IN
uses bit 31. ISO and EP0 are excluded from this helper.

Realization requires initialized EP0, all device/endpoint/DMA interrupt enables
off, DMA off and no CPU FIFO operation. The caller must keep the device
unconfigured or disconnected for the batch. Duplicate directions and existing
ISO realization are refused. All supported bulk/interrupt directions at maximum
size, EP0 and the endpoint pointer table fit in 2576 of the 4096 EP_RAM bytes.
Only EP_RLZED is cleared; other device events remain pending. A bounded signed
wait latches the existing SIE fault on timeout, retaining uncertain hardware
state until the controller is stopped and prepared again.

The register-model test enumerates endpoint/type/size combinations in both
directions, realizes every supported direction together, and checks active
interrupt/DMA/FIFO rejection, missing EP0, clock/power loss, invalid arguments,
stale completion flags, duplicate realization and timeout recovery. Tests pass
at -O0/-O2 with UBSan. The public lifecycle now uses this preparation step before connection and
after bus reset. Hardware enumeration and data transfers remain unverified.

LPC1769 bulk and interrupt CPU FIFO access
-----------------------------------------

Private `Lpc17UsbDataRead` and `Lpc17UsbDataWrite` transfer one packet through
USB registers, using each realized direction's indexed hardware maximum packet
size. They preserve PRIMASK, reject EP0/ISO/unrealized endpoints and DMA/FIFO
conflicts, and leave endpoint interrupts for the future data IRQ dispatcher.
The SIE full/empty flag handles bulk double buffers: OUT is readable when either
buffer is full, while IN is writable when either buffer is empty. Successful
write means buffer validation, not host acknowledgement.

OUT reads return -2 for an empty endpoint and -1 on failure. Packet data is
staged in a 64-byte local buffer and published only after successful clearing.
Insufficient caller capacity leaves the packet queued. Invalid RX data, packets
larger than MPS and hardware timeouts latch the controller fault. Both helpers
handle zero-length packets and unaligned byte buffers, clear their FIFO enable
on exit, and defer to a pending bus event. A bus event during the copy prevents
successful completion and leaves that event pending.

The register model covers endpoints 1, 2, 13 and 15, lengths 0..64, short tails,
MPS enforcement, full/empty/stalled endpoints, DMA/FIFO conflicts, invalid
arguments, RX validity, bus-event interruption and every SIE command phase's
timeout. -O0/-O2 UBSan host tests and Debug/Release EP0 example links pass.
Cortex-M3 compilation passes with -Wall -Wextra -Werror. Public data endpoints
remain disabled pending IRQ, transfer-state and lifecycle integration. No board
data-transfer test has been run.

LPC1769 data-endpoint interrupt capture
-------------------------------------

`Lpc17UsbDataCollect` captures one enabled/pending physical-endpoint mask and
its raw SIE status bytes. It shares the single-endpoint acknowledge operation
with EP0. Each clear is followed by a bounded CDFULL wait and status read before
the next clear, following UM10360 section 11.10.3.3. The unsigned mask supports
physical endpoint 31. EP0 events, masked endpoint events and device summary
flags remain untouched. Interrupts arriving outside the initial snapshot wait
for the next call; the helper never drains in a loop.

Outputs are published only after every acknowledgement succeeds. A partial
failure leaves outputs unchanged and must not be replayed. Pending bus events
abort capture so the owner can service reset/disconnect; a hardware timeout
latches the existing SIE fault. Unrealized/ISO endpoints, DMA and active FIFO
operations are rejected. This helper runs no callbacks and does not yet enable
data endpoints in the public controller or IRQ dispatcher.

The host register model checks pending/enable intersections, distinct status
bytes, bit 31, masked and EP0 preservation, reasserted/new interrupts, invalid
arguments, DMA/FIFO conflicts, bus arrival after the first acknowledgement,
and timeout at either acknowledgement of a two-endpoint batch. Tests pass at
-O0/-O2 with UBSan. Cortex-M3 warning-clean compilation and Debug/Release links
pass. Transfer-state dispatch and public lifecycle integration remain pending.

LPC1769 CPU data-transfer state
------------------------------

`usb_data_lpc17xx.c` adds caller-owned endpoint state using the existing LPC
endpoint callback type. IN stages one packet, waits for its acknowledged
interrupt, then stages the next. Completion reports the total transfer length
once. Explicit length zero sends one ZLP; exact multiples of MPS do not append
another packet. Only one IN packet is outstanding, avoiding ambiguous counts
when multiple hardware completions share one interrupt bit.

OUT dispatch requests a destination with DRDY when needed, reads one packet,
and reports XFER_CMPL. Receive only reserves the buffer and requests a later
IRQ pass; it never completes inline. This preserves the generic UsbIntrf
submission bookkeeping. A subsequent pass checks for a second buffered OUT
packet. No software data-ready flag or extra payload buffer is allocated.

The owner supplies Ready, Fault and Kick operations. Kick must schedule, not
call dispatch inline. IRQ dispatch uses endpoint generations captured before
callbacks so a stale snapshot cannot complete a replacement IN transfer.
Fault must stop hardware before Forget returns accepted buffers with CANCEL.
Forget preserves bindings, disables the endpoint and rejects reentrant work
through the owner's closing state. Halt/clear-halt handling is now connected through the public lifecycle.

`data17_test.c` mocks FIFO access and checks IN lengths 0, 1, 63, 64, 65, 128
and 129; exact packet bytes; NAK/stale-event rejection; callback resubmission;
OUT backpressure, ZLP and deferred completion; callback shutdown; and initial
or continuation failures. -O0/-O2 UBSan runs pass, as do warning-clean Cortex-M3
compilation and Debug/Release EP0 links. The library project includes the new
module. It is connected to public endpoint operations and USB_IRQHandler.

LPC1769 IRQ connection for data transfers
----------------------------------------

`Lpc17UsbIrqBindData` optionally binds a caller-owned array of 30 physical data
endpoint states after control binding and before interrupt enable. Unbind
clears both bindings. After EP0 service, USB_IRQHandler captures data generations
and SIE statuses, then calls the transfer dispatcher. Callbacks that close,
rearm, stop or unbind cannot complete another endpoint from an obsolete
snapshot. OUT rescheduling uses a later IRQ pass. Device events retain priority;
unsupported IRQs and partially failed data capture stop the controller.

Confirmed bus reset must have retired data RAM and DMA before accepted buffers
are cancelled. The handler masks data IRQs and disables control submissions
while returning buffers, then restores EP0 and emits generic RESET. A callback
that stops or unbinds aborts that sequence. Stale data endpoint interrupt flags
are acknowledged individually during hardware reset recovery; otherwise their
slow-interrupt summary could remain asserted indefinitely.

Host tests cover data dispatch through bit 31, callback rearming/unbinding,
new bus events, capture failure, invalid ownership, reset cancellation and
stale endpoint IRQ cleanup. -O0/-O2 UBSan tests and Debug/Release links pass.
The public lifecycle now binds the array and handles open/close, halt and reset.


LPC1769 public bulk and interrupt lifecycle
-----------------------------------------

Public open, close, bind, send, receive, stall and clear-stall operations now
use the private transfer state and IRQ dispatcher. Each supported direction
reserves 64 bytes per hardware packet during disconnected startup or bus reset;
logical packet sizes retain their fixed-type validation. Open and close do not
reallocate live endpoint RAM. ISO, DMA, SOF and remote wake remain unsupported.

Endpoint status changes mask the selected endpoint IRQ, flush/reset its FIFO
through SIE, and acknowledge its old IRQ before enabling dispatch. Initialization
preserves pending bus events while all interrupt sources are masked. Halting
fails an active transfer; submissions while halted remain queued until clear.
Clear-stall also resets an already-unstalled endpoint. Close/stop return accepted
buffers only after hardware retirement, including deferred cleanup after faults.
Callbacks retain the existing direct interrupt-context behavior.

Host tests cover public lifecycle transitions, repeated clear-stall with callback
rearming, failed-close ownership, bit-31 status commands, pending startup events
and bounded status timeouts. All LPC1769 USB host tests pass at -O0/-O2 with
UBSan. The EP0 example links in Debug and Release; a class/data example and
physical-board enumeration and transfers remain to be validated.


LPC1769 CDC loopback example
---------------------------

`LPC1769/exemples/UsbCdcLoopback` supplies an importable IOC project, board
configuration and AppRun-based application using the existing generic CDC
class. It explicitly initializes the 16-entry event queue and uses nonblocking
Rx/Tx in AppCheckStatus. The library project now links usb_intrf, usbd_epalloc,
usbd_cdc and usbd_cdc_desc. No shared USB implementation changes are required.

`link17.py` builds both EP0 and CDC examples in both configurations and checks
class/data symbol extraction. GNU Arm 13.2.1 CDC text/data/bss are
51956/456/5680 bytes in Debug and 23288/152/5192 in Release. Debug retains
rdimon semihosting and Release uses nosys. Only one CDC instance is covered;
the allocator does not account for LPC1769 fixed endpoint types for arbitrary
composite layouts. Physical enumeration, notifications and binary loopback
remain pending. The example README includes a packet-boundary host check.


LPC1769 shared HID loopback example
----------------------------------

`LPC1769/exemples/UsbHidLoopback` links the existing shared HID application,
which now runs its initialized 16-entry event queue through AppRun. One vendor
HID interface uses interrupt OUT/IN EP1. The MCU library project now includes
usb_int.cpp and usbd_hid.cpp. No controller or class implementation changes
are needed for this standalone example.

`link17.py` links EP0, CDC and HID firmware in Debug and Release and checks the
HID/interrupt class symbols. GNU Arm 13.2.1 HID text/data/bss are
53580/456/4552 bytes in Debug and 23880/152/4064 in Release. LPC11U35 and LPC54628 shared HID
Debug/Release links also pass after the AppRun change. These are build results;
physical enumeration and report transfers are pending. Use the existing
Python/usb_hid_loopback.py host check when a board is available.


LPC1769 shared CDC PRBS transmit example
--------------------------------------

`LPC1769/exemples/UsbCdcPrbsTx` links the shared PRBS application with the
LPC1769 USB pins and remote wake disabled. The shared application now uses
AppRun and AppCheckStatus, retaining unsent bytes across partial writes and
servicing queued events between TX attempts. Both buffered and BYTE_MODE
operation keep the existing PRBS ordering. The event queue has 16 entries.

All eight LPC1769 EP0/CDC/HID/PRBS Debug/Release links pass. PRBS text/data/bss
are 51892/460/6656 bytes in Debug and 23272/156/6168 in Release with GNU Arm
13.2.1. Shared PRBS Debug/Release links also pass on LPC11U35 and LPC54628;
LPC1769 BYTE_MODE compiles with warnings as errors. These checks do not
establish hardware throughput or PRBS integrity. The example README provides
the existing uartprbs_rx.py receiver command for board testing.


LPC1769 CDC loopback FIFO policy
-------------------------------

The local CDC loopback config now sets bBlocking=true. In CFifo this means
refusing new data when full, not waiting for hardware. The former false
setting allowed the oldest queued data to be discarded and defeated the
application's partial-write retry. A focused check using the production
CFifo filled all 1024 TX slots, verified a further write returned zero with
no drops, drained the original bytes in order and accepted data afterward.
Debug/Release CDC firmware links pass with unchanged sizes. Physical slow-host
loopback remains untested.


Shared LPC1769 CDC loopback application
--------------------------------------

The LPC1769 IOC project now links exemples/usb/usb_cdc_loopback.cpp, and the
local application copy is removed. The shared source uses AppRun with echo
work in AppCheckStatus, retaining partial writes and the no-overwrite FIFO
policy. Its existing greeting and product string are used; the LPC1769 host
check consumes the greeting before comparing binary payloads. Board settings
retain remote wake disabled. No generic USB class or controller APIs change.

LPC1769 Debug/Release CDC text/data/bss are 51956/456/5680 and
23288/152/5192 bytes. Hardware transfers remain pending.

Shared CDC Debug/Release links also pass on LPC11U35 and LPC54628 after
the AppRun conversion.


LPC1769 halt versus pending bus event
------------------------------------

A bus event can arrive after SETUP dispatch begins but before the public
halt/clear-halt call. Previously the data-ready guard silently skipped the
operation, while the generic core could record the requested halt state.
The controller now takes its existing fault/disconnect path in that case,
which disables EP0 before the request's status stage. Accepted buffers remain
owned until deferred stop retires the hardware and reports cancellation.

Lifecycle regressions cover both operations with a pending DEV_STAT event,
no endpoint-status write, disabled control/IRQ state, disconnect and exactly
one deferred cancellation. All LPC1769 host regressions pass at O0/O2 with
UBSan; warnings-as-errors Cortex-M3 compilation and all eight Debug/Release
USB firmware links pass. Hardware validation remains pending.


LPC1769 close versus pending bus event
-------------------------------------

Single-endpoint close and close-all now use the same state-change guard as
halt/clear-halt. A pending bus event on an active controller takes the fault
path instead of silently skipping close while class reconfiguration proceeds.
The fault disables control/IRQs and disconnects; deferred stop retires hardware
before cancelling accepted buffers. Closing an already-closed endpoint or an
invalid controller remains a no-op.

Lifecycle regressions cover all four operations with a pending bus event,
including retained ownership and exactly one cancellation after stop. Host
O0/O2 UBSan tests and Cortex-M3 warnings-as-errors compilation pass.
