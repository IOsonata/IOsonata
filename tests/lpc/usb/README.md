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
  cancellation, malformed lengths, halt/clear and failed retirement.
- A host integration fixture links the production generic USB core, UsbIntrf,
  CFifo and DeviceIntrf to the LPC C driver. It exercises SET_ADDRESS, an
  80-byte TX, a two-packet RX, FIFO backpressure and rearming after reads.
  It also unplugs during EP0/IN/OUT traffic through both GPIO processing and
  the USB device interrupt, checks state before the application detach event,
  reconnects without reinitialization, and services a latched disconnect after
  VBUS has already returned.
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
queue mocked. It includes the public UsbRemoteWakeup path. These host tests
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
handler directly; IN copies into SRAM on submission but retains logical caller
ownership until completion. Initial OUT readiness is requested from IRQ after
class setup; later completions request the next destination directly.
Closing, stopping and bus reset retire hardware before cancellation callbacks,
and block reentrant submissions during cancellation. Bindings survive reset and
Stop/Start. Halting an active transfer reports XFER_FAILED after retirement,
avoiding replay when an ACK races EPSKIP. A completion already observed before
halt is reported normally. New submissions while halted wait for clear, which
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
arm-none-eabi-gcc. No full ARM link or board USB test has been run.
