# LPC11Uxx USB checks

Run `python3 tests/lpc/usb/run.py`.

- C and C++ include the production controller header through `usb/usb.h`.
- The production `usb_lpc11uxx.c` runs against mapped vendor CMSIS peripheral
  declarations at -O0 and -O2 with undefined-behavior checking. Tests cover
  invalid clocks without hardware changes, dedicated/main PLL clock paths,
  minimum AHB frequency, priority clamping, refusal of a live controller,
  endpoint-list initialization, W1C write masks and bounded EPSKIP shutdown.
- Host GNU binutils exercise the production linker script's USB SRAM placement,
  NOLOAD behavior, removal when unused, and failure on USB SRAM overflow.

The register fixture records W1C write values; ordinary mapped RAM does not
simulate W1C register semantics. A core-barrier hook can complete EPSKIP or
leave it stuck to exercise the timeout. These tests do not simulate USB bus
transactions, PLL accuracy, NVIC timing or ARM relocations.

## Current implementation

`usb_lpc11uxx.c` now supplies private hardware preparation and release helpers.
It uses the existing vendor `LPC_USB_Type` and `LPC_SYSCON_Type` declarations.
The library project already compiles this source and now links its private
header. Applications continue to use the generic USB API.

Preparation requires a working external system oscillator and clock state
maintained by the LPC11Uxx SystemInit/clock setters. It checks a nominal 48 MHz
USB clock, PLL lock/power and at least 16 MHz AHB before accessing USB registers.
The physical oscillator's USB accuracy remains a board requirement. It does
not reprogram PLLs, configure board pins or enable USB bus traffic.

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

This is still an incomplete controller port. EP0, data submission, endpoint
callbacks, USB_IRQHandler, pin setup and the public UsbCtrlr entry points remain
to be implemented. No application can enumerate with these helpers alone.
The initial packet storage does not implement ISO transfers. The header's five
endpoint numbers per direction and ISO limits describe the silicon, not
completed support or simultaneous buffer capacity.

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
