# STM32 USB device port

The existing IOsonata USB core, endpoint allocator and device classes use
`src/usb_otg_stm32.cpp`. It implements the STM32 OTG FIFO controller in
interrupt-driven device mode. `src/usb_otg_target_stm32.cpp` contains only
device-specific clock, power, PHY and vector setup. Both use ST's device
register definitions through `stm32.h`.

No application-specific STM32 USB initialization is required. Applications
use the existing `UsbInit`, `UsbEnable`, USB classes and application event queue.
No second USB stack or Cube HAL USB library is linked.

## Device coverage

| Devices | Engine | Setup and limits |
|---|---|---|
| STM32WBA65 | OTG HS, integrated HS PHY | 32 MHz HSE and USB regulator/booster; 4096-byte FIFO; 9 endpoint numbers |
| STM32F401xC/xE | OTG FS, integrated FS PHY | HSE-derived PLLQ at 48 MHz; 1280-byte FIFO; 4 endpoint numbers |
| STM32L476/L496 | OTG FS, integrated FS PHY | MSI 48 MHz calibrated by LSE; 1280-byte FIFO; 6 endpoint numbers |

The library projects for WBA65, F401xC, L476 and L496 link the shared sources.
F401xE selects the same source path but has not been compile tested separately.
Other STM32 devices, including those with the packet-memory (PMA) USB engine,
are not implemented by this change. The header rejects unlisted devices.
Supporting another OTG device requires its clock/PHY/vector setup and verified
FIFO and endpoint limits; sharing an STM32 family name is not sufficient.

Control, bulk and interrupt transfers are implemented. Isochronous functions
report failure and the capability masks are zero. Host mode, DMA, external
ULPI PHYs, high-bandwidth transactions and low-power PHY clock gating are not
implemented. Suspend/resume events are supported, but `bLowPowerSuspend` does
not turn off the clock or regulator. HS is selected on WBA65 and the controller
negotiates FS when connected through a full-speed host or hub.

FIFO transfers accept unaligned caller buffers. IN accepts at most one maximum
packet per submission, as reported by its completion callback. EP0 copies the
source into the FIFO before returning, allowing stack-local control replies.
OUT reception requires a caller buffer large enough for one maximum packet.
An accepted data buffer remains owned until completion or cancellation; the
callback runs directly in the USB interrupt. Bus events use the existing
USB/application event queue. Controller waits are bounded; a failed retirement
resets the controller before releasing buffers and queues recovery.

RX FIFO space is reserved before traffic, and each IN endpoint receives a
non-overlapping FIFO. An endpoint open can fail when a composite device needs
more FIFO space than the MCU provides, even if endpoint numbers remain free.

## Clock and pin requirements

WBA65 uses its dedicated PD6 D+ and PD7 D- PHY pads. The target code follows
the CubeWBA 1.10.0 USBX MSP power sequence without changing the system PLL.
It exports `OTG_HS_IRQHandler`, matching IOsonata's existing vector table;
the NVIC number is ST's `USB_OTG_HS_IRQn`.

F401, L476 and L496 use PA11 D-, PA12 D+ (AF10), and PA9 VBUS sense. Optional
`pIOPinMap` entries are applied after those defaults. Boards must route VBUS
to the sensing pad. These pins cannot simultaneously serve another peripheral.

F401 uses `HSE_VALUE` (25 MHz default, matching the ST family default). Set it
to the actual board crystal in the **library** build. An unused PLL is started
with a 48 MHz PLLQ output without switching SYSCLK. An already-running PLL is
checked and left unchanged; initialization fails if it does not supply 48 MHz
from HSE. HSI alone is not accepted as the USB reference.

L476/L496 require the existing system setup to have MSI at 48 MHz and LSE ready.
USB enables MSI calibration by LSE and selects MSI for CLK48. Initialization
fails for another clock arrangement rather than retuning a live system PLL.

## WBA65I-DK1 examples and hardware validation

Import `STM32WBAxx/STM32WBA65/lib/ioc` and either example's `ioc` directory:

- `STM32WBAxx/STM32WBA65/exemples/UsbCdcLoopback`
- `STM32WBAxx/STM32WBA65/exemples/UsbCdcPrbsTx`

Both link the common application in `exemples/usb`, use standard `main()` and
provide Debug/Release configurations. Build the library in the same
configuration before building the application. Use Release for rate tests.

Connect **CN5**, the board's native USB connector, to the host for these tests.
CN6/ST-LINK remains the debugger and its own UART bridge. The new USB CDC port
is separate from the previously tested ST-LINK virtual COM port.

1. Flash UsbCdcLoopback. Confirm enumeration and the CDC serial port, then open
   it and verify the banner and echoed characters.
2. Run `python3 Python/usb_cdc_loopback.py --port <new-port> --duration 60`.
   Require zero errors and zero pending bytes after draining.
3. Repeat with cable removal/reconnection and device reset. Check that the
   host can reopen the port and transfer data each time.
4. Test HS directly and FS through a full-speed hub; record the negotiated
   speed. Test host suspend/resume and endpoint halt recovery separately.
5. Flash UsbCdcPrbsTx and use `Python/uartprbs_rx.py` on the new CDC port.
   Record the selected byte/buffer mode, errors and sustained rate. The CDC
   baud setting is line-coding metadata, not the USB transfer rate.

### Hardware results - October 18, 2026

WBA65I-DK1 enumeration and CDC transfers are hardware validated:

| Test | Build | Result |
|---|---|---|
| PRBS TX | Debug | Approximately 194.5 kB/s |
| PRBS TX | Release | Approximately 649 kB/s |
| CDC loopback | Release, 60 seconds | 72094720 bytes TX and RX; 0 errors; 0 pending; PASS |

Loopback averaged 1201554.01 B/s TX and 1201451.62 B/s RX. First RX was
3 ms after start. The banner arrived after a DTR toggle. Cable reconnect,
suspend/resume, FS fallback and other classes remain untested. Negotiated
USB speed was not recorded in the host log.

F401, L476 and L496 USB hardware tests remain pending.

## Software validation (October 18, 2026)

- ARM GCC 13.2.1: controller, target initialization, generic CDC stack and both
  common application sources compiled for WBA65, F401xC, L476 and L496 at
  `-O0` and `-Os`, with the matching Cortex-M CPU and hard-float ABI.
- `tests/usb/stm32_otg_build_test.py` reproduces those object compilations.
  It does not build the complete IOsonata archive or link board firmware.
- `tests/usb/stm32_otg_test.py` runs the production transfer functions with ST
  register structures and mocked CPU intrinsics. Address/undefined-behavior
  sanitizers pass for buffer ownership, short packets, reentrant unbinding,
  cancellation, unaligned IN, ZLPs, EP0 reply copying, invalid status OUT and
  bounded flush failure. Run with `ASAN_OPTIONS=detect_leaks=0` where the host
  environment prevents LeakSanitizer process inspection.
- The maintainer built and ran WBA PRBS TX in Debug and Release, and CDC
  loopback in Release. Other family firmware links and board tests remain pending.

References used for register sequences and board wiring:

- [ST WBA USB low-level driver](https://github.com/STMicroelectronics/stm32wbaxx-hal-driver/blob/main/Src/stm32wbaxx_ll_usb.c)
- [ST F4 USB low-level driver](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_ll_usb.c)
- [CubeWBA 1.10.0 USBX MSP](https://github.com/STMicroelectronics/STM32CubeWBA/blob/v1.10.0/Projects/NUCLEO-WBA65RI/Applications/USBX/Ux_Device_CDC_ACM/Core/Src/stm32wbaxx_hal_msp.c)
- [STM32WBA65I-DK1 user manual UM3462](https://www.st.com/resource/en/user_manual/um3462-discovery-kit-with-stm32wba65ri-mcu-stmicroelectronics.pdf)
