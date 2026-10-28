# nRF54H20 project integration

These projects are incomplete ports. Both libraries build, but they are not
ready for hardware use.

| Project | Configurations | Source selection |
| --- | --- | --- |
| `nRF54H20_App/lib/ioc` | Debug, Release | Generic Bluetooth HCI host/controller APIs and USB device stack |
| `nRF54H20_Net/lib/ioc` | Debug, Release | Generic Bluetooth stack and Nordic SDC adapters; `NRFXLIB_SDC` is defined for C and C++ |

Both Arm projects target Cortex-M33 with hard float and use GNU C17 and GNU
C++23, with C++ exceptions and RTTI disabled, like the current nRF54L
libraries. The application event queue, `AppRun`, `AppWait`, and system logger
are linked into both projects.

## Build state

Both libraries build in Debug and Release against the installer externals
(nrfx 4.7.0, sdk-nrfxlib) with arm-none-eabi-gcc 15.2.1. The projects include
the nrfx v4 folders (`bsp/stable/mdk/nrf54h` and the others the nRF54L15
library uses). `lib/include` has `nrfx_config_nrf54h20_application.h` and
`nrfx_config_nrf54h20_radiocore.h`, unchanged copies of the nrfx 4.7.0
templates.

Implemented for the build:

- Startup: `ResetEntry.c`, the core vector table and `system_nrf54h.c`
  (`SystemInit`, `SystemCoreClock`) in both libraries.
- GPIO (`iopincfg_nrfx.c`, application core): ports 0, 1, 2, 6, 7 and 9, pin
  interrupts on the 8 GPIOTE130 channels through the application core
  interrupt line and port event.
- Crypto: the application and radio cores have no RNG peripheral, CRACEN or
  CryptoCell; the secure domain owns them. The libraries link the generic
  `Crypto` (`src/crypto/crypto_default.cpp`): software algorithms, `Random`
  from the software generator, `IsSecure()` false.
- Radio core: `nrf_get_mac_address` reads FICR `BLE.ADDR`. The SDC adapter
  uses software P-256.

Nothing has been run on nRF54H20 hardware.

## Remaining target integration

- A supported MCU needs startup, GPIO, UART and timer implementations. The
  libraries link the generic UART and timer classes but no nRF54H20 UART or
  timer driver.
- Pins used by a peripheral need their `CTRLSEL` routing; `IOPinConfig` does
  not set it.
- Entropy from the secure domain. The SDC entropy callback uses
  `CryptoRngNrfInstance`, which these libraries do not provide, so a Bluetooth
  application for the radio core does not link until that service is
  integrated.
- H20 has USB hardware. The application project includes the generic USB
  device classes and HCI USB transport, but an H20 `UsbCtrlr` implementation
  still needs its NRFS power/clock/VBUS integration. The LM20 implementation
  in `ARM/Nordic/nRF54/src/usb_ctrlr_nrf54.cpp` accesses VREGUSB and clocks
  directly and is deliberately not linked into either H20 project.
- The radio project still needs H20 MPSL integration. The shared
  `ARM/Nordic/src/nrf_mpsl.cpp` selects nRF54L or older Nordic interrupt
  mappings; it does not implement H20 interrupt handling and is not added to this project.
- The application project has generic HCI code but no application-to-radio
  HCI transport. Enabling Bluetooth on that core requires this integration.

The L15/LM20 S145 BM profiles must not be copied into H20 as a substitute
for these target integrations. The RISC-V H20 project is a separate unfinished
port and is not changed by this Arm project update.
