# nRF54H20 project integration

These projects are incomplete ports. Updating their source lists does not make
them ready for a target build or hardware use.

| Project | Configurations | Source selection |
| --- | --- | --- |
| `nRF54H20_App/lib/ioc` | Debug, Release | Generic Bluetooth HCI host/controller APIs and USB device stack |
| `nRF54H20_Net/lib/ioc` | Debug, Release | Generic Bluetooth stack and Nordic SDC adapters; `NRFXLIB_SDC` is defined for C and C++ |

Both Arm projects use GNU C17 and GNU C++23, with C++ exceptions and RTTI
disabled, like the current nRF54L libraries. The application event queue,
`AppRun`, `AppWait`, and system logger are linked into both projects.

## Remaining target integration

- H20 has USB hardware. The application project includes the generic USB
  device classes and HCI USB transport, but an H20 `UsbCtrlr` implementation
  still needs its NRFS power/clock/VBUS integration. The LM20 implementation
  in `ARM/Nordic/nRF54/src/usb_ctrlr_nrf54.cpp` accesses VREGUSB and clocks
  directly and is deliberately not linked into either H20 project.
- The radio project still needs H20 MPSL integration. The shared
  `ARM/Nordic/src/nrf_mpsl.cpp` selects nRF54L or older Nordic interrupt
  mappings; it does not implement H20 interrupt handling and is not added to this project.
- The application project's existing `src/system_nrf54h.c` link points to a
  file absent from the repository. Startup, target peripheral support, and
  the external H20 SDK configuration remain to be integrated and built.
- The application project has generic HCI code but no application-to-radio
  HCI transport. Enabling Bluetooth on that core requires this integration.

The L15/LM20 S145 BM profiles must not be copied into H20 as a substitute
for these target integrations. The RISC-V H20 project is a separate unfinished
port and is not changed by this Arm project update.

Validation of this update covers XML, repository source links, C/C++ preprocessor
definitions, and Debug/Release source parity. An H20 cross-build and hardware
validation are still required after the missing target code is implemented.
