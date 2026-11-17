# ESP32-C3 PowerOffDemo

This project targets ESP32-C3-DevKitM-1. Import `ioc` as an existing Eclipse
managed project. Build the ESP32C3 library and PowerOffDemo in the same
configuration. UART0 uses TX21/RX20 at 1,000,000 baud, 8N1, through the board's
USB-to-UART bridge. Open the terminal before resetting the board.

The default board configuration selects resumable Light-sleep:

- `WAKE_BUT_PIN` is 9, the BOOT button.
- `POWER_DEMO_RESUME_WAKE` is 1.
- `POWER_DEMO_STANDBY_RAM_TEST` is 1.

Release BOOT before reset. After each `Entering Standby` line, press and release
BOOT. The application should resume, increment the wake count and report
`SRAM sample PASS: 128 static + 128 stack bytes`. Repeat at least five times.
UART output is drained before Disable; Enable restores the UART after wake.

For reset-on-wake Deep-sleep, change `WAKE_BUT_PIN` to 3 and
`POWER_DEMO_RESUME_WAKE` to 0. Connect a momentary switch between a board header
labelled GPIO3 and GND; do not interpret 3 as the header position number.
Release the switch before reset and between wakes. After `Power off, press the
button to wake up`, connect GPIO3 to GND briefly. GPIO9 cannot wake Deep-sleep.
The application should restart and report reset reason 0x100. A start count of
1 is expected because this port exposes no retention registers. UART PowerOff
releases its pins and interrupt route before MCU Deep-sleep; startup initializes
it again. No watchdog is enabled by the demo.

C3 hardware completed six consecutive Light-sleep wakes with both SRAM samples
passing. After the wake-pad and status-clearing correction, four consecutive
Deep-sleep GPIO3 wakes restarted with reset reason 0x100. The board remained
asleep until GPIO3 was connected to GND. Repeated dots on the UART remain
unexplained; sleep current has not been measured. The SRAM samples apply
to Light-sleep only. Returning from SystemSleep can also follow its WFI fallback
and does not establish sleep current.

Build validation used RISC-V GNU 15.2.0 with source lists and settings from the
Eclipse managed projects, through command-line compiler invocations. All 204
C3 library sources compiled in Debug and Release. Both demo modes linked after
the wake-pad correction. The existing GPIO host checks passed for C2, C3, C5,
C6 and C61 at O0 and O2 with undefined-behavior sanitization. These checks do
not simulate pad power loss or replace the board retest.

| Mode | Build | text | data | bss |
| --- | --- | ---: | ---: | ---: |
| Light-sleep | Debug | 34352 | 576 | 2224 |
| Light-sleep | Release | 13641 | 480 | 1624 |
| Deep-sleep | Debug | 40168 | 616 | 4076 |
| Deep-sleep | Release | 19171 | 596 | 3808 |

For the Deep-sleep retest, leave GPIO3 disconnected from GND after reset. The
board should stay asleep until GPIO3 is connected to GND briefly, then restart
once with reset reason 0x100. Release GPIO3 before the next sleep. Repeat at
least five times. A continuous reset loop is a failure.

The committed board configuration remains Light-sleep with BOOT/GPIO9.
