# LPCXpresso54628 watchdog demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/misc/wdt_demo.cpp` on the WWDT driver
(`ARM/NXP/LPC546xx/src/wdt_lpc546xx.cpp`). Output on the LPC-Link2 VCOM,
Flexcomm 0, 1000000 baud 8N1, in Debug and Release.

The demo sets a 2000 ms timeout, reloads 10 times every 500 ms, then stops
reloading: the WWDT resets the MCU about 2 s after the last reload and the
demo starts again. The warning interrupt comes 1023 counts (about 8 ms) before
the reset.

The WWDT counts the watchdog oscillator, 500 kHz at its reset setting divided
by 4. NXP gives this oscillator +-40 percent, the reset may come between about
1.2 and 2.8 s after the last reload. Once started the WWDT runs until the next
reset; the debugger cannot stop it by a register setting.

Status: Debug and Release build and link, the driver passes the host register
model in `tests/lpc/lpc546xx/wdt_test.cpp`. No hardware result yet.
