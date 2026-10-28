# LPCXpresso54628 RTC demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/misc/rtc_demo.cpp` on the RTC driver
(`ARM/NXP/LPC546xx/include/rtc_lpc546xx.h`, class `RtcLpc546xx`). Output on
the LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1, in Debug and Release.

The RTC runs on the 32.768 kHz crystal of the board (the NXP SDK RTC example
needs no board setting). Init fails if the crystal does not oscillate within
2 s. The demo logs the clock, whether the calendar was kept from before the
reset (a reset or a debugger restart keeps the RTC running), a boot count in
backup register 0, then the date and time every second from the wakeup timer
and an alarm at second 0 of each minute. A calendar not set since power on
takes the build time of the demo.

This RTC has no sub-second counter: the milliseconds print as 000. Its wakeup
timer counts at 1.024 kHz and is restarted by its interrupt.

Status: Debug and Release build and link, the driver passes the host register
model in `tests/lpc/lpc546xx/rtc_test.cpp`. No hardware result yet.
