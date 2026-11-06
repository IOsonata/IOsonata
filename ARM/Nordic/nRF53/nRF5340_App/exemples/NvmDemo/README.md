# Thingy:53 external flash test

Build IOsonata_nRF5340_App and NvmDemo with the same configuration
in IOcomposer. Program the application-core ELF through an external SWD probe
on P8. This standalone image replaces the application firmware.

The board routes QSPI IO0 (P0.13), IO1 (P0.14), SCK (P0.17) and CS (P0.18)
to the MX25R6435F. The demo requests 8 MHz and uses READ2IO (0xBB) for reads
and single-line page program (0x02) for writes. IO2 and IO3 stay disconnected.
The driver verifies the JEDEC ID C2 28 17 before testing the flash.

In Debug, enable semihosting in the debugger and run the application. The
test output appears in the debugger console.

For the Release UART log, connect P0.04 (expansion connector P1 SDA) to the RX input of a
3 V-compatible USB-UART bridge and connect ground. Leave the bridge TX and
power pins disconnected. Open the serial port at 1000000 baud, 8N1, then
reset the board.

The demo erases and programs external flash offsets 0x7FA000 through
0x7FCFFF. Use a board where those three sectors can be overwritten.
Look for the 0xBB read and 0x02 write commands, the page-boundary readback,
and ALL PASS. To test retention, switch the Thingy off and on with the same
image. For Debug, reconnect the debugger with semihosting enabled. The next run should report the previous boot count and increment it.
