# LPCXpresso54628 SDRAM test

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/misc/ext_ram_test.cpp`, which sets up the SDRAM
with the generic `Sdram` (`include/coredev/sdram.h`, the EMC port in
`ARM/NXP/LPC546xx/src/emc_lpc546xx.cpp`) from the `SDRAM_` macros of
`src/board.h`: the 16 MB SDRAM of the board (MT48LC8M16A2B4-6A, 8M x 16, 4
banks, 12 rows, 9 columns, CAS latency 3) on EMC_DYCS0 at 0xA0000000, the
timings and pins of the NXP SDK board files. Output on the LPC-Link2 VCOM,
Flexcomm 0, 1000000 baud 8N1 in Release; the Debug build uses semihosting.

The EMC clock is the CPU clock divided, 73.3 MHz for the 75 MHz of the board
header. The NXP SDK runs it at half the CPU clock and asks for a delay
calibration (`EMCDLYCTRL`) above 100 MHz; the port keeps its value at
reset. See the [SDRAM guide](../../../../../../docs/sdram-user-guide.md).

The test prints the size, address and clock of the memory, then checks the
data lines (a walking one), the address lines (one word at each power of
two), the byte lanes (byte and half word writes, the DQM lines), and the
whole 16 MB written with a value made from the address and read back, then
inverted. It ends with
"ALL PASS" and the error count; the first errors print their address and
values.

Status: Debug and Release build and link. The EMC set up passes the host test
in `tests/lpc/lpc546xx/emc_test.cpp` with a model of the EMC and of the
SDRAM power up sequence. No hardware result yet.
