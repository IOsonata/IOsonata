# LPCXpresso54628 SD card test

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/storage/sdcard_test.cpp`: the generic SD card driver
(`SDCard`) on the SDIF of the MCU as the SD/MMC host interface (`Sdmmc`,
`ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp`). `src/board.h` gives the pins and
settings of the SD card slot: SDIF pins of the NXP SDK board files (P2_3 CLK,
P2_4 CMD, P2_6 to P2_9 D0 to D3, P2_10 card detect, P2_5 card supply switch),
and SD_D4 to SD_D7 (P4_29, P4_30, P4_31, P5_0), not wired to the slot, which
the SDIF needs in their SD function (NXP SDK, after the errata). 4 data
lines, 25 MHz requested (24.4 MHz, the main clock / 9). LED D7 is on while
the card supply is on. Output on the LPC-Link2 VCOM, Flexcomm 0, 1000000 baud
8N1 in Release; the Debug build uses semihosting.

The host turns the card supply off then on. The driver identifies the card at
400 kHz (CMD0, CMD8, ACMD41, CMD2, CMD3, CMD9, CMD7), sets the 4 line bus
(ACMD6) and the 512 byte blocks of SDSC cards (CMD16), then runs the
transfers at the highest clock not above the request and the card maximum
(TRAN_SPEED). Transfers poll the FIFO, without DMA or interrupts. CMD18 and
CMD25 end with the auto stop of the host. See the
[SD card guide](../../../../../../docs/sdcard-user-guide.md).

The test prints the card type, size, clock and CID, then uses 16 sectors in
the middle of the card: it saves them, writes and reads one sector, writes 16
sectors in one transfer and reads them back in one transfer then one at a
time, transfers from buffers on odd addresses, reads the last sector and the
last two in one transfer, checks that a read past the end is refused, then
writes the saved sectors back. It ends with "ALL PASS". Use a card without
data to keep: the sectors are not put back when the card fails during the
test.

Status: Debug and Release build and link. The host interface and the driver
pass the host test in `tests/lpc/lpc546xx/sdif_test.cpp` with a model of the
SDIF host and of an SD card. No hardware result yet.
