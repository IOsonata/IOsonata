# LPCXpresso54628 internal flash Nvm demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/storage/nvm_demo.cpp` on the internal flash port
(`ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp`, `NvmIntrf`) under the generic `Nvm`
driver. Output on the LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1, in Debug
and Release.

The flash is erased and programmed by the IAP ROM in 256 byte pages. The
port reports a 256 byte erase unit and a 1 byte write granularity: a write
reads its page, merges the new bytes, erases and programs the page. The demo
uses three pages below the top three pages of the flash (0x7FA00 to 0x7FCFF
on the 512 KB part), far above the program.

The demo prints the geometry and the region, then checks: a boot stamp
written and read back through the driver and in the memory, erase of the
scratch pages to all ones, a write across a page boundary, a word programmed
then overwritten with ones (the port reports "replaces the word, it rewrites
in place"), unaligned erase and a read past the end refused. It ends with
"ALL PASS" and the operation counts. Power cycle the board: the boot count of
the stamp goes up by one.

Status: Debug and Release build and link. The port passes the host test in
`tests/lpc/lpc546xx/nvm_test.cpp` under the generic Nvm driver, with a model
of the flash and the IAP ROM. No hardware result yet.
