# LPCXpresso54628 internal flash Nvm demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/storage/nvm_demo.cpp` on the internal flash port
(`ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp`, `NvmMcuIntrf`) under the generic `Nvm`
driver. Debug uses semihosting. Release uses LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1.

The flash is erased and programmed by the IAP ROM in 256 byte pages. The
port reports a 256 byte erase unit and a 1 byte write granularity: a write
reads its page, merges the new bytes, erases and programs the page. The demo
uses the last three pages of the linker-reserved NVM1 region
(0x7FD00 to 0x7FFFF on the 512 KB part). The linker shortens the executable
FLASH allocation to 0x7F000, preventing the firmware from entering this area.

The demo reads the previous boot stamp, erases the test region, writes a
128-byte pattern across a page boundary, and reads it back. It then writes
and verifies the next boot stamp. Power-cycle the board and check that the
boot count increases.

Previous Debug and Release builds and the host model passed before the
0.13 reserved-region configuration change. Rebuild and rerun
`tests/lpc/lpc546xx/nvm_test.cpp` against the updated API. No hardware
result yet.
