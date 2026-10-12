# LPC11U35 startup checksum

Run `python3 tests/lpc_startup/run.py` with host GNU `as`, `ld` and
`objcopy`. The test uses the three production LPC11U35 linker scripts,
substituting the host object format and omitting target runtime libraries.
Synthetic 32-bit vectors explicitly include Thumb bits. For each layout,
two handler placements must produce eight vector words whose sum is zero
modulo 2^32. The test also checks the stack top and application origin.

All three layouts place the stack top at 0x10001FE0, keeping SRAM0 addresses
0x10001FE0 through 0x10001FFF free for the IAP ROM (UM10462, section 20.8.9).
The normal layout therefore has 8160 bytes of SRAM0 for application use.
The stack check fails if a layout lets the application use that reserved area.

This checks linker arithmetic and memory layouts, not ARM relocations or
startup execution. The production vector source was compiled to Cortex-M0
assembly at -O0 and -O2, confirming the linker checksum is word 7 and the
reserved words remain zero. A complete ARM link and board boot test are
still required; the ARM binutils available during this change could not run.

The implementation follows the existing LPC54628 linker pattern and NXP's
[link-time checksum guidance](https://community.nxp.com/t5/LPCXpresso-IDE-FAQs/Creating-an-LPC-valid-image-checksum-directly-in-the-image/m-p/469247).
Cortex-M0 has only three handler addresses in the first seven vector words.
The normal image, DFU boot and DFU application scripts calculate the checksum
from their own stack top and resolved handlers.

Custom linker scripts must define `__valid_user_code_checksum` with the same
expression, or retain a programmer/post-build checksum patch. Its vector
reference is weak, matching LPC54628 and allowing older custom scripts to
continue linking; those scripts do not gain an automatic valid checksum.

For target validation, link an LPC11U35 example (such as TimerDemo), extract
its first eight vector words before programming, and check their 32-bit sum
is zero. Repeat after overriding NMI_Handler or HardFault_Handler. Flash
without checksum patching and verify a power-cycle boot. Repeat the checksum
check for the DFU boot and application layouts (application vectors are at
slot 0, not flash address zero).
