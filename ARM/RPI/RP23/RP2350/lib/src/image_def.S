// RP2350 boot ROM PICOBIN secure Arm executable, one block, vectors at XIP_BASE.
.section .Version,"a"
.balign 4
.global RpImageDef
RpImageDef:
.word 0xffffded3
.word 0x10210142
.word 0x000001ff
.word 0
.word 0xab123579
