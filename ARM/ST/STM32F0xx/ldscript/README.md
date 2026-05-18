# STM32F03x linker script coverage

The uploaded gcc_stm32f030x.ld uses:

- FLASH origin 0x08000000, length 64K
- RAM origin 0x20000000, length 8K

That covers the STM32F03x 64KB flash / 8KB SRAM memory class, which maps to STM32F030x8 parts.

## Memory-class linker scripts

| File | Flash | RAM | Covers |
|---|---:|---:|---|
| gcc_stm32f03x_16k_4k.ld | 16K | 4K | STM32F030x4, STM32F031x4 |
| gcc_stm32f03x_32k_4k.ld | 32K | 4K | STM32F030x6, STM32F031x6, STM32F038x6 |
| gcc_stm32f03x_64k_8k.ld | 64K | 8K | STM32F030x8 |
| gcc_stm32f03x_256k_32k.ld | 256K | 32K | STM32F030xC |

## Family/flash-code aliases

| File | Equivalent memory class |
|---|---|
| gcc_stm32f030x4.ld | 16K / 4K |
| gcc_stm32f030x6.ld | 32K / 4K |
| gcc_stm32f030x8.ld | 64K / 8K |
| gcc_stm32f030xc.ld | 256K / 32K |
| gcc_stm32f031x4.ld | 16K / 4K |
| gcc_stm32f031x6.ld | 32K / 4K |
| gcc_stm32f038x6.ld | 32K / 4K |

## Variant mapping

### STM32F030

- STM32F030F4Px -> gcc_stm32f030x4.ld
- STM32F030C6Tx -> gcc_stm32f030x6.ld
- STM32F030K6Tx -> gcc_stm32f030x6.ld
- STM32F030C8Tx -> gcc_stm32f030x8.ld
- STM32F030R8Tx -> gcc_stm32f030x8.ld
- STM32F030CCTx -> gcc_stm32f030xc.ld
- STM32F030RCTx -> gcc_stm32f030xc.ld

### STM32F031

- STM32F031C4Tx -> gcc_stm32f031x4.ld
- STM32F031F4Px -> gcc_stm32f031x4.ld
- STM32F031G4Ux -> gcc_stm32f031x4.ld
- STM32F031K4Ux -> gcc_stm32f031x4.ld
- STM32F031C6Tx -> gcc_stm32f031x6.ld
- STM32F031E6Yx -> gcc_stm32f031x6.ld
- STM32F031F6Px -> gcc_stm32f031x6.ld
- STM32F031G6Ux -> gcc_stm32f031x6.ld
- STM32F031K6Tx -> gcc_stm32f031x6.ld
- STM32F031K6Ux -> gcc_stm32f031x6.ld

### STM32F038

- STM32F038C6Tx -> gcc_stm32f038x6.ld
- STM32F038E6Yx -> gcc_stm32f038x6.ld
- STM32F038F6Px -> gcc_stm32f038x6.ld
- STM32F038G6Ux -> gcc_stm32f038x6.ld
- STM32F038K6Ux -> gcc_stm32f038x6.ld
