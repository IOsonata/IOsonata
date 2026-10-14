# LPC11U35 I2CScan

Import `ioc` and the LPC11U35 library project (`../../lib/ioc`) into
IOcomposer/Eclipse. Build both with the same Debug or Release configuration,
then flash I2CScan. The project links the existing generic
`exemples/i2c/i2c_scan.cpp`.

Connect the target device as follows:

| Signal | LPC11U35 pin |
| --- | --- |
| SDA | P0.5 |
| SCL | P0.4 |
| UART output | P0.19 TX |
| UART input | P0.18 RX |
| Ground | Common ground |

SDA and SCL need external pull-ups to the board's I/O supply. P0.4 is also
listed as an LED pin in the Blinky example; ensure that any attached LED
circuit does not load SCL. The scanner does not configure LED outputs.

Connect a 3.3 V UART adapter RX to P0.19. Open the terminal at 115200 baud,
8-N-1, without flow control before resetting the board. The scan runs once,
then waits. It probes 7-bit addresses 0x08 through 0x77 using START,
address+write and STOP, without writing data bytes.

At the default 100 kHz, expect:

```text
I2C scan, I2C 0
Rate 100000 Hz
Device at 0xNN
1 devices found
```

The address and count depend on the connected devices. With no responding
device, expect `0 devices found` and no lockup. Reset to scan again.

To check repeated START and a one-byte register read, define both
I2C_SCAN_REG_DEVADDR and I2C_SCAN_REG in `src/board.h` or the compiler
options. Choose a readable, side-effect-free register on the attached
device. The example prints its value after scanning. It sends the register
address as one byte, so this option is for devices with an 8-bit register
address. Leave both undefined for address scanning alone.

Change I2C_SCAN_RATE to 400000 to repeat with a fast-mode-capable device.
The LPC11U35 driver supports polling master operation; interrupt and DMA
modes remain disabled in the generic scanner.

Validation: Debug and Release compiler-to-assembly checks at 100 and
400 kHz, with and without the optional register read; project links and
library/linker paths checked. Full firmware linking and board tests remain
pending.
