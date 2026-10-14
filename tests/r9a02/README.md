# R9A02G021 port checks

Emulator checks of the SAU, UARTA and IICA drivers. Each script builds a fixture with the
port's sources for RV32IMAC (no Zba, Zbb or Zbs, which unicorn does not
decode), runs it on unicorn and models the peripherals as the manual
describes them: the SAU channels, the ICU IR and DTCE flags, the CLIC input
enables, the DTC chains, Software Standby and Snooze. Interrupts are
delivered through `R9a02IrqDispatch`. Software Standby runs the firmware up
to its WFI, plays what happens while the CPU is stopped, delivers the
wake-up interrupts and resumes after the WFI. Where 12.2.10 is silent, both
readings are run: the SELSR0 event reaching ICU_SNZCANCEL in Normal mode or
not, the Snooze end also setting the IR of the SAU route or not.

```sh
python3 tests/r9a02/build_spi_slave_emu.py
python3 tests/r9a02/build_uart_snooze_emu.py
python3 tests/r9a02/build_i2c_emu.py
python3 tests/r9a02/build_spi_slave_emu.py --tool-prefix /path/to/bin/riscv-none-elf-
```

Needs xPack riscv-none-elf-gcc, unicorn 2.1 and pyelftools. The objects go to
`tests/r9a02/build`. Each script prints PASS and exits with 0, or lists the
failed checks and exits with 1.

The UART check also supports LLVM for Renesas RISC-V and Debug builds:

```sh
python3 tests/r9a02/build_uart_snooze_emu.py --llvm --tool-prefix /path/to/llvm-for-renesas-riscv/bin/
python3 tests/r9a02/build_uart_snooze_emu.py --llvm --tool-prefix /path/to/llvm-for-renesas-riscv/bin/ --debug
```

Release (`-Os`) and Debug (`-O0`) UART checks passed with LLVM 19.1.7.202501
on 2026-10-14. These check code and modeled register behavior, not timing or
electrical behavior. Hardware performance uses Release `UartPrbsTxTest`
first, then `UartLoopback` for RX, with the same configuration across MCUs.

## UART DTC transmit (2026-10-14)

The UART fixture includes `dtc_r9a02.cpp` and `uarta_r9a02.cpp`. The model
executes the DTC records through their bus access widths, requires DTCST and
the CLIC input enabled, and raises the completion event on the last transfer.
It checks SAU UART0 and both UARTA channels with polling/interrupt RX and
blocking/non-blocking FIFOs:

- Exact TX order for single bytes, blocks, FIFO wrapping and backpressure;
  private DMA storage remains intact when a non-blocking FIFO overwrites data.
- Autonomous DTC transfers without CPU handlers, followed by block chaining;
  SAU halfword writes and UARTA byte writes, including a full transmit buffer
  at DTC completion and delayed CPU handling.
- A one-byte producer bypasses descriptor setup for an idle byte and only
  enqueues while TX is busy. Queued bytes become real DTC blocks; active
  descriptors stay unchanged by subsequent enqueues. TX-ready callbacks occur
  once the queue drains in both byte-interrupt and DMA operation.
- SAU sends an isolated CPU byte with transfer-end notification. Immediate
  interrupt delivery must not retire it before following bytes accumulate.
  The queued DTC block restores buffer-empty notification.
- After DTC completion, the next block is staged while the last byte still
  occupies the transmit buffer. Reusing the freed FIFO space cannot overwrite
  that block. The following buffer-empty event arms the staged data in order.
- Short RX bursts, RX callbacks, parity-error discard, seven-bit masking and
  full duplex traffic during DTC TX.
- Disable/Enable with an active transfer, active reinitialization, rate changes
  preserving the staged tail, PowerOff cleanup, setup failure and retry,
  default FIFOs, and TX draining around SAU Snooze entry and wake-up.

RX uses the existing CPU path, as in the SAM4L port. RX DMA is not implemented.

The IRQ checks also verify pulse acknowledgement before the callback, level
acknowledgement after it, no CPU dispatch while the DTC owns the event, and
preservation of a new pending event when a handler replaces its route. These
cover the pulse dispatch shortcut used by UART without changing level-pin
acknowledgement behavior.

The unchanged `UartPrbsTxTest` source was linked with all four interrupt/DMA
selections, and `UartLoopback` was linked, in both Release and Debug using the
same LLVM toolchain. Build and emulator results do not establish hardware
throughput. The port README records the maintainer's four-mode measurements
at `bf44b8f8` and the interrupt-on/DMA-on regression at `7856bcbe`.
The subsequent single-byte transfer-end change still awaits hardware
measurements.

## SPI slave (2026-10-13)

`build_spi_slave_emu.py` builds `spi_slave_fixture.cpp` with the SPI, SAU,
DTC, ICU and pin sources, then runs `spi_slave_emu.py`, which is the
external SPI master of SPI21 and of SPI20. It checks:

- SPIInit refusals: 7-bit frames with the DTC, no interrupt, five pins, a
  chip select without IRQ line, `SPICSEL_MAN`, a rate above fMCK / 8; the
  SMR, SDR and SPS values; SetRate; a master transfer refused.
- With and without the DTC, in the four clock modes: frames of 1 to 128
  bytes against 64-byte buffers, receive and transmit buffers of other
  sizes and none, both orders of the SAU and chip select interrupts at the
  end of a frame. Received bytes, bytes sent to the master, count, -1 past
  the receive buffer.
- Frames back to back, a byte interrupt two bytes late (-1), a DTC run end
  interrupt taken after the last byte (the byte is read from SDR), the chip
  select low when the next frame is prepared (-1), Disable, Enable and
  PowerOff.
- Reception in Snooze on SPI20, with and without the DTC: SELSR0 and SNZCR,
  a rate above 0.5 Mbps refused at the enabling and by SetRate;
  ICU_SNZCANCEL on only from the standby entry to the wake-up; frames of 1,
  2, 16 and 64 bytes started in Software Standby: byte 0 received in
  Snooze, the next ones in normal operation, MISO idle in byte 0 then the
  transmit data from byte 1; a wake-up without a frame; the chip select low
  at the standby entry (not set for Snooze); a one-byte frame whose chip
  select interrupt comes first; another Snooze set up replacing it (not set
  for Snooze, kept when the slave's is turned off); Snooze turned off.

The model is the reading of the manual that the driver follows: the first
buffer empty event right after the first SDR write, BFF kept when SDR holds
transmit data, and the Snooze reception of byte 0 (23.5.7) are to confirm on
the board, the first two with the SpiMasterSlave example, the third with an
external master.

## UART reception in Snooze (2026-10-13)

`build_uart_snooze_emu.py` builds `uart_snooze_fixture.cpp` with the UART,
SAU, ICU, pin, CFIFO and UART core sources, then runs `uart_snooze_emu.py`,
which sends the characters to UART0 and collects what it transmits. The
transmit channel has its buffer and shift register: a character ends each
time the firmware polls a SAU status register outside an interrupt, and the
interrupts pending are taken at such a poll when the firmware has them
enabled. mcycle, which the firmware uses to bound its waits, advances by 100
at each read. It checks, for the HOCO at 32 and 24 MHz and 4800 and 9600 bps:

- The operation clock and STCLK of table 23.95 in normal operation and in
  Snooze (SSCm.SWC and SSEC set, the transmit channel stopped), SELSR0,
  SNZCR and SNZEDCR1, ICU_SNZCANCEL on only from the standby entry to the
  wake-up.
- The TX CFIFO sent before the channels stop, at the enabling and at the
  standby entry, none cut; the transmission going on after a rate change.
- A character received in Snooze, with BFF set by it or not: it reaches the
  RX CFIFO once with one `UART_EVT_RXDATA`, the UART is back in normal
  operation and the next characters are received; an ICU_SNZCANCEL request
  left from Normal mode dropped.
- An answer sent from `UART_EVT_RXDATA`, after the wake-up and for a
  character the standby hook takes (sent after the wake-up); Snooze turned
  off and the UART disabled from that event while the channel is set for
  Snooze; another Snooze set up replacing it (not set for Snooze, kept when
  the UART's is turned off).
- A wake-up without a character, a character received before the standby
  entry, a rate outside table 23.95 refused while Snooze is set up, Snooze
  turned off, PowerOff.
- Enabling refused for the HOCO at 48 MHz, PCLKB divided, and 115200 bps.

The Snooze model takes the character in SDR at the end of its reception and
no error flag (SSEC = 1); the error case of figure 23.41 (back to Software
Standby without the CPU) is hardware only and is to confirm on the board
with the UartSnoozeDemo example.

## I2C master (2026-10-14)

`build_i2c_emu.py` builds `i2c_fixture.cpp` with the IICA, SAU simplified
I2C, SAU, ICU, pin and DeviceIntrf sources, then runs `i2c_emu.py`. The two
buses, IICA1 (P010, P011) and IIC20 (P102, P101), are modeled bit by bit:
each line is the wired AND of the GPIO, the controller and the slaves, and
two slave devices (0x50, 0x68) decode START, STOP, address, data and ACK
from the line changes. The IICA follows 24.2 to 24.4 (STT, address and data
in IICA, events at the 9th or 8th clock with WTIM, ACK from ACKE, WREL, SPT,
IICBSY, STCF, LREL, pins low while IICE = 0), the SAU 23.8 (SO and CKO under
software control, 8 clocks and the ACK clock per SDR write, PEF on a NACK,
the ACK driven in reception while SOE = 1). It checks, polling and with
`bIntEn`:

- I2CInit, the IICA widths and IICCTL1 of 24.3.2, both IICA pins in output
  mode, the scan of the I2CScan example (only 0x50 and 0x68 answer), a
  start and stop in the read direction without data (one byte read without
  ACK first), register reads of 1, 2 and 4 bytes with a restart and the last
  byte not acknowledged, a register write, a reception without command, a
  data byte not acknowledged, an absent device, a slave holding SDA low
  cleared by Reset, a rate change to 400 kHz, PowerOff. Every operation
  leaves both lines high; with `bIntEn` each Rx, Tx and Read with data ends
  with one `DEVINTRF_EVT_COMPLETED`.
- The rules of the manual on the way: IICE set with both lines high, ACKE,
  WTIM and SPIE changed only with IICE = 0 or in a clock stretch, IICF and
  IICCTLn1 written with IICE = 0, IICA written only in a transmission
  stretch, no stop or restart after an acknowledged received byte or before
  the first one, SAU SCR and SMR written with the channel stopped, SO and CKO
  written only while software controls them.

Where the manual leaves a choice both readings are run: the IICA drives a
pin only in output mode (PDR = 1, 24.2.9 and figure 24.16) or whatever PDR;
SO keeps the last data bit after the ACK slot of a SAU transmission or goes
to 1. What the model cannot decide (the IICA pin direction, the SAU restart,
SCL as CMOS or open drain, a slave stretching SCL) is for the I2CScan
example on the board.
