# LPC546xx host regressions

Run from a Linux checkout with G++ and Python 3:

```
python3 tests/lpc/lpc546xx/run.py
```

`timer_test.cpp` compiles the production CTIMER driver
(`ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp`) against the repository's
LPC54628 device header. The CTIMER, SYSCON and ASYNC_SYSCON registers and the
NVIC are a host model: the counter jumps from match to match, the match
flags of the enabled match interrupts are set and the IRQ handler is called
like the NVIC would. It checks the 64 bit count over counter wraps, trigger
periods longer than a counter cycle, single shot and continuous triggers, the
pending interrupt for a deadline passed while arming, prescaler selection,
and the configuration checks of TimerInit.

`iopin_test.cpp` compiles the pin configuration
(`ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp`) with plain memory for IOCON.
It checks that the two type I pins, P3_23 and P3_24, get no pull resistor bits
whatever the requested resistor, keep I2CDRIVE and leave I2CSLEW in I2C mode,
and that type D pins get the requested resistor and open drain.

`i2c_spi_test.cpp` compiles the Flexcomm selection, I2C master and SPI master
drivers with the generic `device_intrf.cpp`. The I2C model is the master state
machine of the Flexcomm I2C with one simulated slave. It checks the START,
repeated START, data acknowledge and the NACK of the last byte read, NACK of
the address and of data, the SCL low and high times selected for 100 kHz,
400 kHz and 1 MHz, and the configuration checks of I2CInit (slave without
address, more than four addresses, 10 bit and SMBus rejected). The SPI model is a
MOSI to MISO loopback. It checks the dummy byte on receive, the RX FIFO not
written on transmit, the chip select held for a command and read, the frames
in flight, the clock divider, the configuration checks of SPIInit (multi
master, a slave without its SSEL pin, other PHY, frame size), and the
Flexcomm function change and lock.

`uart_dma_test.cpp` compiles the DMA channel driver
(`ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp`) and the Flexcomm USART driver with
the generic CFIFO and `device_intrf.cpp`. The DMA model reads each channel
descriptor from the table at SRAMBASE and moves one value per step while the
channel is enabled, triggered and its request is on; the request of a USART
TX channel is on while the 16 entry TX FIFO is not full. It checks the
Flexcomm request channel numbers, claim and free, the DMA0 priority taken from
the most urgent channel, the start checks of width, size and alignment, the
descriptor end addresses, the count left of a running or stopped transfer and
the dispatch of the shared vector. The UART part checks TX DMA in place from a
blocking CFIFO over CFIFO wraps, the copy from a non-blocking CFIFO and its
overflow, a DMA error, disable during a block, the polling mode where TxData
and RxData handle the block done, and the channel given back when DMA is
turned off.

`i2c_bus_test.cpp` runs the Flexcomm I2C driver as master and as slave on one
simulated bus, with the DMA model, in the four master modes (polling,
interrupt, DMA, DMA with interrupt) against the two slave modes (interrupt,
DMA), with the sequence of `exemples/i2c/i2c_master_slave.cpp`. The master
and slave pending states wait for each other the way clock stretching does;
with MSTDMA or SLVDMA set the data states are served by the DMA request of the
channel and hidden from the pending bits. Each case records the bus as text
(START, address, data, ACK/NACK, STOP) and compares it. It also checks the
TX buffer reused after an interrupt TxData returns, data NACK on the first and
on a later byte with the DeviceIntrfTx retries, a write longer than the slave
buffer, a 1500 byte write split into DMA blocks of 1024, a read past the slave
data (0xFF filler), a DMA error followed by a retry, a polling write followed
by a repeated START write, and the DMA channel given back on re-init without
DMA.

`spi_bus_test.cpp` runs the Flexcomm SPI driver as master (Flexcomm 3) and
as slave (Flexcomm 9) on one simulated bus, with the DMA model, in the four
master modes against the two slave modes, with the sequence of
`exemples/spi/spi_master_slave.cpp` in SPI modes 0 to 3: writes and reads of
1, 16 and 129 bytes and a 1 byte frame before a long one, which the slave must
start again from byte zero. The master GPIO chip select drives the slave SSEL,
its edges set SSA and SSD. A master halts while its RX FIFO is full, a slave
sends 0xFF with TXERR when its TX FIFO is empty and sets RXERR when its RX
FIFO is full. FIFOWR keeps the control bits of its last 32 bit write or of a
halfword write to its control half, as the DMA writes need, and the Flexcomm
function select resets the model. It also checks 1500 byte frames (DMA blocks
of 1024 on the master, the slave DMA then its interrupt), a frame longer than
the slave buffer (-1) or than the slave data (DummyByte), a slave RX overrun
(-1), the command phase of a read with the chip select held, the copy of an
interrupt transmit, slave Disable and Enable, a DMA error followed by a
retry, and the DMA channels given back on re-init without DMA.

`wdt_test.cpp` compiles the WWDT driver (`ARM/NXP/LPC546xx/src/wdt_lpc546xx.cpp`)
with a model of the WWDT and of the SYSCON clock and power registers. The
model needs the WWDT bus clock for every access and the watchdog oscillator
powered for counting, keeps WDEN, WDRESET, WDPROTECT and LOCK once set,
clears WDINT by writing 1, loads 0xFF for a smaller TC, checks the feed
sequence (0xAA then 0x55, interrupts masked, no access between them), resets
on a feed above WINDOW and at the timeout, and sets WDINT at WARNINT. It
checks the counts and rounding at 125 kHz and other WDTOSCCTRL settings, the
longest and shortest timeouts, the window, the reload channels, the warning
event 1023 counts before the reset, a WWDT running at Init, and the
configurations refused.

`rtc_test.cpp` compiles the RTC driver (`ARM/NXP/LPC546xx/src/rtc_lpc546xx.cpp`)
with the generic `src/miscdev/rtc.cpp`, and models the RTC on a 32768 Hz
crystal: no register write while SWRESET is set except the CTRL write that
clears it alone, COUNT written only with RTC_EN 0, the first second 1 s after
RTC_EN, the wakeup countdown every 32 crystal cycles, write 1 to clear flags,
the interrupt, and usDelay advancing the time (shim/idelay.h). It checks the
start with and without a crystal, the calendar over 1970 to 2105 against
timegm, the dates refused, the alarm fields (every minute, a weekday, day 31,
an hour today or tomorrow, the end of the counter range, a second that ends
while the match is written), the wakeup periods and limits, the backup
registers and the calendar mark, a CTRL write that leaves another flag set,
Disable, a reset of the MCU, ownership and the interrupt without owner.

`adc_test.cpp` compiles the ADC driver (`ARM/NXP/LPC546xx/src/adc_lpc546xx.cpp`)
with a model of ADC0, the SYSCON clock, power and reset registers, IOCON and
the GPIO direction. The model needs the ADC0 clock and ADC0 out of reset for
every access, ADC_ENA 10 us after ADC0 and its supplies are powered, ADC_INIT
not in the write that sets ADC_ENA, the calibration at 30 MHz or less (81 ADC
clocks, CALREQD until a calibration since the chip power up), conversions of
15 + TSAMP clocks at 60 MHz or less, sequence channels and trigger changed
only while the sequence is off, one pass per START, back to back conversions
in burst mode, and DATAVALID and OVERRUN cleared by the data register read.
A pin converts its voltage only while it is analog with a sampling time that
covers what the test asks; the temperature sensor needs TS powered 100 us
before, TSAMP 7, and gives its value from the 9th conversion of a burst. It
checks erratum ADC.1: all ADC pins digital during the set up and when ADC0
powers down. Tests: the configurations and channels refused, the pin saved
and put back, stale results, the sampling time to divider and TSAMP choice at
220 MHz, 19 MHz, 12 MHz, 3.75 MHz and 400 kHz, the temperature sensor, Disable
and Enable with ADC_INIT, Calibrate, conversions and calibrations that do not
end followed by Reset, the supplies PowerOff turns off, ownership and the
destructor.

`pwm_test.cpp` compiles the PWM driver (`ARM/NXP/LPC546xx/src/pwm_lpc546xx.cpp`)
with the pin driver of the port and a model of SCT0: the SCT0 clock and
reset, MATCH, OUTPUT and CONFIG written only while the counter is halted,
CLRCTR, match events of state 0, set and clear of the outputs with the
conflict resolution, LIMIT clearing the counter and loading the match reload
registers. The test counts the clocks each output is high over a period. It
checks the configurations refused, the closest frequency, the SCT0 output of
each pin function, outputs shared between channels, the pins at their
inactive level while stopped and after close, both polarities, duty cycles 0
to 100 percent and their rounding, the change at the next period, the first
period after Start, a frequency change in the middle of a period, Stop in the
active part of a period, a channel opened while running, eight channels at
once, Init again and the takeover by another PwmDev_t.

`nvm_test.cpp` builds the internal flash port (`ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp`)
with the generic `src/storage/nvm.cpp`, `src/device.cpp` and
`src/device_intrf.cpp` as their own objects, the flash as a host array and
the IAP ROM as a model: calls with interrupts masked, the sector prepared by
the call before for one erase or program, 256 byte programs on a page
boundary from a word aligned source outside the flash, a page programmed
once between erases. It checks the geometry, writes across page and sector
boundaries, a byte merged into its page, ones written back, a write of the
same content that programs nothing, a source in the flash, page erase,
failures of prepare, erase and program and a page that does not verify or
erase blank, the arbiter refusal, the operation counts, and the port alone:
frames, opcodes, ranges, two reads in one transfer and a write across a page
in one call.

`emc_test.cpp` compiles the EMC SDRAM set up, `SdramInit` of `coredev/sdram.h`
(`ARM/NXP/LPC546xx/src/emc_lpc546xx.cpp`), with the pin driver of the port
and a model of the EMC, its four dynamic chip selects and the SDRAM power up
sequence: the EMC clock and reset, the EMC clock divider before the EMC is
enabled, the address map and latencies of one chip select, the read strategy
and timings before the first command, NOP and precharge all 100 us apart or
more, 8 refreshes or more at the programmed rate before the mode register
command, one read with the mode value on the row address and bank 0, normal
mode, then the buffer. It checks the values of the NXP SDK board at 73.3 MHz
(address map 0x09, RAS 2, mode 0x33), the C++ class, the closest EMC clock,
the address map, the mode register and its row address bit, the base and the
size for each part of the user manual table on a 16 and a 32 bit bus and
each chip select, timings rounded up and kept within their fields, tRCD from
1 to 3 clocks, the refresh wait at a slow clock, and the configurations
refused: parts out of the table, x32 parts on a 16 bit bus, a memory above
the 128 MB of a chip select, CAS latency and tMRD out of range.

`sdif_test.cpp` compiles the SD/MMC host interface
(`ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp`) with the pin driver of the port,
links the generic SD card driver (`src/storage/sdcard_impl.cpp`) and
`src/storage/diskio_impl.cpp`, and runs them on a model of the SDIF host and a
model of an SD memory card.
The host model keeps the SDIF clock at 52 MHz or less, has a 64 word FIFO,
gives each command its time at the card clock, takes the
clock registers only through an update clock command with the clock off for
a new divider, refuses a register write during a reset or before the host
took the command before, stops the card clock while the FIFO is full on a
read, waits for data after a data command the card refused, and ends the data
path when a stop command completes. The card model answers the SD commands
in their states, resets when its supply is off 1 ms or more, and refuses
commands while it is busy. The test checks the set up of SDHC, SDSC version
2 and SDSC version 1 cards (command order, CID, CSD sizes, the clock dividers
of 400 kHz and of the transfers from the request and TRAN_SPEED), 1 and 4
data lines, no card, a card without detect pin, single and multiple sector
reads and writes with buffers not aligned, 200 sector transfers, the end of
the card, the cached DiskIO path, the FIFO full with slow register accesses,
and the failures: no response, R1 errors, response and data CRC errors, a
card that stays busy, a card that stops its data, a host that does not take
or does not end a command, an address the card refuses, programming errors in
the status or in the response of the auto stop, then PowerOff. The interface
is also driven directly: frames in pieces, frame and data in one transfer,
the response read once, the data moved once, frames refused before any
command (length, start bits, flags, slot, 4 lines on a 1 line host), the bus
width after CMD0 and after Init, rate, clock off and on, reset. 101 of 114
mutants of the interface are killed, the others are equivalent.

`i2s_test.cpp` compiles the I2S driver (`ARM/NXP/LPC546xx/src/i2s_lpc546xx.cpp`)
with the Flexcomm and DMA drivers of the port and links `src/cfifo.c`. A model
of Flexcomm 6 and 7 in their I2S function makes the frames on a bus: the
master makes one every 32 SCK from the audio PLL clock (FCLKSEL 2) and DIV, in
slave mode the test makes them, a Flexcomm takes part in the frames that
start after its MAINENABLE. Each frame a transmit Flexcomm takes its FIFO
entry or sends 0 when empty (TXI2SE0), a receive Flexcomm takes the frame of
a codec model (a counting pattern, or the frame sent in loopback) or sets
RXERR when its FIFO is full. The model checks classic I2S, 16 bit data, 32
bit frames, the configuration written with MAINENABLE off, 8 entry FIFOs
(a write to a full FIFO or a read of an empty one fails the test), the DMA
requests, the FIFO interrupts and their levels. The DMA model moves the first
value of a channel when it is started, as the controller does while the CPU
goes on. The audio PLL is a stub. The test checks the configurations
refused, the rates (the PLL asked 256 x Fs x K, MCLK, SCK dividers, the PLL
offset, rates out of the 32 bit range), transmit and receive in polling,
interrupt, CFIFO without interrupt and DMA modes, stereo and mono frames,
full duplex master and slave on Flexcomm 6 or 7, frames lost between calls
without interrupt, the overrun of the receive FIFO, a full receive CFIFO, DMA
errors on both channels, polling timeouts, Disable, Enable and Reset after a
failure, Disable with both DMA channels on their second buffer, a rate change
with frames queued, the events (TX_READY, TX_FIFO_EMPTY, RX_DATA) only when
frames move, no Flexcomm interrupt with the DMA, an object powered off that
does nothing to the one set up after, and PowerOff.

`audpll_test.c` builds `ARM/NXP/LPC546xx/src/system_LPC546xx.c` with SYSCON
and the voltage registers in mapped memory and checks the audio PLL: the
exact output for the audio rates the I2S driver asks from 12 MHz (FRO or
crystal), 16, 24 and 24.576 MHz inputs, the closest output against a search
of every N, M and P (CCO 275 to 550 MHz, Fin / N 10 kHz or more), the lowest
and highest outputs, the registers read back with the NXP decoders, the
bandwidth bits, the clock input, power down and the other SYSCON registers
unchanged, and the failures (VD3 not ready, no lock, no setting). The lock
and VD3 ready bits are written by the test.

`lcd_test.cpp` compiles the LCD controller driver
(`ARM/NXP/LPC546xx/src/lcd_lpc546xx.cpp`), set up from `DisplayCfg_t` with
the panel timings (`pTiming`) and the frame buffer (`pFrameBuffer`), with
the pin driver of the port and links the generic `src/display/display.cpp`
and the 5 x 7 system font. The LCD model takes the timing, polarity and
frame base registers only with the output off, the panel power (LCDPWR) only
after the output enable (LCDEN) and off before it, the format only with the
output off. The test checks the configurations refused (size, pixel size,
timings, no timing, frame buffer, pins) and the limits taken, the registers
of the LPCXpresso54628 panel (TIMH, TIMV, POL, CTRL RGB565 TFT, the frame
base), the closest panel clock (divider rounded, no divider at the LCD clock
or more, the largest divider), each polarity and the RGB order, the clocks,
pins, the backlight and the panel reset pin at their display control
indexes, absent or out of the pin count, Disable, Enable, PowerOff and Init
again, then in the four orientations: pixels at the corners, Fill and BitBlt
clipped at the edges and out of the screen, Scroll up and down and of the
whole screen, text from the font with a scroll at the bottom, Clear, Reset,
the line height on a small panel after an orientation change. 26 of 26
mutants of the set up killed.

`ethernet_test.cpp` compiles the Ethernet driver
(`ARM/NXP/LPC546xx/src/ethernet_lpc546xx.cpp`) with the pin driver of the
port and links the generic `src/device_intrf.cpp`. The ENET model resets its
registers on the SYSCON reset and on the DMA software reset, which ends only
with the PHY reference clock and samples ETHPHYSEL; it takes register writes
only with the ENET clocked and out of reset, an MDIO transfer with the MDC
divider of the AHB clock and never a new one before the last one ends. Its
DMA works on the descriptors of the driver with the tail pointers compared for
equality: transmit until the tail or a descriptor it does not own, with the
time stamp write back; receive into a descriptor it owns, else it stops (RBU)
until a tail write. The PHY model is a LAN8720A: BMCR reset, BMSR link latched
low, the speed in register 31. The test checks the configurations refused,
the failures of Init (no reference clock, no PHY, another PHY, an MDIO
transfer that does not end at each step, a PHY reset that does not end), the
registers of the DMA, MTL and MAC, the MDC divider from 20 to 300 MHz, the
link through EthernetPollLink (100 and 10 Mbit/s, full and half duplex, the
modes the PHY does not give, link latched low, auto negotiation not complete),
frames sent and received in order through the rings, the 4 transmit
descriptors queued and the 5th refused, the receive DMA stopped when the 4
descriptors are full and started again by the read, frames dropped (error,
context, not whole, too short or long, longer than the buffer of the call),
the interrupt events, Disable and Enable, the fatal bus error then Reset, the
calls on a device not set up while another one runs, and Close. 190 of 193
mutants killed; the three left are writes of reset values after the DMA
software reset (DMA_CH_CTRL 0, MAC_INTR_EN 0) and the transmit tail one past
the last descriptor on a wrap (the NXP SDK choice), the same as the first
descriptor when the tails are compared for equality.

`spifi_test.cpp` compiles the SPIFI driver (`ARM/NXP/LPC546xx/src/spifi_lpc546xx.cpp`)
with the pin driver of the port, a model of the SPIFI and a model of a serial
NOR flash, and links the generic `src/storage/nvm.cpp` and
`src/storage/flash.cpp`. The SPIFI model checks the clock, reset and clock
source, CTRL, ADDR and IDATA written only between commands, the CMD fields
(opcode serial, address bytes, intermediate bytes, the fields on the dual or
quad lines with CTRL DUAL), one data byte per access within DATALEN in the
direction of the command, and STAT RESET. The flash model, Winbond like,
checks each command against its address size, address lines, dummy cycles
and data lines, the Quad Enable bit for the quad commands, the write enable
latch, the busy time, and wraps a page program in its page. The test checks
the configurations refused, the pins and CTRL of SPI mode 0 and 3, the
closest SPIFI clock from the main clock, reads 0x03, 0x0B, 0x3B, 0xBB, 0x6B
and 0xEB, programs 0x02, 0x32 and 0x38, erase, 3 and 4 byte addresses, dummy
cycles and lines the SPIFI or the Phy cannot do, reads longer than DATALEN
split at the next address, a write and a read without address kept to one
command, data calls without a command, a command that does not end, a
command left running, memory mode found active, a reset that does not end,
Disable, Enable and PowerOff, then Nvm with single, dual and quad commands
(with the Quad Enable bit in status register 1 or 2) and the legacy Flash
driver in quad mode.

`crypto_test.cpp` compiles the crypto provider
(`ARM/NXP/LPC546xx/src/crypto_default_lpc546xx.cpp`) with a model of the
LPC54628 SHA block and of the ROM RNG, then runs the shared
`exemples/crypto/crypto_engine_test.cpp` on it. The software crypto sources
and micro-ecc are their own objects. The SHA model computes SHA-256: the
SHA0 clock and reset, CTRL NEW in SHA-256 mode outside a block, words written
only when WAITING is set at the first word of a block, each word 4 message
bytes loaded little endian, WAITING and DIGEST clear while a block runs and
DIGEST read only when set. It checks the known-answer test at Init (block
absent, wrong digest, failing second answer, block taken, no RNG), one-shot
hashes of 0 to 257 bytes against the software SHA-256 with the block count,
a stream on the block next to a stream in software and one-shot hashes in
between, pieces that end on a block boundary, HashInit again on the storage
of the owner, a copy of the owner refused, a dropped owner, the length limit,
argument errors, failures (WAITING or DIGEST not set, ERROR, a block slower
than the poll limit) that turn the block off and leave software in use,
HMAC on the block, the RNG and a Crypto object not initialized.

The models do not cover the prescaler counter, bus timing through the async
APB bridge, I2C electrical timing (the bus model waits where the hardware
stretches the clock) or interrupt preemption. Hardware validation is still
required.
