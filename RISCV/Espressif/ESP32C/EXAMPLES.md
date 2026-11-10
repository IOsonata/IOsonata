# ESP32 C-series examples

Managed projects link the generic IOsonata example sources. Configure wiring in
each project's `src/board.h`. Build the matching library and application with
the same Debug or Release configuration. Use Release for throughput tests.

| Example | Purpose | Targets |
| --- | --- | --- |
| UartLoopback | Binary echo and full-duplex traffic | C2/C3/C5/C6/C61 |
| UartPrbsTxTest | Continuous PRBS transmission | C2/C3/C5/C6/C61 |
| UartPrbsRxTest | Check incoming PRBS and report errors | C2/C3/C5/C6/C61 |
| UartRetarget | Standard input/output through UART | C2/C3/C5/C6/C61 |
| UartSlipPrbsTx | PRBS payloads in SLIP frames | C2/C3/C5/C6/C61 |
| UartSlipPrbsRx | Decode SLIP and check PRBS continuity | C2/C3/C5/C6/C61 |
| I2cScan | Discover responding slave addresses | C2/C3/C5/C6/C61 |
| I2cMaster | Write/read verification with the generic I2C slave demo | C2/C3/C5/C6/C61 |
| SpiLoopback | Verify SPI with MOSI wired to MISO | C2/C3/C5/C6/C61 |
| I2cSlave | Interrupt-driven slave memory at address 0x22 | C3/C6 |
| SpiMasterSlave | SPISoft master and SPI2 slave with four jumpers | C3/C6 |
| SpiMaster | Command and data transfers with an external SPI peer | C2/C3/C5/C6/C61 |
| TimerIrqTest | Periodic and single-shot timer interrupts | C2/C3/C5/C6/C61 |
| LedDemo | Onboard addressable RGB LED through SPI | C3/C6 |
| PulseTrain | GPIO output timing on GPIO4/5/6/7 | C3/C6 |
| CryptoTest | Implemented cryptographic operations | C3/C6 |

UART tests use 1,000,000 baud, 8N1, no flow control. The new I2C/SPI master
examples use polling without DMA. Hardware validation is pending for the new
examples; cross-builds and host tests do not establish hardware operation.

For UartPrbsRxTest, send from the PC and display firmware diagnostic output:

```sh
python3 Python/uartprbs_tx.py --port /dev/cu.usbserial-142140 --baud 1000000
```

For UartSlipPrbsRx, add `--slip`. For UartSlipPrbsTx, use a SLIP-capable
receiver such as the generic UartSlipPrbsRx on another MCU. The raw
`uartprbs_rx.py` checker does not decode SLIP framing. Avoid connecting two
actively driven UART outputs to the same input.

I2cMaster needs a second MCU running `exemples/i2c/i2c_slave_demo.cpp` at
address 0x22. Connect SDA GPIO6, SCL GPIO7, and ground; provide external pull-ups.
It writes and reads 16 bytes at offset 2 and prints PASS or FAIL. C3/C6
I2cSlave projects provide that peer. Hardware I2C slave mode is implemented
for C3/C5/C6/C61; C2 has a master-only controller. C6 LP I2C master support
is not implemented, so this I2C test currently needs two boards.

SpiMaster uses mode 3 at 1 MHz with automatic chip select. Connect an external
SPI peer to the board header's SCK, MOSI, MISO and CS pins plus ground. It sends
100 bytes (0 through 99), then sends command byte 0 and reads 20 bytes. It prints
the response; the peer determines its meaning. Use SpiLoopback for an automatic
MOSI-to-MISO comparison.

UartRetarget prints a greeting, reads a word from standard input, and echoes it.
LedDemo enables UART progress messages for hardware diagnosis; see its README.

SpiMasterSlave uses the generic example and a software master on the same board.
Connect GPIO0 to GPIO1 (CS), GPIO2 to GPIO3 (SCK), GPIO4 to GPIO5 (MOSI), and
GPIO6 to GPIO7 (MISO). It tests all four SPI modes with 1, 16 and 64-byte
full-duplex frames, repeated frames and short chip-select frames. Only
interrupt-driven, non-DMA slave operation is selected. SPI2 slave support is
shared across C2/C3/C5/C6/C61, with a 64-byte frame limit and a required gap
between frames for interrupt handling. Use SpiLoopback for hardware-master
validation. Build results and modeled register tests are not hardware validation.
