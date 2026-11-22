# nRF5340 App UART TX integrity test

This is an Eclipse CDT managed project, cloned from CorePowerTest for the
nRF5340 application core. It links the shared
`exemples/uart/uart_tx_integrity_test.cpp` and the normal IOsonata UART
library. The board pin map is shared in layout with CorePowerTest.

## Protocol

The firmware emits 2,048 consecutive 64-byte binary frames. Frame format:

- bytes 0..3: ASCII IOTX
- bytes 4..7: little-endian sequence number, starting at zero
- bytes 8..61: deterministic xorshift32 payload seeded by sequence
- bytes 62..63: little-endian CRC-16/CCITT-FALSE of the first 62 bytes

There is **no SysLog, printf, text banner, sleep, or UART flush** in the
firmware. Transmission uses `UART::Tx`, interrupt mode, DMA mode, and
blocking UART FIFO. Every call can accept fewer bytes than requested; the
test retries the unsent suffix. The firmware leaves the UART enabled once
it finishes generating frames.

## Hardware procedure

1. Import `UartTxIntegrityTest/ioc` in IOcomposer (Eclipse Embedded CDT)
   alongside the matching nRF5340 App library project.
2. Build and flash the Release or Debug configuration.
3. Start the host receiver **before resetting the board**:

   `python3 Python/uart_tx_integrity.py --port /dev/cu.usbmodem... --baud 1000000`

4. Reset the nRF5340, then capture the result. The firmware delays 1.5 s
   before sending so the receiver can attach.
5. For 115,200 baud, change the managed project's UART_BAUDRATE
   preprocessor symbol to `115200`, rebuild/flash, then run:

   `python3 Python/uart_tx_integrity.py --port /dev/cu.usbmodem... --baud 115200`

   The source default is 1,000,000 baud. The host and target must match.

6. Verify the receiver's exact frame/byte counts and first missing ranges.

The Audio DK PCA10121 is the default board. Define `NRF5340_DK_PCA10095`
in the project's managed symbols for the standard DK.

A perfect run reports 131,072 raw bytes, 2,048 unique frames, zero missing,
zero duplicate, zero out of order, zero bad candidates, and zero unframed
bytes. A mismatch cannot by itself prove whether the bytes were lost in the
UART peripheral, DK virtual COM bridge, host OS, or Python serial reader.
Repeat using an external USB-UART adapter on the UART TX pin to distinguish
the DK virtual COM transport from the MCU.

Run `python3 Python/uart_tx_integrity.py --self-test` to check the receiver's
internal parser without hardware.

**Do not mistake this test for a power/clock validation.** Once byte
integrity is established, repeat the original CorePowerTest with SysLog.
