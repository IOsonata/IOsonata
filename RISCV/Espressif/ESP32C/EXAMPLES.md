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
| I2cMasterSlave | LP hardware master and HP slave on one board | C6 |
| I2cSlave | Interrupt-driven slave memory at address 0x22 | C3/C5/C6/C61 |
| SpiMasterSlave | SPISoft master and SPI2 slave with four jumpers | C2/C3/C5/C6/C61 |
| SpiMaster | Command and data transfers with an external SPI peer | C2/C3/C5/C6/C61 |
| TimerIrqTest | Periodic and single-shot timer interrupts | C2/C3/C5/C6/C61 |
| LedDemo | Onboard addressable RGB LED through SPI | C3/C6 |
| PulseTrain | GPIO output timing on GPIO4/5/6/7 | C3/C6 |
| CryptoTest | Implemented cryptographic operations | C3/C6 |
| PowerOffDemo | RTC/PMU sleep and wake, static/stack SRAM continuity | C3/C6 |

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
It writes and reads 16 bytes at offset 2 and prints PASS or FAIL. C3/C5/C6/C61
I2cSlave projects provide that peer. Hardware I2C slave mode is implemented
for C3/C5/C6/C61; C2 has a master-only controller. C6 I2cMasterSlave
uses its LP master and HP slave on one board. Connect GPIO6 to GPIO4 (SDA)
and GPIO7 to GPIO5 (SCL). That example enables internal pull-ups on both
controllers; start with short jumpers and add external pull-ups if needed
for bus rise time. C3 I2cSlave still needs an external master.

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

The C2/C5/C61 slave-test projects use their devkit USB-to-UART console pins:
C2 TX20/RX19, C5 TX11/RX12, and C61 TX11/RX10. These differ from the
GPIO4/5 external-adapter wiring in their earlier bring-up examples.

## PowerOffDemo (ESP32-C3/C6)

### C6 BOOT input trace

The C6 board enables `POWER_DEMO_BUTTON_TRACE`. Before disabling UART
or entering sleep, it reports GPIO9's idle level, waits for a stable
BOOT press and release, and prints the observed input levels, GPIO IRQ
count and last pending mask. The press should read 0, the release 1.
The last pending mask should include GPIO9 (0x00000200) and the IRQ
count must increase. If the input or interrupt check fails, the demo
reports the failure and does not enter sleep. While waiting for BOOT,
it repeats the live GPIO level approximately once a second.

A successful button test is followed by two different wake cycles.
The first `Entering Standby` uses CPU WFI only, with UART disabled and
the SRAM sample prepared. Press BOOT again. A successful baseline prints
"CPU WFI baseline PASS" and the normal resume/SRAM results. The next
`Entering Standby` requests PMU Light-sleep. Press BOOT a third time
to test PMU wake, independently of the CPU WFI baseline.

If the CPU WFI baseline fails, the problem is not specific to PMU sleep;
check the GPIO interrupt source and UART disable/enable sequence.
If CPU WFI passes but PMU does not return, attach the Eclipse debugger
without resetting the chip and inspect
`g_PowerDemoWakeStage`, `g_Esp32C6SleepStage`,
`g_Esp32C6SleepPmuRaw`, `g_Esp32C6SleepWakeCause`,
`g_Esp32C6SleepRejectCause`, `g_Esp32C6SleepWakeEnable`
and `g_Esp32C6SleepGpio`. The last two are snapshots taken
before requesting PMU sleep.

The demo stage is 0x01 before GPIO sampling, 0x02 on a debounced
press, 0x03 on release, 0x04 after the GPIO interrupt check,
0x05 before UART disable, 0x06 after UART disable,
0x07 before the wake operation, 0xA0 waiting in CPU WFI
(first cycle), 0xA1 after the GPIO9 WFI event, 0x08 after the wake
operation returns, and 0x09 after UART re-enable. The C6 driver stage
is 0x10
at entry, 0x11 after wake-pin preparation, 0x12 after register
save, 0x13 after PMU configuration, 0x14/0x15 during
CPU clock switching, 0x16 at sleep request, 0x17 after
PMU wake/reject/timeout, 0x18 after clock restoration and
0x19 after register and wake-pin restoration. Stages 0x1D
and 0x1E indicate PMU configuration or wake-pin preparation
failure. The raw PMU status distinguishes wake (bit 31)
from rejection (bit 30).

The shared `exemples/misc/power_off_demo.cpp` provides both wake-resume and
reset-on-wake demonstration paths. The default C3/C6 board configurations
use UART0 at 1 Mbaud, GPIO9 BOOT and `POWER_DEMO_RESUME_WAKE=1`.
They drain/disable the UART before `SystemSleep()` and check 128 bytes
each of static and stack storage after wake. RTC/PMU Light-sleep is
requested when GPIO9 is configured and idle; CPU WFI is a fallback if
the controller rejects sleep or its preparation fails.

For Deep-sleep, select a GPIO available on the board in the dedicated
wake range (C3 GPIO0-5; C6 GPIO0-7), provide an external momentary
switch to ground, change `WAKE_BUT_PIN` accordingly and set
`POWER_DEMO_RESUME_WAKE=0` in that board's `src/board.h`.
The demo then calls `SystemPowerOff()`; only a reset exits it.
GPIO9 is not a valid Deep-sleep wake input. GPIO0/1 must not be
used when the 32.768 kHz crystal is selected.

On November 16, 2026, the C6 BOOT input trace passed: GPIO9 read
high when released and low when pressed; the GPIO ISR count increased
and the pending mask contained 0x00000200. PMU wake has not succeeded:
the board displayed "Entering Standby" without a subsequent resume
message, and RESET restarted the demo. This failure is the reason for
the staged CPU WFI/PMU trace. The C3 controller and C6 power/current
measurements remain untested. The new RTC/PMU source still requires
Debug/Release managed-library builds and demo link checks in a target
toolchain.
