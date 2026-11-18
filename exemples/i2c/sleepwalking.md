# SAM4L I2C address-match wake

Use the existing I2CSlaveDemo and I2CMasterDemo managed projects. No additional
master application is needed. Hardware validation is deferred.

GNU Arm GCC 15.2.1 compiled and linked both profiles in Debug and Release:
SAM4LCxC and SAM4LSxC I2CSlaveDemo, and SAMD21 I2CMasterDemo (12 links).
The SAM4L driver was rebuilt for each library configuration. Existing power
register checks also passed. These command-line builds use the Eclipse project
source lists and settings; they are not hardware results.

## Build

1. Clean rebuild the SAM4LCxC library, then
   `ARM/Microchip/SAM4L/SAM4LCxC/exemples/I2CSlaveDemo/ioc`.
2. In the slave's `src/board.h`, set `I2C_SLEEP_TEST` to `1`.
3. Build `ARM/Microchip/SAMD/SAMD21/exemples/I2CMasterDemo/ioc`, also with
   `I2C_SLEEP_TEST` set to `1` in its board.h. Build its matching library first.
4. Both Debug and Release use the UART at 1,000,000 baud in this profile.
   Run without a debugger attached when checking sleep behavior.

The SAM4LSxC I2CSlaveDemo has the same selectable profile. Its pins are an
example for the C package; verify them against the actual board. It has no
recorded hardware validation. Leave I2C_SLEEP_TEST at zero for the ordinary
memory slave and single-round master examples.

## Wiring for later hardware testing

| Signal | SAM4L8 Xplained Pro | SAM D21 Xplained Pro |
| --- | --- | --- |
| SDA | EXT1 pin 11, PA23, TWIS0 peripheral B | EXT1 pin 11, PA08, SERCOM2 peripheral D |
| SCL | EXT1 pin 12, PA24, TWIS0 peripheral B | EXT1 pin 12, PA09, SERCOM2 peripheral D |
| Ground | GND | GND |

Provide one pair of external 4.7 kohm pull-ups to 3.3 V. Power the boards
normally and share ground; do not connect their USB-powered supply outputs.
The OLED1 on SAM4L EXT2 can remain connected. Do not run a bus scanner or
another master during this test. The master must support clock stretching.

## Sequence and expected results

Start the slave first. It prints address 0x22, drains and disables its UART,
and calls SystemSleep with TWIS enabled. The driver enables TWIS clock requests
in PM.PPCR and the corresponding asynchronous wake bit in PM.AWEN. SystemSleep
therefore selects WAIT. No timer or watchdog is started by this example.

Start or reset the D21 master. Each of three rounds:

1. Read from the unmatched address 0x23 three times, one second apart.
   The master must print `Unmatched address 0x23: PASS` each time. The slave
   must produce no new output and its sleep-return counter must not advance.
2. After another second, write 16 bytes at offset 2 to address 0x22.
   The slave wakes, receives the bytes, reports completion and returns to WAIT.
3. After another second, read the same bytes using a repeated START. The master
   must print `I2C master write 16 read 16: PASS`.

Expect six sleep returns for six separated matching transactions, nine address
requests (the combined read has two address phases), zero unexpected wakes and
zero slave errors. The master can be reset to repeat the sequence without
resetting the slave. Extra sleep returns are reported rather than suppressed.
Traffic during slave logging can prevent a transaction from starting in WAIT;
keep the supplied gaps and check both consoles.

This demonstrates address filtering while the CPU sleeps. The unmatched START
revives only the required peripheral clocks; an address match wakes the CPU.
A NACK alone is not proof of SleepWalking: also check the slave counter and
matching read/write results. Measuring sleep current needs separate equipment.

The slave remains awake while a transfer is active. Its interrupt callback
installs buffers before releasing address-match clock stretching. DMA defaults
to off. Repeat with I2C_SLAVE_DMA_ENABLE set to true to exercise the existing
PDCA data path after wake; address detection itself does not require DMA.

## Driver lifecycle

Both TWIS0 and TWIS1 enable their own asynchronous wake bit during initialization
and Enable, and clear only that bit during Disable and PowerOff. Reset restores
it through the existing bus-reset disable/enable sequence. Reinitializing an
instance removes the old mode's wake request before changing its configuration.
Other peripherals' PM bits are preserved. Finish bus transfers before disabling
or reinitializing an interface. PowerOff is terminal until reinitialization;
use Disable/Enable for a temporary stop.

This addition implements the TWIS path only. ADC/PEVC and other autonomous
SleepWalking peripheral paths are not provided by this example.

Reference: SAM4L datasheet 42023H, sections 6.1.2, 10.6.2.2, 10.7.14,
10.7.17 and 28.8.9.
