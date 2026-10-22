# STM32 realtime clock

`RtcStm32` (`ARM/ST/include/rtc_stm32.h`, `ARM/ST/src/rtc_stm32.cpp`)
implements the generic RTC device of `include/miscdev/rtc.h` on the RTC of the
STM32 families: F0, F4, L1, L4, L4+, G4, H7 and WBA. One source serves them
all; the differences are keyed on the CMSIS device header. DevNo 0, no
interface.

## Generic RTC device

`RtcDevice` (`include/miscdev/rtc.h`) derives from `Device`. An MCU RTC and an
RTC chip implement it the same way: `Init(const RtcCfg_t &Cfg, DeviceIntrf *
const pIntrf)` takes nullptr on the MCU RTC, the I2C or SPI interface of a chip
with its address in `RtcCfg_t::DevAddr`. Setting and reading the time is
required; alarms, periodic wakeup and backup registers are optional, refused by
default. Events (`RTC_EVT_xxx` bits) go to the event handler of the
configuration in interrupt context: the MCU RTC handles its interrupts, the
application calls `IntHandler` of a chip from the interrupt of the pin wired to
its interrupt output. A chip implementation keeps its interface and address
with `Interface` and `DeviceAddress` of `Device` and reaches its registers with
`Device::Read` and `Device::Write`. An implementation sets the protected
members the accessors return (clock, alarms, backup registers, wakeup period,
calendar set) and gives its events with `RtcEvt`.

The calendar is in the C runtime types. `SetTime` and `GetTime` take a
`struct tm` (`GetTime` also gives the microseconds in the second);
`SetTimeEpoch` takes a `time_t`, `GetTimeEpoch` gives a `struct timespec`,
for every implementation (`src/miscdev/rtc.cpp`). The calendar has no time
zone: the `time_t` functions treat it as the local time of the C runtime
(`localtime_r`, `mktime`), UTC unless the application sets `TZ`. With newlib
these functions link its time zone code, which allocates a copy of `TZ` when
it is set.

## RtcStm32

Calendar 2000 to 2099, 24 hours. `SetTime` computes the day of the week and
fails on a date or time out of range. `GetTime` gives `tm_wday` and `tm_yday`,
and `tm_isdst` -1. One RTC: the object of the last Init owns it, the
interrupts call it and the other objects are refused. `Enable` is true for
the owner; `Disable` and `Reset` stop the alarms and the wakeup timer, the
calendar keeps counting. The destructor of the owner disables the RTC
interrupts.

## Clock

| `ClkSrc` | Clock |
|---|---|
| `RTC_CLKSRC_DEFAULT` | LSE, LSI when the LSE does not start within 4 s |
| `RTC_CLKSRC_LFXTAL` | LSE only, Init fails without it |
| `RTC_CLKSRC_LFRC` | LSI |

The LSE runs from the crystal, or from an external oscillator (bypass) when
the board low frequency oscillator of `g_McuOsc` is `OSC_TYPE_TCXO`, at the
frequency of `g_McuOsc` (32768 Hz when `g_McuOsc` gives the RC). The
LSI frequency is the typical value of the datasheets: 40 kHz on the F0,
37 kHz on the L1, 32 kHz on the other families, divided by 128 with the LSI
predivider (L412, L422, L4P5, L4Q5, LSI1PREDIV on the WBA). The LSI is an RC
oscillator: the calendar follows the LSI frequency of the part.

An RTC found running keeps its clock and its time: clock selected, running,
and enabled (RCC RTCEN, none on the WBA). `ClkSrc` then does not change the
clock. The RTC clock selection changes only with a backup domain reset, which
clears the calendar and the backup registers: Init does it only when the
selected oscillator does not run. The other settings of the backup domain
control register are written back, the LSE started again when it ran (other
peripherals may use it); on the WBA also the LSI1, LSI2, the LSE of the
peripherals (LSESYSEN, once the LSE is ready) and the radio sleep timer
clock. The LSE clock security system is not enabled again.

The LSE runs over a system reset. The LSI does too on the WBA (LSI1, in the
backup domain); on the F0, F4, L1, L4, G4 and H7 a system reset stops the LSI
and the calendar until Init starts it again.

The L4 `SystemInit` (`system_stm32l4xx.c`) starts the LSE on a board with the
low frequency crystal, only when it is not ready, and does not reset the
backup domain: the RTC and the backup registers keep their content over a
system reset. Before 2026-10-22 it reset the backup domain at each boot on the
L4+ (L4P5, L4Q5, L4R5, L4S5, L4S9).

## Calendar

The asynchronous prescaler takes the largest division of 1 to 128 that
divides the clock and leaves at least 1 kHz, the synchronous one divides to
1 Hz. `GetTime` gives the microseconds in steps of one synchronous count:

| Clock | Prescalers | Step |
|---|---|---|
| LSE 32768 Hz | 32 x 1024 | 977 us |
| LSI 40 kHz, 37 kHz, 32 kHz | 40, 37, 32 x 1000 | 1 ms |
| LSI 250 Hz (predivider) | 1 x 250 | 4 ms |

A new RTC starts at 2000-01-01 00:00:00. On an RTC found running, Init
rewrites the prescalers when they do not give 1 Hz from the clock, converts
the 12 hour format to 24 hours, sets the day of the week from the date (the
calendar reset value has Monday for 2000-01-01), and on the WBA clears the
binary mode, which restarts the calendar. Each of these enters the
initialization mode, which restarts the second; otherwise Init writes no
calendar register. `TimeValid` is the RTC INITS flag after Init (calendar
year other than 2000) and true after `SetTime`.

The calendar is read from the counters (BYPSHAD): no wait for the shadow
registers after Stop or Standby. SSR is read before and after TR and DR, and
the read done again when a second started in between.

## Alarms

Two alarms (A and B), one on the F0. Each field of `RtcAlarm_t` is matched,
or masked with -1: second, minute, hour, and either the day of the month or
the day of the week. The event comes at the start of the matching second; the
sub-seconds are not matched. `DisableAlarm` stops an alarm.

## Wakeup timer

Not on the F030x8. The driver takes the smallest divider that holds the
period, rounded to the nearest count, and `Wakeup()` gives the period in
effect, rounded to the nearest msec:

| Clock | Period at 32768 Hz | Step |
|---|---|---|
| RTCCLK / 2, 4, 8, 16 | up to 4, 8, 16, 32 s | 61 to 488 us |
| 1 Hz | up to 65536 s | 1 s |
| 1 Hz, 65536 added | up to 131072 s | 1 s |

A period shorter than one count of RTCCLK / 2 gives that count. With the 1 Hz
clock the first period starts at the next second. `SetWakeup` fails on a
period over 131072 s, and keeps the period in effect.

## Backup registers

| Part | Registers |
|---|---|
| F030x8 | none |
| F0 with backup registers (F072) | 5 |
| F4 | 20 |
| L1, L4, L4+, G4, H7, WBA | 32 |

In TAMP on the L412, L422, G4 and WBA, in the RTC on the others. A backup
domain reset clears them.

## Events and backup domain access

With an event handler, Init enables the alarm and wakeup interrupts at
`IntPrio`; the handler runs in interrupt context with `RTC_EVT_xxx` bits.
They go through EXTI rising edge lines on the F0, F4, L1, L4, G4 and H7, and
straight to the RTC interrupt on the WBA:

| Family | Alarm line | Wakeup line | Interrupts |
|---|---|---|---|
| F0 | 17 | 20 | `RTC_IRQn` |
| F4 | 17 | 22 | `RTC_Alarm_IRQn`, `RTC_WKUP_IRQn` |
| L1, G4 | 17 | 20 | `RTC_Alarm_IRQn`, `RTC_WKUP_IRQn` |
| L4, L4+ | 18 | 20 | `RTC_Alarm_IRQn`, `RTC_WKUP_IRQn` |
| H7 | 17 | 19 | `RTC_Alarm_IRQn`, `RTC_WKUP_IRQn` |
| WBA | - | - | `RTC_IRQn` |

Init disables the alarms and the wakeup timer, and leaves the backup domain
write access enabled (PWR DBP) for the RTC and backup register writes.

## Libraries, examples and tests

The F030x8, F401xC, L152RE, L432, L476, L496, L4S9, G474, H753 and WBA65
library projects build the driver and `src/miscdev/rtc.cpp` (the F401xC
library now also builds `src/device.cpp`). Example
`exemples/misc/rtc_demo.cpp`, with `RTC_DRIVER_HEADER` and `RTC_DRIVER` from
`board.h`: `STM32L4xx/STM32L476/exemples/RtcDemo`
(B-L475E-IOT01A, USART1 virtual COM port),
`STM32G4xx/STM32G474/exemples/RtcDemo` (NUCLEO-G474RE, LPUART1 virtual COM
port) and
`STM32WBAxx/STM32WBA65/exemples/RtcDemo` (STM32WBA65I-DK1, USART1 virtual COM
port). It prints the clock in use, a boot count kept in backup register 0,
the date and time at each 1 s wakeup event with the seconds since the epoch,
and an alarm event at the start of each minute. A calendar not set takes the
build time of the example.

`tests/stm32/run_rtc.py` builds the driver, `src/miscdev/rtc.cpp`,
`src/device.cpp` and `src/device_intrf.cpp` (with `ARM/src/iatomic.c` on the
Cortex-M0) for the F030x8, F072xB, F401xC, L152xE, L412, L476, L4S9, G474, H753
and WBA65 at -O0 and -Os and runs each scenario of `rtc_fw.cpp` in Unicorn
against a model of the RTC, its clocks and the backup domain in time, with
system resets between boots. The register layouts, EXTI lines and interrupts of
each family are in the script. Scenarios: new RTC with the calendar, alarms,
wakeup timer and backup registers; no crystal; system reset; LSI over a system
reset; crystal lost; RTC on the HSE moved to the LSI or to the LSE, the LSE
stopping at the backup domain reset; RTC running from its reset values; LSE
selected with the RTC clock disabled; external oscillator at 32768, 31250 and
256000 Hz; 12 hour format; prescalers not giving 1 Hz; WBA binary mode; LSI
predivider; reads across the second, `time_t` functions, calls through
`RtcDevice`, ownership and destruction; an RTC chip implementation of
`RtcDevice` in the test on an I2C interface: interface and address, `time_t`
functions without sub-seconds, optional functions refused by default, event
from `IntHandler`. Each of 216 driver mutations fails it. Not run on hardware
yet.
