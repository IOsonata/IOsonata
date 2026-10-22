# STM32 watchdog

`ARM/ST/src/wdt_stm32.cpp` implements the generic watchdog API of
`include/coredev/wdt.h` (`WdtInit` and the `Wdt` class) on the independent
watchdog (IWDG, IWDG1 on the H7) of the STM32 families: F0, F4, L1, L4, L4+,
G4, H7 and WBA. One source serves them all; the differences are keyed on the
CMSIS device header. DevNo 0 only.

## Timeout

The IWDG counts down a 12 bit counter at the LSI clock divided by 4 to 256
(4 to 1024 on the WBA). The driver takes the smallest divider that holds
`msTimeout`, rounded up to the next counter tick, and `WdtDev_t::msTimeout`
gives the timeout in effect, rounded up. The LSI frequency is the typical
value of the datasheets:

| Family | LSI | Longest timeout |
|---|---|---|
| F0 | 40 kHz | 26.2 s |
| L1 | 37 kHz | 28.3 s |
| F4, L4, G4, H7 | 32 kHz | 32.8 s |
| WBA | 32 kHz | 131 s |

With the LSI predivider set (RCC LSIPREDIV on the L412, L422, L4P5 and L4Q5,
LSI1PREDIV on the WBA) the IWDG counts at 250 Hz and the timeouts are 128
times longer. Init fails on a timeout longer than the longest one. The LSI is
an RC oscillator: the timeout of a part follows its LSI frequency, see the LSI
characteristics of its datasheet.

## Options

- `msWindow`, on the F0, L4, G4, H7 and WBA (window register): a reload
  sooner than `msWindow` after the previous one resets the MCU. The window is
  rounded down to a counter tick. Init fails on a window on the F4 and L1, or
  on a window not shorter than the timeout.
- `NbChan` 1 to 32 reload channels, in software: the IWDG is reloaded when
  each channel has been reloaded since the previous IWDG reload.
- `bRunSleep` must be true: the IWDG counts in Sleep mode. In Stop and Standby
  it follows the IWDG_STOP and IWDG_STDBY option bytes where the part has
  them.
- `bRunHalt` false stops the IWDG while a debugger halts the core (DBGMCU
  freeze bit), true lets it count.
- `EvtHandler`, WBA only: early wakeup interrupt one counter tick before the
  reset, at `IntPrio`. The IWDG of the other families has no interrupt, Init
  fails with an `EvtHandler`.

## Start and running state

Init writes no IWDG register, except the early wakeup register on the WBA
when the IWDG runs. `WdtStart` starts the IWDG and writes its configuration;
the IWDG then runs until a reset. A system reset stops it, unless the IWDG_SW
option byte selects the hardware watchdog: the IWDG then runs from the reset
with its reset configuration (divider 4, reload 4095, 512 ms at 32 kHz), and
Init keeps that configuration and reports it. `WdtRunning` is true after
`WdtStart`, with the hardware watchdog option, and on the WBA whenever the
IWDG runs (SR ONF). On the other families an IWDG started by code that ran
before the application, a boot loader, is not seen: `WdtStart` writes the
configuration of Init once the register updates of that code are done.

## Libraries, examples and tests

The F030x8, F401xC, L152RE, L432, L476, L496, L4S9, G474, H753 and WBA65
library projects build the driver. Example `exemples/misc/wdt_demo.cpp`
(`WDT_DEMO_TIMEOUT_EVT` 0 in `board.h` for an IWDG without interrupt):
`STM32L4xx/STM32L476/exemples/WdtDemo` (B-L475E-IOT01A, USART1 virtual COM
port), `STM32G4xx/STM32G474/exemples/WdtDemo` (NUCLEO-G474RE, LPUART1 virtual
COM port) and `STM32WBAxx/STM32WBA65/exemples/WdtDemo` (STM32WBA65I-DK1,
USART1 virtual COM port, with the timeout event). It prints the timeout,
reloads every 500 ms ten times, then stops reloading: the IWDG resets the MCU
2 s later and the demo starts again.

`tests/stm32/run_wdt.py` builds the driver for the F030x8, F401xC, L152xE,
L412, L476, L4S9, G474, H753 and WBA65 at -O0 and -Os and runs each scenario
of `wdt_fw.cpp` in Unicorn against a model of the IWDG in time: key register,
write protection, register update flags, counter clock with the predivider,
reload, window, hardware and boot loader start, early wakeup interrupt. The
firmware arms each expected reset with its time and cause. Not run on
hardware yet.
