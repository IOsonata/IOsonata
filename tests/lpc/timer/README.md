# Shared LPC timer regression

Run `python3 tests/lpc/timer/run.py` and `python3 tests/lpc/lpc546xx/run.py`
from the repository root. The first runs at O0 and O2 with undefined-behavior
sanitization. Both use host register models; they do not measure hardware timing.

The common implementation is `ARM/NXP/src/timer_lpcxx.cpp`. The LPC11Uxx, LPC17xx and
LPC546xx files supply vendor register aliases, clock setup, timer tables and IRQ
vectors. The LPC11U35, LPC1769 and LPC54628 library projects include the shared source.
The generic C and C++ Timer API is unchanged.

| Family | Timer DevNo | Hardware | Counter / prescaler width |
| --- | --- | --- | --- |
| LPC11Uxx | 0, 1 | CT16B0, CT16B1 | 16 / 16 bits |
| LPC11Uxx | 2, 3 | CT32B0, CT32B1 | 32 / 32 bits |
| LPC17xx | 0, 1, 2, 3 | TIMER0, TIMER1, TIMER2, TIMER3 | 32 / 32 bits |
| LPC546xx | 0, 1 | CTIMER3, CTIMER4 (12 MHz async clock) | 32 / 32 bits |
| LPC546xx | 2, 3, 4 | CTIMER0, CTIMER1, CTIMER2 | 32 / 32 bits |

All are reported as high-frequency devices. LPC546xx device numbering is
preserved. LPC11Uxx timers run from SystemCoreClock. At 48 MHz the minimum
16-bit timer frequency is approximately 732.422 Hz, returned as integer 732 Hz;
the 32-bit timers can reach 1 Hz. Zero selects the base clock, and other requests
select the closest available frequency. Reported integer Hz may truncate a
fractional hardware rate, as with the existing LPC546xx driver.

Each device provides a 64-bit software count and three single-shot or continuous
triggers. MR3 samples the count every half cycle to track rollover. The timer IRQ
must be serviced within half a counter cycle, even when no user trigger is active.
At 48 MHz this is about 683 us for a 16-bit timer. Select a lower frequency or a
32-bit device when interrupt latency can exceed that interval. Four ticks is the
minimum programmable trigger period, not a guarantee that a callback can execute
at that rate. Missed continuous periods are coalesced into one callback.

Only TIMER_CLKSRC_DEFAULT is supported. Tick interrupts and external capture
triggers are unsupported. Interrupt priorities select the closest implemented
NVIC level using the vendor __NVIC_PRIO_BITS: negative values become zero and
excessive values become the maximum. LPC11Uxx clamps to 0..3, LPC17xx to 0..31 and LPC546xx to
0..7. The caller configuration is unchanged. Tests cover both bounds, extreme
integer requests and priority retention across Disable/Enable.
Disable/Enable pauses and resumes the count. Reset and frequency changes restart
active trigger periods. A trigger-specific callback takes precedence over the
generic timer event callback.

The tests cover all four LPC11Uxx vectors, 16/32-bit rollover, trigger periods
longer than a counter cycle, delayed interrupt service, pause/resume, callback
reset, ownership, prescaler bounds, nearest frequency selection and long duration
arithmetic. The LPC546xx suite covers its existing numbering, clocks and triggers.

Validation performed: host tests and ARM compiler-to-assembly checks of the
shared driver, family adapters and generic C++ wrapper using real vendor headers
for LPC11U35 and LPC54628. LPC11U35 TimerDemo firmware links now pass as recorded below; LPC54628
full firmware linking and physical-board timing tests remain pending. For board validation, use `exemples/timer/timer_demo.cpp` with
TIMER_DEMO_INT_PRIO set to 3 for LPC11U35; exercise all four devices and check the
output periods with a scope, including timer pause/resume and counter rollover.

LPC17xx uses the same count, trigger, pause/resume and prescaler implementation.
Its adapter enables only the selected PCONP bit and obtains the base clock from
SystemPeriphClockGet; existing TIMER0..3 PCLKSEL dividers are preserved. Stop
and reconfigure timers after changing CPU or peripheral clock rates. The target
header accesses the vendor's named MR0..MR3 registers and omits PWMC writes
because LPC17xx timers have no such register. LPC11Uxx/LPC546xx retain their
vendor MR arrays and PWM-disable operation through the same small helpers.
No custom production register map or generic Timer API change is introduced.

The LPC17xx fixture compiles the actual adapter and shared source against the
vendor CMSIS type. It checks all four IRQ vectors, each PCLK divider, power-bit
preservation, match register selection, the reserved PWMC offset, priority
clamping, ownership, frequency bounds, three one-shot callbacks, 32-bit rollover
and pause/resume. Mapped memory models reset at NVIC synchronization and supplies
interrupt flags explicitly; it does not simulate electrical timing or preemption.
The existing LPC11Uxx/LPC546xx behavioral suites exercise continuous triggers
and delayed interrupt handling in the unchanged shared algorithm. RTC, RIT and
PWM peripherals are not included in this adapter. Full linking and board tests
remain pending; use the common timer_demo.cpp with DevNo 0..3 for bring-up.


## LPC11U35 TimerDemo ARM link

With Arm GNU tools and newlib-nano installed:

```
python3 tests/lpc/timer/link.py
python3 tests/lpc/timer/link.py --prefix /path/to/bin/arm-none-eabi- --output /tmp/lpc-timer
```

The check compiles the actual shared TimerDemo application with its LPC11U35
board header, startup, shared/family timer drivers and UART console support.
It archives library objects before linking against the production linker
script, and verifies that all required sources appear in the library project.
C++20 matches the existing TimerDemo project settings.

Debug uses -O0 and rdimon semihosting; Release uses -Os and nosys with the
board's UART console enabled. The check verifies TimerInit, all four timer
IRQ vector bindings, reset entry, checksum and the 32-byte IAP reservation.
USB objects are included in the archive to check that the timer application
retains neither the native USB driver nor the USB SRAM section.

Measured with GCC 13.2.1 and binutils 2.42:

| Build | Flash / 65536 bytes | Main RAM / 8160 bytes | USB SRAM | Linker stack allowance |
| --- | ---: | ---: | ---: | ---: |
| Debug | 24232 | 2016 | 0 | 2044 |
| Release | 22576 | 2240 | 0 | 1972 |

The stack allowance is not measured runtime usage. Retained files are under
TimerDemo/{Debug,Release} in the --output directory. This checks firmware
linkage, not execution of startup, LED periods or UART output on a board.
It also does not compile every optional library source. Physical LPC11U35
validation remains pending.
