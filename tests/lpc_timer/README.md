# Shared LPC timer regression

Run `python3 tests/lpc_timer/run.py` and `python3 tests/lpc546xx/run.py`
from the repository root. The first runs at O0 and O2 with undefined-behavior
sanitization. Both use host register models; they do not measure hardware timing.

The common implementation is `ARM/NXP/src/timer_lpcxx.cpp`. The LPC11Uxx and
LPC546xx files supply vendor register aliases, clock setup, timer tables and IRQ
vectors. The LPC11U35 and LPC54628 library projects include the shared source.
The generic C and C++ Timer API is unchanged.

| Family | Timer DevNo | Hardware | Counter / prescaler width |
| --- | --- | --- | --- |
| LPC11Uxx | 0, 1 | CT16B0, CT16B1 | 16 / 16 bits |
| LPC11Uxx | 2, 3 | CT32B0, CT32B1 | 32 / 32 bits |
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
triggers are unsupported. LPC11Uxx interrupt priorities are 0 through 3.
Disable/Enable pauses and resumes the count. Reset and frequency changes restart
active trigger periods. A trigger-specific callback takes precedence over the
generic timer event callback.

The tests cover all four LPC11Uxx vectors, 16/32-bit rollover, trigger periods
longer than a counter cycle, delayed interrupt service, pause/resume, callback
reset, ownership, prescaler bounds, nearest frequency selection and long duration
arithmetic. The LPC546xx suite covers its existing numbering, clocks and triggers.

Validation performed: host tests and ARM compiler-to-assembly checks of the
shared driver, family adapters and generic C++ wrapper using real vendor headers
for LPC11U35 and LPC54628. Full firmware linking and physical-board timing tests
remain pending. For board validation, use `exemples/timer/timer_demo.cpp` with
TIMER_DEMO_INT_PRIO set to 3 for LPC11U35; exercise all four devices and check the
output periods with a scope, including timer pause/resume and counter rollover.
