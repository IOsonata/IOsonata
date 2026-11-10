# nRF52 PEI validation

Run from the IOsonata repository root:

```sh
python3 tests/pei/run.py
```

The host tests compile the production `src/coredev/pei.c` C matrix
and the `ARM/Nordic/nRF52/src/pei_nrf52.cpp` PPI implementation against mock
registers. The matrix is shared by C (`PeiDev_t`) and C++ (`Pei`)
callers. They test bare-metal nRF52832 (20 configurable
channels), S132, S140, SDC, and nRF52805 (10 configurable channels). The mock
headers intentionally omit obsolete PPI_CH_NUM, PPI_GROUP_NUM, and
NRF_SOC_APP_PPI_CHANNELS_SD_ENABLED_MSK definitions. They also
compile the matrix-owning C++ `Pei` object with a C++ driver object and
exercise fan-out, fan-in, source replacement, independent disconnection,
and stale event handles.

The C tests exercise event allocation, fan-out, source rebinding,
enable/disable, release, and stale event rejection through `PeiDev_t`.
The C++ tests exercise the same matrix through the `Pei` wrapper.
`tests/pei/pei_hal_c_api.c` also syntax-checks the public Timer and
GPIO PEI entry points from C, without requiring an MCU build.

Software events (EGU) are tested with the EGU registers mapped at their
nRF52 addresses in host memory: allocation order, channels with INTEN set
by another user, the reserved instances of each build profile (two EGU on
the nRF52805 profile), firing, the task endpoint, unbind/rebind keeping the
task connections, a failed bind giving the channel back, release, deinit
and exhaustion.

Checks cover invalid event/task addresses, resource exhaustion, repeated
lifecycle calls, owned-channel checks, pre-existing channel and group use,
1-to-many and many-to-one connections, SoftDevice service errors, and
interrupt restoration. All SoftDevice operations are checked to run outside
a global interrupt-disabled section.

These are host model tests, **not** a hardware or full MCU-library build.


## Shared GPIOTE PORT event source

`IOPinEnablePortEvent()` exposes nRF52's combined
`GPIOTE.EVENTS_PORT` as a PEI producer without allocating a GPIOTE
channel, configuring pin SENSE, or enabling an interrupt. The caller
uses `IOPinSetSense()` for low/high level detection and explicitly
clears it when done. `IOPinDisablePortEvent()` unbinds only the
logical event source; other PORT interrupt and pin users are untouched.

The host suite includes C/C++ API compile checks and static
non-invasive-source checks. Run the
`ARM/Nordic/nRF52/nRF52832/exemples/PeiPortDemo/ioc` Eclipse
managed project on BLYST-NANO-DK to test the PORT and dedicated IN[n]
event paths concurrently. There is no hardware PASS yet.

## GPIOTE resource exhaustion and conventional IRQ coexistence

The nRF52832 PeiGpioteStress Eclipse managed project is at
ARM/Nordic/nRF52/nRF52832/exemples/PeiGpioteStress/ioc.
Run it on BLYST-NANO-DK, without the UART/semihosting/debugger
breakpoints. The application uses the GPIO API and PEI only, not
GPIOTE/PPI register writes.

On startup, the test tries 64 rounds of 8/8 GPIOTE IN channels,
verifies that channel 9 fails, releases a middle channel and
verifies reuse while excluding duplicate pin ownership. It then
exercises 128 enable/disable/disconnect/release cycles of GPIOTE
PEI event/task/PPI routing, interleaved with 128 ordinary GPIO IRQ
allocation/release cycles.

Inspect g_GpioteStress in the debugger after uninterrupted startup:
Status=2, Error=0, ExhaustionRounds=64, ReusedChannels=64,
RouteRounds=128, IrqRounds=128. The final live test maps BUT1
to an ordinary IRQ (incrementing IrqCount and toggling LED2)
and BUT2 to a PPI-driven LED1 toggle task. Check both buttons
independently and repeatedly; only a physical LED observation
validates the PEI route.

The new check_gpio_lifecycle_source.py in tests/pei/run.py is a
source-code check. It does not replace MCU compilation
or the board test. Results remain pending.

## SAADC PEI event and task chain

The nRF52832 PeiSaadcDemo managed project is at
ARM/Nordic/nRF52/nRF52832/exemples/PeiSaadcDemo/ioc.
It uses TIMER3 compare -> SAADC SAMPLE, SAADC END -> SAADC START,
and two separate GPIOTE LED task indicators. The SAADC sample
buffer is the existing internal AdcnRF52 EasyDMA buffer; PEI
does not configure channels or transfer data.

Preparation requires a configured single-conversion,
noninterrupt ADC and valid enabled channel(s). On nRF52832,
the driver rejects PPI START with TACQ <= 5 us, following
Nordic erratum 150; the demo uses 10 us.
A C header provides the same PEI routing operations used
by AdcnRF52 C++ wrappers, but does not turn the C++-only
ADC configuration driver into a C API.

The host suite includes C/C++ API compile-only checks and
check_saadc_pei_source.py for DMA readiness, task/event
mapping, and no ISR event forwarding. These are source checks,
not a hardware conversion PASS. On the board, LED1 toggles
only after SAADC END, while LED2 toggles directly from TIMER
compare. A running status alone does not prove conversion.

## nRF52 COMP/LPCOMP hardware PEI chain

The generic converters/analog_comp.h now defines a common C-compatible
comparator event/task PEI API with C++ methods on AnalogComp.
Supported nRF52 events: READY, DOWN, UP, CROSS; tasks: START, STOP,
and SAMPLE (SINGLE mode only). Comparator input/threshold/reference
configuration remains inside AnalogCompInit, not in PEI.
No comparator IRQ is enabled for an application with no callback.

The PeiCompDemo Eclipse project lives at
ARM/Nordic/nRF52/nRF52832/exemples/PeiCompDemo/ioc. On BLYST-NANO-DK
BUT2 (P0.13) starts LPCOMP using a GPIOTE event/PPI route.
BUT1 (P0.02, analog AIN0) crosses the VDD/2 reference: LED0
indicates READY, LED1 CROSS, LED2 UP and LED3 DOWN.

The PEI test runner includes check_comp_pei_source.py and C/C++
header compilation. The link-isolation checker also rejects comparator
PEI code in non-PEI firmware. Hardware behavior and MCU build remain
pending.

## nRF52 PWM hardware PEI sequence and task routing

PWMInit(), PWMOpenChannel() and PWMSetDutyCycle() prepare the
PWM driver-owned static RAM sequences and output pins. The C
PWMEnablePeiEvent()/PWMConnectTask() methods and C++ Pwm
wrappers expose the actual register endpoints; they do not
configure peripheral channels or DMA buffers.

PeiPwmDemo is at
ARM/Nordic/nRF52/nRF52832/exemples/PeiPwmDemo/ioc.
On BLYST-NANO-DK, BUT1 starts PWM0 sequence 0 through PPI,
and BUT2 sends PWM0 STOP through PPI. RGB red is the waveform
output; SEQSTARTED0 toggles the *separate* LED1, and STOPPED
toggles RGB blue. The example runs without semihosting or
interrupt event-forwarding. g_PeiPwmStatus=1 indicates routes
are armed but is not a hardware pass.

The existing host runner includes check_pwm_pei_source.py,
C/C++ API compilation and link-isolation cases. NEXTSTEP
is exposed only for a PWM decoder configured in NextStep mode;
the existing RefreshCount decoder never enables it. Disconnect
routes before closing PWM channels or disabling/reinitializing.
MCU compile and board validation are pending.

## nRF52 PDM and I2S PEI support (no audio hardware)

PDM and I2S now expose their existing hardware event sources and
START/STOP destinations through common C and C++ PEI APIs. PEI
does not own any sample DMA memory or interrupt refill state.

For PDM, initialize an existing PdmDev_t with a valid caller-provided
FIFO and bIntEn=true. Call PdmPreparePeiStart() *before* connecting
a hardware event to PDM_PEI_TASK_START. That call allocates the
first driver-owned FIFO sample block and programs SAMPLE.PTR/MAXCNT
without starting. Normal PDM STARTED interrupts continue allocating
the subsequent FIFO blocks. Do not start with bIntEn=false, and
do not restart before STOPPED has occurred.

For I2S, the existing I2SInit() behavior remains immediate start.
For a hardware event to trigger I2S_PEI_TASK_START: initialize with
bIntEn=true; call I2SDisable() to stop; call I2SPreparePeiStart()
to rearm the original double DMA buffers and IRQ service *without*
executing START. The API restores the DeviceIntrf enable count;
I2SDisable() still cleans up after the test. RX-only or TX-only
configurations may bind only their corresponding RXPTRUPD or
TXPTRUPD events. STOPPED remains available for both.

Run python3 tests/pei/run.py from the repository root. It includes
check_audio_pei_source.py, compile-only C/C++ API clients and
linker-map isolation tests. Static source validation is available
without a microphone or codec. No nRF52 audio board test was run;
future tests must validate sample continuity, pointer rollover and
routing teardown on a real attached microphone/I2S device.

## nRF52 SPI/SPIM/SPIS PEI

The C and C++ SPI HAL support READY on legacy SPI, STARTED/ENDRX/
ENDTX/END/STOPPED on SPIM, and ENDRX/END/ACQUIRED on SPIS.
Only SPIM START/STOP/SUSPEND/RESUME tasks are supported after
SPIPreparePeiTransfer() configures one block. Its transmit bytes
are copied to the existing driver-owned TX buffer and any RX buffer
must remain valid until STOPPED and release. Use manual chip select,
8-bit master EasyDMA and bIntEn=false. Anomaly-58 one-byte RX
cases are rejected. SPIS ACQUIRE/RELEASE are still ISR-owned.

Run python3 tests/pei/run.py to include check_spi_pei_source.py,
C/C++ header clients, and no-PEI GNU ld map cases. Hardware-free
PeiSpiDemo at ARM/Nordic/nRF52/nRF52832/exemples/PeiSpiDemo/ioc
uses BUT1 -> SPIM2 START -> END/STOP -> LED1/RGB blue over PPI,
with no external SPI slave. MCU build and board validation remain
pending.

## nRF52 TWI/TWIM/TWIS PEI

I2CDev_t and I2C expose the initialized hardware TWI, TWIM or
TWIS event registers as PEI sources. TWIM STARTTX, STARTRX,
STOP, SUSPEND and RESUME task destinations require a single
prepared transaction. Initialize a 7-bit, noninterrupt EasyDMA
master and call I2CPreparePeiTransfer() with exactly one
nonempty TX or RX block. TX is copied into the existing private
64-byte driver buffer; caller-owned RX memory must stay valid
through STOPPED and release. The generic I2C busy flag is held
while the PEI transaction is prepared. Connect an ERROR->STOP
route for error recovery; the last-byte STOP shortcut handles
successful transfer completion.

The no-target PeiTwiDemo at
ARM/Nordic/nRF52/nRF52832/exemples/PeiTwiDemo/ioc uses
BUT1 -> TWIM1 STARTTX and expects a NACK from an unpopulated
address. ERROR -> STOP + discrete LED1; STOPPED -> RGB blue.
Internal pull-ups are configured, but external 4.7K resistors
on SDA and SCL to VDD are recommended for reliable I2C timing.
Press only once per prepared transaction. The setup status
is not a hardware PASS or proof of correct I2C data.

Legacy TWI's bytewise data and TWIS PREPARERX/PREPARETX
state machine remain with their existing interrupt handlers;
only their event sources are enabled for PEI. The host runner
adds check_i2c_pei_source.py, C/C++ compile checks, and
non-PEI linker-map cases. MCU compilation, real slave
communication and cross-variant validation remain outstanding.

## nRF52 UART/UARTE PEI hardware TX chain

The existing UART C/C++ HAL now exposes mode-specific hardware
UART/UARTE event endpoints. Legacy UART offers CTS/NCTS,
RXDRDY, TXDRDY, ERROR and RXTO; UARTE adds ENDRX, ENDTX,
RXSTARTED, TXSTARTED and TXSTOPPED. CTS/NCTS require hardware
flow control to be configured on the UART instance. The UARTE
receiver is still started by the normal UARTInit and maintains
its own EasyDMA RX FIFO and IRQ/polling service.

For hardware TX, initialize UART in DMA mode, bIntMode=false,
flow control disabled, with no pending TX FIFO contents.
Call UARTPreparePeiTx() with a short block. It copies bytes
into the driver's original TX EasyDMA buffer and programs
TXD.PTR/MAXCNT without starting the transmitter. STARTTX
and STOPTX may be connected as hardware task destinations
only while that buffer is prepared. The caller must disable
and disconnect all task routes before UARTReleasePeiTx(),
which waits for TXSTOPPED before making the buffer reusable.
UART RX STARTRX/STOPRX/FLUSHRX remain driver-owned.

Hardware-free demo:
ARM/Nordic/nRF52/nRF52832/exemples/PeiUartDemo/ioc

On BLYST-NANO-DK, BUT1 starts eight-byte UARTE0 TX from
TX P0.07 at 1 Mbaud. ENDTX toggles discrete LED1 and triggers
STOPTX; TXSTOPPED toggles RGB blue. The source-level tests,
C/C++ compile-only clients and no-PEI link checks are added
to tests/pei/run.py. An on-board result and a target MCU
build are still pending.

## QDEC continuous sampling, PPI groups and excluded one-shot wrappers

QdecDev_t and Qdec expose SAMPLERDY, REPORTRDY, ACCOF, DBLRDY,
STOPPED and hardware START/STOP. QdecPreparePeiStart() enables
the configured peripheral, leaving its accumulated position and
READCLRACC with the original QDEC ISR. The BLYST-NANO-DK
PeiQdecDemo tests hardware START/STOP and sampling events,
not quadrature motion accuracy.

The generic PPI group implementation supports repeated hardware
enable/disable of owned routes with TASKS_CHG[n].EN/DIS.
PeiGroupDemo exercises hardware arm/fire/disarm. The C/C++
group APIs and mock register model are exercised by
tests/pei/run.py and check_system_pei_source.py.

WDT, one-shot TEMP, RNG and passive POWER/CLOCK PEI interfaces
have been **removed**. Their prior APIs did not offer meaningful
repetitive MCU-free operation. Ordinary Wdt, TempnRF5x,
CryptoRngNrf and platform power/clock drivers remain intact,
including the independent TEMP interrupt fix.

check_misc_pei_source.py now verifies both QDEC sampling
integration and exclusion of one-shot wrappers. C/C++ API
smoke tests and non-PEI linker map regressions no longer
reference the removed interfaces. PeiTempDemo and
PeiRngDemo managed projects have been deleted.

PDM/I2S and serial EasyDMA PEI support is provisional;
existing one-shot test projects do not establish repeated
autonomous operation. Verify an actual periodic hardware
sequence and buffer reuse before accepting those APIs.

## Non-PEI firmware link isolation

A standard Timer or GPIO application must not pull the PEI matrix or
nRF52 PPI implementation from `libIOsonata_nRF52832.a`. The TimerDemo
linker map previously demonstrated the regression:

```text
iopincfg_nrfx.o -> pei.o (PeiGetEventEndpoint)
pei.o -> pei_nrf52.o (PeiConnect)
```

Normal GPIO functions now carry *weak* references to the optional PEI
binding functions; without a PEI application they cannot extract either
archive member. Timer initialization leaves its optional PEI callbacks
unset. Only a call to `TimerEnablePeiTrigger` or
`TimerEnablePeiEvent` explicitly requests the MCU's PEI callbacks.
HF Timer and RTC compare state reuse existing `TimerTrig_t` and
per-CC bookkeeping. A compare event never causes an automatic STOP or
CLEAR task, and no private PPI route is installed for rearming.
The Timer ISR advances the selected CC for continuous operation, while
PPI routes the event directly to its task. RTC TICK/OVRFLW bookkeeping
remains in optional PEI-only sections; `--gc-sections` should discard
it from non-PEI firmware.

Clean-build the nRF52832 HAL and TimerDemo, then check the new map:

```sh
python3 tests/pei/check_no_pei_link.py \
  ARM/Nordic/nRF52/nRF52832/exemples/TimerDemo/ioc/Debug/TimerDemo.map
```

The checker fails if the archive extracts `pei.o` or
`pei_nrf52.o` (including ordinary `.a(pei.o)` linker-map entries),
if PEI-specific text remains in the executable,
or if any HF Timer or RTC PEI state arrays remain. The previous
TimerDemo link retained 248 bytes in `s_nRfxRtc*Pei` and
`s_nRfxRtcRaw`; the updated checker rejects this regression. It ignores
discarded debug and input sections. PEI-enabled PeiBlinky must still
pass after this change; link isolation does not establish hardware
operation.

## Compile against installed Nordic CMSIS headers

The mock-register tests cannot catch changes in Nordic's device headers. Use
`python3 tests/pei/check_nrf52_cmsis.py` after installing/updating IOsonata's
external SDKs. This invokes `arm-none-eabi-g++` with the installed nrfx MDK
register declarations and IOsonata's S132, S140, SDC include paths for
nRF52805, nRF52810, nRF52811, nRF52820, nRF52832, nRF52833
and nRF52840. It does not use nrfx HAL drivers,
allocate hardware, or link any firmware. Real MCU-library and on-board builds
remain separate checks.

The script uses `../external/nrfx`, `../external/nRF5_SDK` and
`../external/sdk-nrfxlib` relative to IOsonata, or corresponding environment
variables. Custom paths can also be passed with `--nrfx`, `--nrf5-sdk` and
`--nrfxlib`.

## nRF52 endpoint completion tracker

See [nRF52 PEI coverage and acceptance tracker](../../docs/architecture/pei_nrf52_coverage.md)
for the remaining peripheral families, device-specific differences,
ownership rules and acceptance criteria. Do not move PEI development
to another MCU family until nRF52 coverage is complete.

The RTC driver implements TICK/OVRFLW event sources through
`EVTEN` independently from `INTEN`. Each free compare channel may
bind its `EVENTS_COMPARE[n]` source to PEI without stopping or clearing
the counter. Continuous compare scheduling uses an RTC ISR to update
only `CC[n]` relative to the current counter; PPI still routes each
hardware event without CPU forwarding. Single-shot compare disarms
that event after the first match.

Mixed PEI/interrupt CC channels and TICK/OVRFLW sources are permitted.
`TimerDisableTrigger` unbinds only the selected PEI compare source.
The counter remains free-running for elapsed-time reads. A clean MCU
build and mixed-channel on-target test are required.

**TIMER3 configuration needs a clean MCU build and hardware validation.**
`PeiBlinky/src/board.h` selects `TimerCfg_t.DevNo = 6`, which is
**TIMER3** on nRF52832. With `Freq = 0`, IOsonata selects the 16 MHz
high-frequency timer clock. Clean-rebuild and flash BLYST-NANO-DK.
Check 500 ms LED alternation, BUT1 start, BUT2 stop, BUT1 resume,
teardown/reuse and TIMER3/GPIOTE ISR entries. Device numbering on
nRF52832: RTC0-2 are `DevNo = 0..2`, and TIMER0-4 are `DevNo = 3..7`.
This change does not validate S132/SDC coexistence; use only resources
available to the application when a Bluetooth stack is running.

## BLYST-NANO-DK hardware test

Import `ARM/Nordic/nRF52/nRF52832/exemples/PeiBlinky/ioc` into
IOcomposer. The shared `exemples/pei/pei_blinky.cpp` is MCU independent:
it binds the event and task endpoints into one `Pei` matrix and enables
hardware routing, without CPU event forwarding.

The example now uses `Timer::EnableTimerTrigger(pei, evt, ...)` to
prepare the configured Timer/RTC compare source and bind its event
to PEI. `IOPinEnableTask(pei, evt, ...)` asks the GPIO driver to prepare a
GPIOTE task and connect its endpoint. Peripheral setup stays in the
Timer and GPIO HAL drivers; PEI only programs PPI. The former
`pei_blinky_port_nrf52.c` adapter and `pei_blinky_port.h` are removed.

nRF52 uses a free-running counter. The TIMER3 ISR advances only
the matched CC for the next 500 ms event. PPI routes that event to
GPIOTE in hardware and does not use an ISR to toggle either LED.
Single-shot compares disarm only their own event; neither mode stops
or clears the counter. Mixed interrupt and PEI CC operation requires
hardware validation. `DisableTimerTrigger(trig)` unbinds only that CC.

The standalone project links the production `pei_nrf52.cpp` as an
application object compiled without `SOFTDEVICE_PRESENT`. The standard
`IOsonata_nRF52832` Debug and Release HAL libraries are built for S132;
their PEI implementation calls SoftDevice SVC functions. Without a
running SoftDevice, that path faults. This direct-source workaround is
specific to the standalone test, not to Bluetooth applications.

## Two-button hardware handoff

With `DevNo = 6`, `PeiBlinky` wires four hardware event-to-task
connections through PEI:

- TIMER3.COMPARE -> GPIOTE toggle task for LED 1 and LED 2
- BUT1 (P0.02, falling edge) -> TIMER3.START
- BUT2 (P0.13, falling edge) -> TIMER3.STOP

This TIMER3 selection has not yet received a hardware PASS. The
previously reported BLYST-NANO-DK behavior predates this configuration
change and must not be counted as a TIMER3 validation.

On reset, LED 1 starts off and LED 2 on (both are active-low).
The counter starts stopped while the low-frequency clock remains
enabled. Press BUT1 to begin alternating the LEDs at 500 ms per toggle;
press BUT2 to freeze both LEDs in their current states; press BUT1 again
to resume the counter. STOP does not reset the timer phase. Button
bounce can repeat START/STOP tasks but does not invoke any ISR-based
software forwarding or debounce logic.

The GPIO driver configures and reserves two GPIOTE input event channels
through `IOPinEnableEvent(PeiDev_t *, ...)`, implemented directly in
`iopincfg_nrfx.c`, without enabling GPIOTE interrupts. It shares the
GPIOTE allocation pool with the two output toggle tasks and conventional
GPIO interrupts. `IOPinEnableTask(PeiDev_t *, ...)` directly prepares
and connects GPIO tasks through the same C driver. The Timer driver
exposes its START/STOP task endpoints and
`TimerEnablePeiTrigger(TimerDev_t *, ...)` in C. C++ calls delegate to
these APIs. PEI configures only the four relationships. No application register
access and no example-specific port adapter are required.

Hardware observation (November 7, 2026): the device tester reports
that the updated two-button, two-LED `PeiBlinky` works on the
BLYST-NANO-DK (nRF52832): BUT1 starts alternating the LEDs, BUT2
stops/freezes them, and BUT1 resumes the timer. This is a **functional
hardware PASS** for three event sources and four event-to-task links,
including compare fan-out to both LEDs. That earlier hardware observation
does not validate the new TIMER3 selection. The 500 ms event period is
the configured value, not a separately measured timing result. A debugger
trace confirming expected TIMER3 compare ISR entries and no event-forwarding GPIOTE ISR is still pending.

Hardware observation (November 7, 2026): following the PEI C++
implementation rename, Eclipse project-link corrections and Timer/GPIO HAL
integration, the device tester reports that the nRF52832 HAL and
`PeiBlinky` compiled and the BLYST-NANO-DK LED flashes at approximately
1 Hz. This is a **functional hardware PASS** for the earlier one-LED
shared C++ example: Timer compare -> PPI -> GPIOTE task. It does not
establish a measured frequency, independently confirm that no
Timer/GPIOTE ISR was entered, or constitute a test of the later button-controlled variant, which
is recorded separately above.

The older register-level `pei_timer_gpiote.c` example was also
observed blinking at approximately 1 Hz before the HAL integration.
The debugger IRQ trace, route disable/re-enable and release checks, and
Bluetooth S132/SDC coexistence tests remain pending.

## nRF5340 DPPI host model

Run `python3 tests/pei/run_nrf53.py` from the repository root. This
compiles the real `ARM/Nordic/nRF53/src/pei_nrf53.c` against simulated
DPPIC registers in application secure, application nonsecure, and
network-core builds. It checks event publisher sharing, independent
subscriber enable/disable, reserved channels/groups, duplicate task
rejection, foreign endpoint protection, exhaustion, stale copied handles,
and failed configurations.

This is not a firmware build. Nordic MDK header compilation and on-board
testing are separate acceptance checks.

The shared example uses no target-specific registers or platform
adapter. Timer and GPIO have the endpoint preparation APIs on nRF52.
The nRF53 DPPI implementation and other MCU-specific peripheral drivers are
separate follow-up integrations.
