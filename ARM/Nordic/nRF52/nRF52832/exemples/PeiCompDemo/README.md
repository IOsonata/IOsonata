# PeiCompDemo — nRF52832 / BLYST-NANO-DK

Import the ioc directory in Eclipse Embedded CDT or IOcomposer.

This demonstrates nRF52 LPCOMP/COMP event and task endpoints through the
existing AnalogComp C/C++ HAL and PEI matrix, without event-forwarding ISRs.
The comparator itself owns its input, reference and mode configuration.

Hardware setup: BLYST-NANO-DK BUT1 is P0.02, which is LPCOMP AIN0.
Its pull-up gives the initial high level, and pressing it connects the
input to GND. Internal reference is configured around VDD/2 with
hysteresis. BUT2 (P0.13) generates a dedicated GPIOTE input event.

## Expected behavior

1. After boot, g_PeiCompStatus is 1. All four LEDs are initially off.
2. Press BUT2 once: GPIOTE IN[n] -> PPI -> LPCOMP START. LED0 toggles once
   from LPCOMP READY. No CPU callback or ISR forwards this event.
3. Press and release BUT1 repeatedly. LED1 toggles on both transitions
   (CROSS), LED2 on input rising above the threshold (UP, release) and
   LED3 on input falling below the threshold (DOWN, press).
4. A change in LED0 alone does not prove threshold event routing. Check
   CROSS, UP and DOWN independently.

The example intentionally does not invoke comp.Start(). The comparator
IRQ is disabled when EvtHandler is null; PEI routes the hardware event
registers directly. No UART output or semihosting is used.

The comparator is in CONTINUOUS mode; READY_SAMPLE is configured by its
driver. External PPI SAMPLE tasks are rejected in this mode. START and
STOP task destinations are available. For SINGLE mode, SAMPLE is
available but must only be driven after START and READY.

This is an initial nRF52832 functional test. Hardware measurements,
repeated start/stop cycles, single-mode SAMPLE and teardown/reuse remain
pending. PEI connections must be disconnected before disabling or
reinitializing the comparator. Do not share the underlying COMP/LPCOMP
peripheral with a SoftDevice or another owner.
