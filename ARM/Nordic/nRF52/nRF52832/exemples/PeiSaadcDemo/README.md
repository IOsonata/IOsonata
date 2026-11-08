# PeiSaadcDemo — nRF52832 / BLYST-NANO-DK

Import the ioc Eclipse Embedded CDT managed project.

This demonstrates a complete event-to-task conversion chain without
semihosting or any event-forwarding ISR:

1. TIMER3 compare (500 ms) -> SAADC SAMPLE and LED2 (timer heartbeat).
2. SAADC END -> SAADC START (rearm the same ADC-owned EasyDMA buffer).
3. SAADC END -> LED1 (real conversion-completed heartbeat).

All these connections run through PEI/PPI hardware. The application
never writes a PPI register or SAADC task/event register.

The ADC HAL owns an internal, static RAM buffer and configures a
single-ended VDD measurement. The demo uses its existing
AdcnRF52::Init(), OpenChannel(), and new PreparePeiSampling().
The ADC conversion mode is SINGLE with bInterrupt=false, so no
SAADC ISR automatically issues SAMPLE or START.

SAADC uses 10 us acquisition time. Nordic nRF52832 erratum 150
requires more than 5 us when START is launched by PPI. The
driver rejects that unsafe combination during preparation.

## Expected behavior

- g_PeiSaadcStatus = 1: configured and both PEI event sources enabled.
  Negative values identify the failing setup step.
- g_PeiSaadcElapsedMs continually increases.
- LED2 toggles from the timer compare event.
- LED1 toggles from the SAADC END event, indicating a filled EasyDMA
  result buffer, not merely the timer compare.

The two LEDs should toggle together at the 500 ms cadence after the
first conversion. The two LED hardware paths are deliberately
independent; LED2 alone does not prove SAADC conversion.

This test is intended for standalone bare-metal nRF52832. It does
not validate SoftDevice/SDC ownership. The status variable alone
is not a conversion PASS; observe the LED1 END path physically.

## Lifecycle

Disconnect all SAADC tasks and unbind SAADC event producers before
closing an ADC channel or calling StopConversion()/Disable().
The existing ADC driver's static result buffer remains the only
EasyDMA target; no PEI code allocates buffers or changes DMA
ownership. The example runs indefinitely and has no teardown.
