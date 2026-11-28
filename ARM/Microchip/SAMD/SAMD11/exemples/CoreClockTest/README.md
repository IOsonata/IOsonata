# SAMD11D14AM CoreClockTest

Build the SAMD11 library and CoreClockTest with matching Eclipse configurations.
Output uses the EDBG UART at 1,000,000 baud.

The test checks HF and LF clock enable and disable, CPU running modes,
and peripheral clock requests, readback and restoration. The RTC runs while
the test enables and disables LF clock requests. Its counter must still
advance after the test releases its last request. TC1 at 250 kHz, with the
interrupts off, times `usDelay`, `msDelay` and `nsDelay` of 100 ms at 48 and
24 MHz, each within 2 percent.

For delay calibration during porting, set `CORE_CLOCK_TEST_DELAY_CALIBRATION=1`
in the application's C++ preprocessor symbols. The same clock tests run.
TC1 times the public delay loops at 48 and 24 MHz, the measured factors are
timed again within 2 percent before they are printed. Each
`Delay divisors` line gives the CPU frequency and the microsecond and
nanosecond divisors scaled by 1,000,000.

With xPack Arm GCC 15.2.1 the flash images used 14,204 bytes with calibration
off and 16,372 bytes with calibration on, before the delay checks came back
on Nov. 27, 2026. The SAMD11D14AM has 16,384 bytes. Built with xPack Arm GCC
14.2.1 and the same project options, the source with the delay checks is 640
bytes larger with calibration off and 772 bytes larger with calibration on
than the source before. With xPack Arm GCC 15.2.1, the current Release build
uses 16,088 bytes with calibration off, leaving 296 bytes. Calibration on no longer
fits the SAMD11D14AM. The D11 uses its nominal delay divisors. The D21 uses
its own TC3 measurements at 24 and 48 MHz. To repeat the D21 measurement,
set `CORE_CLOCK_TEST_DELAY_CALIBRATION` to 1 in its board.h.
