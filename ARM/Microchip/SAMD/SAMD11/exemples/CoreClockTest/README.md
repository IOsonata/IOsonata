# SAMD11D14AM CoreClockTest

Build the SAMD11 library and CoreClockTest with matching Eclipse configurations.
Output uses the EDBG UART at 1,000,000 baud.

The test checks HF and LF clock enable and disable, CPU running modes,
and peripheral clock requests, readback and restoration. The RTC runs while
the test enables and disables LF clock requests. Its counter must still
advance after the test releases its last request.

For delay calibration during porting, set `CORE_CLOCK_TEST_DELAY_CALIBRATION=1`
in the application's C++ preprocessor symbols. The same clock tests run.
SysTick measures the public delay loops at 48 and 24 MHz. Each
`Delay divisors` line gives the CPU frequency and the microsecond and
nanosecond divisors scaled by 1,000,000.

With xPack Arm GCC 15.2.1 the flash images use 14,204 bytes with calibration
off and 16,372 bytes with calibration on. The SAMD11D14AM has 16,384 bytes.
