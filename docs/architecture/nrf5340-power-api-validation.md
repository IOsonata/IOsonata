# nRF5340 CorePowerTest runtime validation

CorePowerTest is one generic firmware image per MCU. It queries the
power implementation's `SystemPowerCapabilitiesGet` mask at runtime and runs
every supported check. Board headers contain only physical UART, GPIO
and reserved RAM resources. They do not select System OFF or standby.

The App core advertises CPU reduced-speed mode, resumable sleep with RTC
wake, wake pin, System OFF, retention registers and reserved-RAM retention.
The Net core advertises resumable sleep with RTC wake and retention
registers; independent System OFF is unsupported.

The runner first checks reset reasons and retention registers, verifies
results preserved from the previous System OFF cycle, then exercises
running power mode transitions and three timer-driven sleep/resume
cycles. It reports each result and a per-boot failure total. On App,
System OFF is the final action and the next boot verifies the reset
reason and retained/nonretained RAM; no single execution can continue
after actual System OFF. On Net the unsupported phases print UNSUPPORTED.

The generic capability API is a port service. Other MCU targets need
their own capability implementation to receive exhaustive runtime
coverage. A missing capability API is reported explicitly, never guessed.
Compatibility wrappers retain older Eclipse project source links until
those projects are migrated.

CorePowerTest reports API behavior; it does not independently measure
power consumption or oscillator source quality.

CorePowerTest uses interrupt-driven UARTE with EasyDMA for console output
on nRF5340. Driver-owned asynchronous transfers continue independently of
the test's CPU polling logic. The test uses only the generic UART interface.
