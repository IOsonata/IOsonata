# nRF5340 generic power API validation

The application and network cores each have exactly one Eclipse managed
PowerApiTest project, using the shared `exemples/misc/power_api_test.cpp`
implementation. The previous app StandbyDemo duplicate is removed.

The application-core board profile currently selects System OFF and
retention verification; the network-core board profile selects resumable
standby with RTC timer wake. Both paths print per-cycle API PASS/FAIL
records and a failure count. Reset-on-wake results are checked after
the next boot, not upon returning from SystemPowerOff.

The generic implementation preserves its legacy build path through a
one-line compatibility wrapper, for other MCU projects not yet migrated.

The same app project can build the resumable configuration by defining
POWER_DEMO_RESUME_WAKE, POWER_DEMO_TIMER_WAKE_MS,
POWER_DEMO_TIMER_WAKE_DEVNO and POWER_DEMO_STANDBY_RAM_TEST in its
board profile. Separate demo projects are not needed.
