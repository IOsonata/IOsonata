# nRF5340 application-core StandbyDemo

This separate Eclipse managed project uses the existing, target-independent
`exemples/misc/power_off_demo.cpp` and does not modify the System OFF
PowerOffDemo project.

The default nRF5340 Audio DK (PCA10121) console uses application UARTE0
RX P1.04, TX P1.05. Define `NRF5340_DK_PCA10095` for the standard DK
(RX P0.22, TX P0.20). The board profile selects resumable
`POWER_DEMO_RESUME_WAKE=1`, LF timer 0 wake every 1000 ms, and the
static/stack RAM sample check. `SystemSleep` uses CPU WFI and preserves
NVIC wake sources and system clock ownership.

Build using the IOcomposer Eclipse managed project, or:

```sh
python3 tests/nordic/build_managed_power_clock.py --target nrf5340_app
```

Flash StandbyDemo to the application core. For simultaneous-core tests,
first allow the application core to release the network core and assign
its console pins through a suitable dual-core startup arrangement;
NetCoreStart is a separate application-core firmware image and cannot
execute simultaneously with StandbyDemo. The application-core first-pass
standalone test does not start the network core.

Observe repeated output:
`PowerOffDemo: Standby`, `Entering Standby; automatic timer wake`,
`UART rate after resume: PASS`, `CPU/APB clock after resume: PASS`,
`Standby timer wake: PASS`, increasing resumed counts, and
`SRAM sample PASS`. These are hardware acceptance checks, not
confirmed by the managed-project source wiring tests.
