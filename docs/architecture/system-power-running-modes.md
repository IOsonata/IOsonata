# CPU running power modes

IOsonata initializes a supported MCU core at the fastest operating CPU
frequency supported by its configured silicon and core port. Low-power running
mode is an explicit request, separate from sleep and power off.

## Generic System Core interface

```cpp
SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER);
// Application continues executing.
SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);
```

`SystemPowerModeGet()` reports the currently selected CPU running mode.
Neither function enters `SystemSleep()` or `SystemPowerOff()`.
The mode setter may return false if a transition is not supported, is invoked
from an interrupt context, or cannot preserve the core's required clocks.
Unsupported low-power requests must not silently change state.

Applications should not need MCU-specific clock register writes, UART baud
reconfiguration, or manual peripheral clock coordination. Each MCU backend
must honor the requirements of its peripherals before supporting a new mode.
Each MCU port retains its own maximum-frequency default calibration.
When CPU frequency changes, `SystemCoreClockUpdate()` updates delay factors.
DWT-based measurement is allowed without a Timer or RTC dependency.

## Nordic nRF5340

The application core starts at **128 MHz** (HCLK128M Div1) under IOsonata,
despite the hardware's 64 MHz reset divider. The low-power running mode uses
**64 MHz** (HCLK128M Div2). The network core remains at its fixed 64 MHz rate.

The nRF5340 application-core HCLK divider is independent of peripheral
PCLK16M/PCLK32M/PCLK64M sources, so this initial implementation does not
alter those domains or peripheral ownership. Production silicon can switch
HFCLKCTRL during operation. Mode switching is rejected in an ISR or with
interrupts masked. After selecting the divider, the implementation waits for
hardware settling and updates the observed CPU frequency and delay factors.

The default 128 MHz factors are those supplied by the MCU port. When running
at 64 MHz, the CPU-local DWT path may measure the assembly loop factors.
Returning to 128 MHz restores the port's default factor values.

Other Nordic cores currently advertise performance mode only; a low-power
running request returns false until that core has a verified hardware
implementation. This does not imply lack of sleep or peripheral-level
clock gating.

## Acceptance checks

- Verify startup application-core CPU clock is 128 MHz in Debug and Release
- Verify entering LOW_POWER returns 64 MHz and PERF returns 128 MHz
- Verify the delay factors at each frequency and when returning to default
- Verify UART communication and independent PCLK-based peripheral timing
- Verify repeated transitions while Bluetooth and asynchronous drivers run
- Confirm IRQ latency during optional CPU-local DWT calibration is acceptable

Eclipse CDT managed Debug/Release builds and hardware execution are required
before release sign-off. Host/source checks alone are insufficient.
