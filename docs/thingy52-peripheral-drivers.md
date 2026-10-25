# BH1745, CCS811 and SX1509B drivers

These are MCU-independent IOsonata drivers using synchronous I2C `DeviceIntrf`
transactions. They do not require a Nordic SDK, Zephyr or a vendor binary.
The application owns the bus, board power sequencing, pin assignments and
interrupt wiring. Serialize access to each device; do not call bus operations
from an ISR. Transport objects and callback contexts must outlive the drivers.

| Device | Class | Header | Source |
| --- | --- | --- | --- |
| BH1745NUC | `ColorBh1745` | `sensors/color_bh1745.h` | `src/sensors/color_bh1745.cpp` |
| CCS811 | `GasCcs811` | `sensors/gas_ccs811.h` | `src/sensors/gas_ccs811.cpp` |
| SX1509B | `IoExpSx1509` | `miscdev/ioexp_sx1509.h` | `src/miscdev/ioexp_sx1509.cpp` |

The headers and sources are linked into the nRF52832, nRF52840, nRF5340 App,
nRF9160 and nRF91x1 library projects. Other builds can compile the same sources
with their existing Device/DeviceIntrf and delay implementations. Shared base
interfaces, MCU selections and linker scripts are unchanged.

## BH1745

`Init` checks both identity registers, resets the device and leaves conversion
stopped. `Configure` accepts integration-time codes 0..5 and gains 1, 2 or 16.
`Enable` starts conversion. `Read` returns new raw RGBC counts without invented
lux or colour-temperature calibration. Failed/not-ready reads leave the supplied
sample unchanged. Reading the ready flag consumes it; a failed data transfer
may therefore lose that sample. This is a separate implementation from BH1749.
Interrupt source, persistence, thresholds, latching and clearing are supported.

```cpp
#include "sensors/color_bh1745.h"

bool StartColor(ColorBh1745 &Color, DeviceIntrf &Bus)
{
	return Color.Init(&Bus) && Color.Configure(0, 16) && Color.Enable();
}
```

## CCS811

`GasCcs811` derives from `GasSensor`. The typed `Ccs811Data_t` supplies eCO2 in
ppm, eTVOC in ppb and raw ADC/current readings. `AlgorithmValid` distinguishes
algorithm output from raw-only mode; it does not certify sensor conditioning.
The generic read supplies resistance, `MeasIdx=-1` for zero-current samples,
and `AirQualIdx=NaN`; eCO2 is never passed off as an air-quality index.
Generic reads return the previous cached value on failure. Typed reads leave
the caller's value unchanged on failure.

Initialization checks hardware/application identity and starts the installed
application firmware. Missing firmware is an error; this driver does not flash
CCS811 firmware. Supported sampling frequencies are **mHz**, following `Sensor`:
1000, 100, 17 (one sample/minute, exact period 60 seconds), and 4000 (raw only).
Frequency requests select the nearest rate (ties select the lower rate); zero
clamps to 17 mHz. Read back `SamplingFrequency()` to obtain the selected value.
A transition blocked by the idle requirement leaves the old rate selected.
Single-shot/timer modes and external heater profiles are unsupported.
`IntHandler` only sets a flag; call `Read`/`UpdateData` in application context.

Provide `WakeControl` before initialization when nWAKE is GPIO-controlled; absent
a callback, the board must hold it low. The driver brackets each transaction
with wake timing and releases wake after transport failures. GPIO callbacks
must return failure when their own transport fails.

Environment compensation, baseline save/restore, thresholds, interrupts,
firmware-version reads and error acknowledgment are provided. Observe initial
burn-in and 20-minute run-in before relying on measurements/restoring a baseline.
To reduce sampling rate, enter mode 0, wait at least 600000 ms using the board's
monotonic clock, then pass the actual elapsed idle time to `SetDriveMode`.
The driver rejects direct reductions and never blocks for ten minutes.
A reset invalidates the object and returns the chip to its bootloader; call
`Init` again. It retains prior sampling history across reset/reinitialization
so reset cannot silently bypass the lower-rate idle requirement.

```cpp
#include "sensors/gas_ccs811.h"

bool StartGas(GasCcs811 &Gas, DeviceIntrf &Bus)
{
	GasSensorCfg_t cfg = {};

	cfg.DevAddr = CCS811_I2C_ADDR0;
	cfg.OpMode = SENSOR_OPMODE_CONTINUOUS;
	cfg.Freq = 1000;
	return Gas.Init(cfg, &Bus);
}
```

## SX1509B

`Init` attaches without resetting outputs, and disables interrupt autoclearing
on pin reads. The chip has no identity register; the board must know its address.
`Disable` deliberately leaves outputs driving so it cannot drop board rails.
`SoftReset` is explicit and destructive: pins return to inputs.

GPIO configuration includes direction, pulls, inversion, open drain, drive/slew,
high-input mode, debounce, edge interrupts and event acknowledgment. LED support
includes brightness, static operation, blink/single shot and breathing on the
capable pins. Configure the shared oscillator/LED clock before debounce or LEDs.
LED timing fields are datasheet codes, not milliseconds. Clock/divider settings
and logarithmic mode affect multiple channels. Keypad scanning and dedicated
level-shifter modes are outside this GPIO/LED implementation.

**Never seed an output latch from `ReadPins`.** Reads report physical levels,
including externally pulled-low open-drain pins. Before masked output writes,
either explicitly reset, call `WriteOutputs` with all desired latch bits, or
`AdoptOutputLatch` with an exact latch value maintained by the board owner.
`ConfigurePin` preloads its initial logical latch level before enabling output.
Unrelated latch bits are preserved. A failed output write makes that bank's
shadow unknown; resynchronize explicitly before further masked writes.
Do not write the expander's registers behind the driver's back.

Multi-register configuration can partially succeed and is not transactional.
Check every result; do not assume rollback after an I2C error. GPIO-bank reads
are sequential rather than simultaneous snapshots. `ClearInterrupts` writes
only the supplied W1C mask, so pass the sources actually handled.

Example callback for an active-low CCS811 wake signal routed through an expander:

```cpp
#include "miscdev/ioexp_sx1509.h"
#include "sensors/gas_ccs811.h"

struct GasWakeContext {
	IoExpSx1509 *Expander;
	uint8_t Pin;
};

bool GasWake(bool Active, void *pContext)
{
	GasWakeContext *ctx = static_cast<GasWakeContext *>(pContext);
	uint16_t mask;

	if (ctx == nullptr || ctx->Expander == nullptr || ctx->Pin > 15)
		return false;
	mask = 1U << ctx->Pin;
	return ctx->Expander->WritePins(mask, Active ? 0 : mask);
}
```

The board initializes the expander, establishes its exact output latch and
configures the selected wake pin as a non-inverted output before installing
this callback. Pin numbers and rail policy are intentionally not embedded in
the generic drivers.

## Verification

Run `python3 tests/sensors/run_thingy52_devices.py` with a host C++ compiler.
It compiles the production drivers and real Device/DeviceIntrf implementation
against a mock bus at O0 and O2 with strict warnings, AddressSanitizer and UBSan.
The runner defaults to disabling LeakSanitizer for restricted test environments;
these production drivers allocate no heap memory.

Coverage includes identities, failed/short transfers, unchanged sample outputs,
endianness, conversion modes, startup command framing, wake release, environment
encoding, baseline operations, interrupt masks, LED register layouts and output
latch preservation/invalidations. Hardware and complete target firmware builds
have not been validated in this environment.

Register references:

- [ROHM BH1745NUC Rev.002](https://media.digikey.com/pdf/Data%20Sheets/Rohm%20PDFs/BH1745NUC.pdf)
- [ams CCS811 v1-06](https://cdn.sparkfun.com/assets/2/c/c/6/5/CN04-2019_attachment_CCS811_Datasheet_v1-06.pdf)
- [Semtech SX1508B/SX1509B Rev.4](https://www.mouser.com/datasheet/2/761/sx150x_89b-1283302.pdf)
