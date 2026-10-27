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

## Thingy:52 board test

Import these two IOcomposer/Eclipse projects:

- `ARM/Nordic/nRF52/nRF52832/lib/ioc` (`IOsonata_nRF52832`).
- `ARM/Nordic/nRF52/nRF52832/exemples/Thingy52PeripheralTest/ioc`.

Build the library and application using the matching Debug or Release
configuration. The application uses `NRF52832_XXAA` and the existing
`gcc_nrf52_xxaa.ld` full-flash linker script. It is a standalone test, without
SoftDevice or the old Thingy52 firmware. Program it through an external SWD
probe. Both configurations use the external UART console, without semihosting.

Connect an adapter with logic levels compatible with the board's VREG supply:

| Thingy:52 pin | Adapter connection |
| --- | --- |
| P0.02 / ANA_DIG0 / P4 pin 3 | RX |
| P0.03 / ANA_DIG1 / P4 pin 4 | TX (optional; test only prints) |
| P4 pin 6 or 20 | GND |

Use **115200 baud, 8N1, no flow control**. The USB connector supplies power and
charging; it is not a USB-to-UART bridge. Do not use an RS-232 voltage adapter.

The test owns the production PCA20020 v1.x board's power and expander state:

- P0.30 enables VDD, then P0.16 releases the expander reset input.
- I2C0 uses SDA P0.07 and SCL P0.08, with synchronous EasyDMA transfers.
- LIS2DH12 is at 0x19 on I2C1, SDA P0.14 and SCL P0.15, on the always-on rail.
- MPU9250 is at 0x68 on I2C0. `Thingy52MotionPower` enables SXIO8 and waits
  100 ms before initialization. It preserves gas, microphone and LED outputs.
- HTS221 (0x5F) and LPS22HB (0x5C) share I2C0. Both run at 1 Hz.
- The expander is explicitly reset. External GPIOs 0..3 and battery-monitor
  pin 4 remain inputs. Motion, microphone and gas supplies start off, as does
  the speaker amplifier. Both RGB LED groups start off.
- CCS811 power is SXIO10, reset is SXIO11 and wake is SXIO12. Power-up waits
  30 ms before reset release; shutdown lowers reset and wake before power.
- Lightwell RGB uses SXIO7/5/6; color-sensor illumination uses SXIO13/14/15.
- The button is P0.11; color and gas interrupt inputs are P0.31 and P0.22.

The reusable implementation is `exemples/common/thingy52_board.*` under the
nRF52832 directory. Call its startup functions only when taking ownership of
the board; they deliberately reset existing expander configuration. Do not run
them while another application is using the expander or switched supplies.
The old 0.7/0.8/0.9 prototype boards have different power polarity and are not
supported by these helpers.

After startup, the console prints CCS811 firmware versions, raw BH1745 RGBC
counts, gas measurements, button state and interrupt pin levels once per loop.
The lightwell cycles red/green/blue. Hold the button until the next sample to
toggle the color-sensor illumination. The first color sample after a toggle
may precede the illumination change. The color interrupt window spans the
full range and normally remains inactive; gas data-ready interrupts are
polled and acknowledged by the gas read. No MCU interrupt handler is installed.

The test also reads the existing `AccelLisxdh`, `AgmMpu9250`, `HumiHts221` and
`PressLps22hb` drivers. Motion output is signed raw counts; the AK8963 counts
include factory sensitivity trim. These are register snapshots, not a motion
fusion or FIFO streaming test. The gyro uses its explicit unfiltered 32 kHz
mode; the console reads it once per loop. The timer timestamps let the example
recognize a new magnetometer sample and complete humidity/temperature pairs.
HTS221 reports centi-degrees C and centi-percent RH; LPS22HB reports Pa.

CCS811 compensation is updated only after a fresh HTS221 temperature/humidity
pair, within the driver's supported temperature range. Until that first pair,
the chip defaults apply. Missing/partial samples retain the previous
compensation. No saved baseline is restored. Early readings do not establish accuracy;
allow sensor conditioning. A failed initialization or board-control write
prints `STOP` and stops further bus operations. Not-ready sensor reads are
reported separately. No firmware update or persistent configuration is written.

Run `python3 tests/sensors/run_thingy52_board.py` for board sequence tests and
host syntax checks, and `python3 tests/sensors/run_thingy52_devices.py` for the
generic driver tests. The board tests use the production SX1509/CCS811 drivers
on a shared mock bus and check rail defaults, wake bracketing, LED/control-bit
preservation, power-off ordering and transfer failures. These tests do not
replace an ARM build or hardware validation; both remain pending.

`python3 tests/sensors/run_thingy52_motion.py` checks the existing motion drivers
on I2C and SPI at O0/O2 with ASan/UBSan. It covers LIS2DH12 polling with an empty
FIFO, even FIFO counts, signed high-resolution samples, address increment,
MPU9250 I2C bypass after reset, signed AG samples, little-endian AK8963 samples,
factory trim, overflow rejection and short transfers. The direct-register path
is used by this example; DMP/FIFO packet parsing is not exercised.

The nRF52832 library already linked the motion drivers; the HTS221 and LPS22HB
header/source links are now included as well. Rebuild the library before the
example. Microphone, speaker and battery measurement are outside this test.

Board references:

- [Nordic pin maps](https://docs.nordicsemi.com/r/bundle/ug_thingy52/page/ug/thingy52/hw_description/pin_maps.html)
- [Nordic connector pinouts](https://docs.nordicsemi.com/r/bundle/ug_thingy52/page/ug/thingy52/hw_description/hw_connector_pinout.html)
- [Nordic gas-sensor power sequencing](https://github.com/NordicSemiconductor/Nordic-Thingy52-FW/blob/master/source/drivers/drv_gas_sensor.c)
