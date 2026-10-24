# Thingy:91 and Thingy:91 X device support

Working baseline: `prerelease_0.13_4`. This is a hardware inventory and implementation checklist, not a claim of completed board validation.

## Thingy:91 (PCA20035)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9160 LTE/GNSS | Modem | Existing LTE and GNSS examples |
| nRF52840 Bluetooth/USB | UART/USB | Existing MCU and HCI support |
| ADXL362 accelerometer | SPI | Existing `accel_adxl362` |
| ADXL372 high-g accelerometer | SPI | `accel_adxl372` integrated in the nRF9160 library; Thingy:91 SPI3 test project added; hardware validation pending |
| BME680 environmental sensor | I2C | Existing `tphg_bme680` |
| BH1749NUC color sensor | I2C | `color_bh1749`: RGB/IR readout, gain/timing and threshold interrupt support; Thingy:91 test project added; hardware validation pending |
| 24CW160 EEPROM | I2C | Existing universal external-memory interface; configure address and geometry, hardware validation pending |
| ADP5360 PMIC, charger and fuel monitor | I2C | `pm_adp5360`: BUCK/BUCKBOOST, charger/status, battery SOC/voltage, fault/PGOOD support; read-only Thingy:91 test added; hardware validation pending |
| RGB LED, button, buzzer, nRF52840 reset | GPIO/PWM | Shared Thingy:91 board helper and board test added; hardware validation pending |
| Board power rails | ADP5360 | 1.8 V BUCK and 3.3 V BUCKBOOST are modeled always-on by the current Nordic board description; status is covered by ADP5360Test and no separate nRF9160 sensor-rail enable GPIO is defined |

## Thingy:91 X (PCA20065)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9151 LTE/GNSS | Modem | Verify target MCU support and modem integration |
| nRF5340 application/network cores | IPC/radio | Existing generic nRF5340 support; board integration required |
| nRF7002 Wi-Fi | SPI/coexistence | Development paused; existing experimental code retained; do not continue until explicitly resumed |
| ADXL367 low-power accelerometer | I2C | `accel_adxl367` integrated in the nRF91x1 library; Thingy:91 X sensor-rail-aware test project added; hardware validation pending |
| BMI270 six-axis IMU | SPI | `ag_bmi270` repaired; production-driver host regression and Thingy:91 X SPI3 test project added; hardware validation pending |
| BMM350 magnetometer | I2C | `mag_bmm350` corrected to Bosch ODR/OTP compensation and 24-bit decoding; host regression and Thingy:91 X test project added; hardware validation pending |
| BME688 environmental sensor | I2C | Existing `tphg_bme680` forced-mode path is register-compatible (BME68x ID `0x61`, I2C `0x76/0x77`); Thingy:91 X power-aware compatibility test added; parallel/sequential gas modes are not exposed; hardware validation pending |
| GD25LE255E 256 Mbit flash | SPI | Existing universal external-memory interface; GD25LE255E 32 MiB/4-byte-address configuration and read-only Thingy:91 X test added; hardware validation pending |
| nPM6001 regulator PMIC | I2C | `pm_npm6001`: BUCK3 enable, PWM/HYS control, DAC voltage programming and thermal/BUCK3-OC monitoring; Thingy:91 X test project added; hardware validation pending |
| nPM1300 charger/fuel/LED PMIC | I2C | `pm_npm1300` software complete with production-driver host tests; Thingy:91 X hardware validation pending |
| RGB LEDs, button, external load switch | GPIO/PMIC | Shared Thingy:91 X board helper and board test added; hardware validation pending |
| Battery/USB power and shared sensor rails | PMIC/GPIO | Shared Thingy:91 X helper covers BUCK2/GPIO2, LDO1/LDO2, PMIC IRQ and nPM6001 READY; hardware validation pending |


## BMI270 Thingy:91 X integration

- BMI270 shares SPI3 with the external flash and nRF7002. SPI3 uses SCK P0.13, MOSI P0.14 and MISO P0.15.
- The shared chip-select order is external flash P0.12 = CS0, nRF7002 P0.17 = CS1 and BMI270 P0.10 = CS2. The BMI270 test keeps all three CS outputs configured and inactive except the selected device.
- BMI270 INT2 reaches nRF9120 P0.06. The initial board test uses polling, so interrupt validation is left for hardware testing.
- Sensor power is supplied through nPM1300 LDO2 using the common Thingy:91 X board helper.
- The production driver uses the BMI270 24-bit sensor-time counter when no external IOsonata timer is supplied, reads the live FIFO length registers, parses Bosch headerless FIFO order (gyro then accel), and converts die temperature to the generic C x 100 representation.
- Test project: `ARM/Nordic/nRF91/nRF91x1/exemples/BMI270Test`.
- Host regression: `python3 tests/sensors/run_bmi270.py`.

## Integration rules

- Device drivers live under the existing generic sensor, storage or power-management interfaces.
- Pin assignments, bus instance selection, supplies and chip selects belong to the board configuration.
- The Thingy:91 X shared buses must not be driven concurrently from both processors.
- Reuse existing drivers where the silicon device matches; do not treat similarly named parts as interchangeable without checking register and protocol compatibility.
- Add detection/readout tests and build each board target before recording hardware support as validated.
- Preserve existing modem, BLE, USB and networking applications.

## References

- Nordic Thingy:91 hardware user guide: https://docs.nordicsemi.com/r/bundle/ug_thingy91/
- Nordic Thingy:91 X hardware user guide: https://docs.nordicsemi.com/r/bundle/ug_thingy91x/
- Thingy:91 X hardware downloads: https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-91-X/Download

## nPM6001 Thingy:91 X integration

- The board uses nPM6001 BUCK3 for the Wi-Fi power domain; other outputs are unused or hardware-disabled.
- `pm_npm6001` does not alter rails on initialization. On Thingy:91 X, nPM1300 LDO1 is used as the nPM6001 ENABLE load switch and nPM1300 GPIO4 reads nPM6001 READY; the board test performs that sequence before accessing nPM6001.
- The internal BUCK3 DAC supports 500–3300 mV in 25 mV steps. Voltage updates are performed in forced PWM mode and restore the prior mode. The Thingy:91 X test leaves voltage programming disabled by default and reports the existing BUCK3 setting before enabling the rail.
- The driver covers BUCK3, thermal warning and BUCK3 overcurrent monitoring. General-purpose nPM6001 GPIO, watchdog, hibernate and unused output-rail programming are not integrated for Thingy:91 X.
- Test project: `ARM/Nordic/nRF91/nRF91x1/exemples/nPM6001Test`.


## Thingy:91 board integration

- `ARM/Nordic/nRF91/nRF9160/exemples/common/thingy91_board.*` centralizes button 1 on P0.26, the active-high RGB channels on P0.29/P0.30/P0.31, PWM1 buzzer output on P0.28 and the active-low nRF52840 reset on P0.10.
- `ARM/Nordic/nRF91/nRF9160/exemples/Thingy91BoardTest` deasserts the nRF52840 reset, emits one 440 Hz startup tone, cycles the RGB LED and reports the button state.
- The current Nordic board description does not define a separate nRF9160 sensor-power enable GPIO. The ADP5360 1.8 V BUCK and 3.3 V BUCKBOOST rails are both modeled always-on, so no extra sensor-rail control is invented in IOsonata.
- Hardware validation is pending.

## ADP5360 Thingy:91 integration

- The original Thingy:91 uses ADP5360 at I2C address `0x46`; BUCK is the always-on 1.8 V rail and BUCKBOOST is the always-on 3.3 V rail.
- `pm_adp5360` verifies device ID `0x10` without changing board power state. Regulator and charger programming is explicit.
- The driver exposes BUCK/BUCKBOOST voltage, current-limit and enable control, BUCK PWM mode, active discharge, charging configuration/status, battery presence/SOC/voltage, fault status and power-good status.
- `ARM/Nordic/nRF91/nRF9160/exemples/ADP5360Test` is read-only by default. Rail reprogramming is guarded by `ADP5360_TEST_PROGRAM_RAILS`.


## BME688 Thingy:91 X integration

- The onboard BME688 is at I2C address `0x76` and uses the register-compatible BME680/BME68x interface already implemented by `tphg_bme680`; no second BME688 driver is added.
- nPM1300 LDO2 is the Thingy:91 X sensor-domain load switch. `ARM/Nordic/nRF91/nRF91x1/exemples/BME688Test` explicitly selects load-switch mode and enables that rail before probing the sensor.
- The test uses the existing BSEC-backed gas path and reports temperature, pressure, humidity and gas-resistance data. Bosch BSEC remains an external build dependency exactly as in the existing TPHDemo.

## nPM1300 Thingy:91 X integration

- The PMIC is at I2C address `0x6B`.
- nPM1300 GPIO3 is the PMIC interrupt output; on the nRF9151 it reaches P0.02. The generic driver does not hard-code this board selection.
- BUCK2 is the main 3.3 V rail and is board-controlled by nPM1300 GPIO2. The nPM1300 driver exposes GPIO routing for BUCK enable/retention/PWM and load-switch enable instead of encoding this board policy in the driver.
- LDO1 is used as the nPM6001 enable load switch. LDO2 is the sensor-domain load switch. nPM1300 GPIO4 reads nPM6001 READY.
- `ARM/Nordic/nRF91/nRF91x1/exemples/common/thingy91x_board.*` centralizes these PMIC mappings, the nRF9151 P0.02 PMIC interrupt input, the P0.03 external-board 3.3 V enable, button 1 on P0.26 and the active-high RGB channels on P0.29/P0.31/P0.30. The nPM6001 and BME688 tests use the shared helper rather than duplicating rail sequences.
- `ARM/Nordic/nRF91/nRF91x1/exemples/Thingy91XBoardTest` initializes the board-safe PMIC routing, keeps external-board power off by default, cycles the RGB LED and reports button and nPM6001 READY state.
- The board battery profile uses a 4.2 V termination target and a 675 mA nominal 0.5C charge request. The nPM1300 has a 2 mA charge-current step, so `SetCharge()` programs 674 mA and reports 674 mA as the actual setting.
- Battery presence and state of charge remain unknown until a battery-specific host fuel-gauge model is integrated.
- Hardware validation is pending.
- Test project: `ARM/Nordic/nRF91/nRF91x1/exemples/nPM1300Test`; default mode is non-destructive and reports charger/status, VBAT, NTC temperature, BUCK/LDSW status and GPIO4.

- BMM350 validation project: `ARM/Nordic/nRF91/nRF91x1/exemples/BMM350Test`.
- ADXL367 validation project: `ARM/Nordic/nRF91/nRF91x1/exemples/ADXL367Test`.
- GD25LE255E validation project: `ARM/Nordic/nRF91/nRF91x1/exemples/GD25LE255ETest`; read-only by default.
- ADXL372 validation project: `ARM/Nordic/nRF91/nRF9160/exemples/ADXL372Test`.
