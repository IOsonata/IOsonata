# Thingy:91 and Thingy:91 X device support

Working baseline: `prerelease_0.13_4`. This is a hardware inventory and implementation checklist, not a claim of completed board validation.

## Thingy:91 (PCA20035)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9160 LTE/GNSS | Modem | Existing LTE and GNSS examples |
| nRF52840 Bluetooth/USB | UART/USB | Existing MCU and HCI support |
| ADXL362 accelerometer | SPI | Existing `accel_adxl362` |
| ADXL372 high-g accelerometer | SPI | `accel_adxl372` compiled; hardware validation pending |
| BME680 environmental sensor | I2C | Existing `tphg_bme680` |
| BH1749NUC color sensor | I2C | `color_bh1749`: RGB/IR readout, gain/timing and threshold interrupt support; Thingy:91 test project added; hardware validation pending |
| 24CW160 EEPROM | I2C | Existing universal external-memory interface; configure address and geometry, hardware validation pending |
| ADP5360 PMIC, charger and fuel monitor | I2C | `pm_adp5360`: BUCK/BUCKBOOST, charger/status, battery SOC/voltage, fault/PGOOD support; read-only Thingy:91 test added; hardware validation pending |
| RGB LEDs, buttons, buzzer | GPIO/PWM | Board-specific pin/power configuration required |
| Battery measurements and sensor rail enables | ADC/GPIO | Board-specific integration required |

## Thingy:91 X (PCA20065)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9151 LTE/GNSS | Modem | Verify target MCU support and modem integration |
| nRF5340 application/network cores | IPC/radio | Existing generic nRF5340 support; board integration required |
| nRF7002 Wi-Fi | SPI/coexistence | Development paused; existing experimental code retained; do not continue until explicitly resumed |
| ADXL367 low-power accelerometer | I2C | `accel_adxl367` compiled; hardware validation pending |
| BMI270 six-axis IMU | SPI | Existing `ag_bmi270` |
| BMM350 magnetometer | I2C | Existing `mag_bmm350` |
| BME688 environmental sensor | I2C | Existing `tphg_bme680` forced-mode path is register-compatible (BME68x ID `0x61`, I2C `0x76/0x77`); Thingy:91 X power-aware compatibility test added; parallel/sequential gas modes are not exposed; hardware validation pending |
| GD25LE255E 256 Mbit flash | SPI | Existing universal external-memory interface; configure device geometry/commands, hardware validation pending |
| nPM6001 regulator PMIC | I2C | `pm_npm6001`: BUCK3 enable, PWM/HYS control, DAC voltage programming and thermal/BUCK3-OC monitoring; Thingy:91 X test project added; hardware validation pending |
| nPM1300 charger/fuel/LED PMIC | I2C | `pm_npm1300` software complete with production-driver host tests; Thingy:91 X hardware validation pending |
| RGB LEDs, buttons, external load switch | GPIO/PMIC | Board integration required |
| Battery/USB power and shared sensor rails | PMIC/GPIO | Board integration required |

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
- `ARM/Nordic/nRF91/nRF91x1/exemples/common/thingy91x_board.*` centralizes these PMIC mappings, the nRF9151 P0.02 PMIC interrupt input and the P0.03 external-board 3.3 V enable. The nPM6001 and BME688 tests use the shared helper rather than duplicating rail sequences.
- The board battery profile uses a 4.2 V termination target and a 675 mA nominal 0.5C charge request. The nPM1300 has a 2 mA charge-current step, so `SetCharge()` programs 674 mA and reports 674 mA as the actual setting.
- Battery presence and state of charge remain unknown until a battery-specific host fuel-gauge model is integrated.
- Hardware validation is pending.
- Test project: `ARM/Nordic/nRF91/nRF91x1/exemples/nPM1300Test`; default mode is non-destructive and reports charger/status, VBAT, NTC temperature, BUCK/LDSW status and GPIO4.
