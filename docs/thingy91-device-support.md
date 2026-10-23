# Thingy:91 and Thingy:91 X device support

Working baseline: `prerelease_0.13_4`. This is a hardware inventory and implementation checklist, not a claim of completed board validation.

## Thingy:91 (PCA20035)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9160 LTE/GNSS | Modem | Existing LTE and GNSS examples |
| nRF52840 Bluetooth/USB | UART/USB | Existing MCU and HCI support |
| ADXL362 accelerometer | SPI | Existing `accel_adxl362` |
| ADXL372 high-g accelerometer | SPI | Missing |
| BME680 environmental sensor | I2C | Existing `tphg_bme680` |
| BH1749NUC color sensor | I2C | New `color_bh1749` driver; build/hardware validation pending |
| 24CW160 EEPROM | I2C | Existing generic `Seep` driver; configure device geometry and I2C address, hardware validation pending |
| PMIC, charger and fuel monitoring | I2C/GPIO | Board-specific integration required |
| RGB LEDs, buttons, buzzer | GPIO/PWM | Board-specific pin/power configuration required |
| Battery measurements and sensor rail enables | ADC/GPIO | Board-specific integration required |

## Thingy:91 X (PCA20065)

| Device | Interface | IOsonata status |
| --- | --- | --- |
| nRF9151 LTE/GNSS | Modem | Verify target MCU support and modem integration |
| nRF5340 application/network cores | IPC/radio | Existing generic nRF5340 support; board integration required |
| nRF7002 Wi-Fi | SPI/coexistence | Missing Wi-Fi device integration |
| ADXL367 low-power accelerometer | I2C | Missing |
| BMI270 six-axis IMU | SPI | Existing `ag_bmi270` |
| BMM350 magnetometer | I2C | Existing `mag_bmm350` |
| BME688 environmental sensor | I2C | Verify BME680 driver compatibility before enabling; not validated |
| GD25LE255ELIG 256 Mbit flash | SPI | Missing chip-specific flash driver |
| nPM6001 regulator PMIC | I2C | Missing |
| nPM1300 charger/fuel/LED PMIC | I2C | Existing `pm_npm1300`; board integration required |
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
