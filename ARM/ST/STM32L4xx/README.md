# STM32L4 port status

STM32L476, STM32L496 and STM32L4S9 are supported at the minimum MCU level:
startup, GPIO, UART and timers are implemented. Library projects
exist for each MCU. The maintainer confirms these STM32L4 ports are hardware
validated and used in existing projects (2026-10-08). The UART was then the
L4 copy `src/uart_stm32l4xx.cpp` and the timer the L4 LPTIM copy
`src/timer_lptim_stm32l4xx.cpp`. The libraries now build the STM32 UART and
timer drivers shared with the other families, `ARM/ST/src/uart_stm32.cpp` and
`ARM/ST/src/timer_*stm32.cpp`, and the L4 copies are removed; the shared
drivers have not been run on L4 hardware yet.
The automated checks
below describe the scope of the recent source review; they do not mean the
ports lack hardware validation. See the
[central MCU support list](../../../docs/supported-targets.md).
An MCU needs startup, GPIO, UART and timer support to be considered supported.
Peripheral source files and project metadata alone do not establish this.
Hardware validation is recorded separately from implementation coverage.

STM32L432 (STM32L432KC on the NUCLEO-L432KC) has a library project and the
example projects Blinky, TimerDemo, UartLoopback and UartPrbsTxTest built from
the same L4 sources (2026-10-13, Blinky 2026-10-19). It is not tested on
hardware yet; see the STM32L432 paragraph under Validation.

## Virtual device numbers

UART indices are 0 for LPUART1, 1 for USART1, 2 for USART2, 3 for USART3,
4 for UART4 and 5 for UART5. IRQ selection, reset and clock control use this
mapping. These indices are not hardware instance numbers. The STM32L432 has
LPUART1, USART1 and USART2 only; indices 3 to 5 fail initialization.

UART supports seven or eight data bits, none/even/odd parity and one or
two stop bits on every instance, LPUART1 included, in polling, interrupt or
DMA mode. Other framing, nine data bits, synchronous mode and IrDA fail
initialization before the device is changed. USART1 to UART5 run from PCLK,
the driver writes their CCIPR kernel clock selection to 0. LPUART1 selects
LSE, HSI16 or PCLK1 for the requested rate. A character received with the RX
CFIFO full is dropped and counted in `RxDropCnt` with a blocking CFIFO
(`bFifoBlocking`); a non-blocking CFIFO drops its oldest byte instead.

UART DMA (`bDMAMode`, with or without `bIntMode`) sends TX blocks straight
from the TX CFIFO and receives into a circular ring moved into the RX CFIFO,
as on the F0 parts. With a non-blocking CFIFO each TX block, at most 32 bytes,
is a copy taken out of the CFIFO, see `ARM/ST/STM32F0xx/README.md` (STM32L432
in the emulator at 80 MHz and 1 Mbaud: loopback 96.6 kB/s, 99.0 with a
blocking CFIFO). Each channel has its own vector. Since 2026-10-19 the vectors
belong to the DMA channel manager `ARM/ST/src/dma_stm32.cpp`, which all four
L4 libraries build: a driver claims the channels the reference manual gives
its request, the first to ask has them (UART, SPI and I2C). L476 and L496
select the request in the DMA1 or DMA2 CSELR, L4+ parts route it through
DMAMUX1 (DMA1 channel n on DMAMUX1 channel n - 1, DMA2 channel n on DMAMUX1
channel n + 6). On the NUCLEO-L432KC (2026-10-19), UartPrbsTxTest on USART2,
the ST-LINK virtual COM port, at 1 Mbaud: DMA without interrupts and DMA with
interrupts 99.4 kB/s, interrupts without DMA 99.9 kB/s, no loss after the
start of the stream.

| DevNo | Hardware | TX DMA | RX DMA | CSELR | DMAMUX1 TX, RX |
|---|---|---|---|---|---|
| 0 | LPUART1 | DMA2 channel 6 | DMA2 channel 7 | 4 | 35, 34 |
| 1 | USART1 | DMA1 channel 4 | DMA1 channel 5 | 2 | 25, 24 |
| 2 | USART2 | DMA1 channel 7 | DMA1 channel 6 | 2 | 27, 26 |
| 3 | USART3 | DMA1 channel 2 | DMA1 channel 3 | 2 | 29, 28 |
| 4 | UART4 | DMA2 channel 3 | DMA2 channel 5 | 2 | 31, 30 |
| 5 | UART5 | DMA2 channel 1 | DMA2 channel 2 | 2 | 33, 32 |

The DMAMUX1 numbers are those of the L4R and L4S; the L4P and L4Q use one
more.

SPI indices 0-2 select SPI1-3. On L432/L476/L496, index 3 selects QUADSPI.
On L4S9, indices 3 and 4 select OCTOSPI1 and OCTOSPI2. The L432 has no SPI2:
index 1 fails initialization. QUADSPI and OCTOSPI accept polling master
configurations only; interrupt, DMA and slave requests fail initialization.

The four L4 libraries build the shared STM32 SPI driver,
`ARM/ST/src/spi_stm32.cpp`. SPIInit passes index 3 and up to
`ARM/ST/src/quadspi_stm32.cpp` (L432, L476, L496) or
`ARM/ST/src/octospi_stm32.cpp` (L4S9), declared in
`ARM/ST/include/qospi_stm32.h`. These replace `src/spi_stm32l4xx.cpp`,
`src/quadspi_stm32l4xx.cpp`, `src/octospi_stm32l4xx.cpp` and
`include/spi_stm32l4xx.h`, removed (2026-10-19). The QUADSPI rate is now
HCLK / (PRESCALER + 1), as for the OCTOSPI: the old code divided by
PRESCALER, a division by zero at the highest rate. The QUADSPI StopTx now
raises the chip select it lowered (QSPI_CS_IOPIN_IDX).

SPI1 to SPI3 (DevNo 0 to 2), master in polling and interrupt mode, slave in
interrupt mode with the hardware NSS pin, 4 to 16 bit data, the rate from the
BR divider. DMA for master and slave through the DMA channel manager: SPI1 TX
DMA1 channel 3 and RX channel 2 (CSELR 1), or DMA2 channels 4 and 3 (CSELR 4)
when those are taken; SPI2 DMA1 channels 5 and 4; SPI3 DMA2 channels 2 and 1
(CSELR 3). The STM32L432 example SPIMasterSlave runs SPI1 as master
(A1 PA1 SCK, D10 PA11 MISO, A6 PA7 MOSI, D3 PB0 chip select GPIO) against SPI3
as slave (D13 PB3 SCK, D12 PB4 MISO, D11 PB5 MOSI, A3 PA4 NSS) on one
NUCLEO-L432KC, wired A1 to D13, A6 to D11, D10 to D12 and D3 to A3. The A6 and
D12 lines are shared with the I2CMasterSlave wiring. On the NUCLEO-L432KC
(2026-10-19), master polling and master interrupt, each against the interrupt
driven slave, pass in SPI modes 0 to 3 at 625 kHz, 8 bit data: master transmit
and receive frames of 1, 16 and 129 bytes, and a 1 byte frame that discards
the slave transmit prefetch before the next frame. With DMA (2026-10-19) the
same test passes in all 8 combinations: master polling, interrupt, DMA and DMA
with interrupt, each against the interrupt driven slave and the DMA slave.

The master chip select is a GPIO driven by the driver; the hardware NSS is
used by the slave only. The old driver drove the master NSS pin by hardware
with a single chip select, so the L476, L496 and L4S9 SPIMasterSlave examples
now configure the master chip select pin as a GPIO. The shared driver is not
tested on the L476, L496 and L4S9 hardware yet. On the B-L475E-IOT01A the
STM32L476 SPIMasterSlave runs SPI1 as master on the Arduino header (D13 PA5
SCK, D12 PA6 MISO, D11 PA7 MOSI, D10 PA2 chip select GPIO) against SPI2 as
slave on the Pmod connector CN10 (pin 4 PD1 SCK, pin 3 PD3 MISO, pin 2 PD4
MOSI, pin 8 PD0 NSS), wired D13 to pin 4, D11 to pin 2, D12 to pin 3 and D10
to pin 8. The other SPI2 pins are taken on this board: PB12 and PB13 by the
WiFi module, PB14 by LD2, PB15 by the sub-GHz module.

## Timer

The libraries build the STM32 timer driver: `ARM/ST/src/timer_stm32.cpp`
(TimerInit), `timer_lptim_stm32.cpp` (LPTIM) and `timer_tim_stm32.cpp` (TIM),
described in `ARM/ST/include/timer_stm32.h`. Timer DevNo are virtual
indices, the LPTIM first (`TimerGetLowFreqDevCount` 2), then the TIM from the
basic timers to the advanced timers:

| DevNo | L476, L496, L4S9 | L432 | Triggers |
|---|---|---|---|
| 0 | LPTIM1 | LPTIM1 | 1 |
| 1 | LPTIM2 | LPTIM2 | 1 |
| 2 | TIM6 | TIM6 | 1, the counter period |
| 3 | TIM7 | TIM7 | 1, the counter period |
| 4 | TIM16 | TIM16 | 1 |
| 5 | TIM17 | TIM15 | 1 (TIM17), 2 (TIM15) |
| 6 | TIM15 | TIM2 | 2 (TIM15), 4 (TIM2) |
| 7 | TIM2 | TIM1 | 4 |
| 8 to 12 | TIM3, TIM4, TIM5, TIM1, TIM8 | | 4 |

An LPTIM counts at its kernel clock, LSI, LSE (crystal or bypass with
`OSC_TYPE_TCXO`) or HSI16, divided by 1 to 128, 16 bits extended to 64 in
software; one compare trigger (CMP). CFGR and IER are written with the timer
disabled, ARR and CMP with the timer enabled, confirmed by ARROK and CMPOK.
A continuous trigger is scheduled one period after its previous compare
value. A TIM counts at the APB timer clock divided by 1 to 65536,
`TIMER_CLKSRC_DEFAULT` only; its triggers are its compare channels, a
deadline longer than the counter period is kept in software. External
trigger and the tick interrupt are not supported. The old L4 LPTIM driver
also took an external trigger; no IOsonata code used it, it wrote CFGR with
the timer enabled, and its timeout mode restarts the counter of the 64 bits
count.

The timer vectors of the vector file are weak aliases of
`Stm32TimerVector`: TimerInit refuses a timer whose vector is replaced by an
application handler (TIM6 shares its vector with the DAC). The details are
in `ARM/ST/STM32F0xx/README.md` and `ARM/ST/include/timer_stm32.h`.

## I2C limits

The four L4 libraries build the shared STM32 I2C driver,
`ARM/ST/src/i2c_stm32.cpp`; `src/i2c_stm32l4xx.cpp` is removed (2026-10-19):
master in polling and interrupt mode, slave in interrupt mode, 7 bit
addresses. DMA for master and slave through the DMA channel manager: I2C1 TX
DMA1 channel 6 and RX channel 7 (CSELR 3), or DMA2 channels 7 and 6 (CSELR 5)
when those are taken; I2C2 DMA1 channels 4 and 5; I2C3 DMA1 channels 2 and 3
(CSELR 3); I2C4 DMA2 channels 2 and 1 (CSELR 0). The STM32L432 example
I2CMasterSlave runs I2C1 (DevNo 0, D1 PA9 SCL, D0 PA10 SDA) as master against
I2C3 (DevNo 2, A6 PA7 SCL, D12 PB4 SDA) as slave on one NUCLEO-L432KC, wired
D1 to A6 and D0 to D12. The Release build prints on the ST-LINK virtual COM
port at 1 Mbaud. On the NUCLEO-L432KC (2026-10-19), master polling and master
interrupt, each against the interrupt driven slave, pass: write 8 bytes at an
offset, read them back, and a 4 byte read, at 100 kHz with the internal
pull-ups. With DMA (2026-10-19) the same test passes in all 8 combinations:
master polling, interrupt, DMA and DMA with interrupt, each against the
interrupt driven slave and the DMA slave.

The shared driver is not tested on the L476, L496 and L4S9 hardware yet. The
old driver accepted polling master configurations only.
The STM32L476 EepromDemo (`exemples/storage/eeprom_demo.cpp`) needs an
external I2C EEPROM on the B-L475E-IOT01A Arduino header, I2C1 (DevNo 0, D14
PB9 SDA, D15 PB8 SCL, open drain), by default a 24xx32 class part at 0x50.
The STM32L476 MotionSensorDemo (`exemples/sensor/mot_sensor_demo.cpp`) reads
the onboard LSM6DSL of the B-L475E-IOT01A (`src/sensors/ag_lsm6dsl.cpp`) on
I2C2 (DevNo 1, SDA PB11, SCL PB10) at 0x6A, with its data ready on INT1 PD11,
EXTI line 11. The Debug build prints with semihosting.
`tests/sensors/run.py` runs the driver against a register model of the
LSM6DSL. B-L475E-IOT01A2 run (2026-10-21), Debug, 2 g full scale: at rest
the acceleration magnitude reads 1.02 to 1.04 g in two orientations, the axes
follow the board when it is turned and reach the 2 g full scale when it is
shaken. The data ready interrupt on PD11 arrives and its handler reads the
status and the 14 output bytes in 2.4 ms at 100 kHz I2C. The printed interval,
about 100 ms, is the time of one semihosting printf, not the 52 Hz output
rate. The demo needs float printf (`-u _printf_float`) with newlib-nano.
Second run the same day with the gyroscope printf, 125 dps full scale: in 53
readings at rest the angular rate reads 2.35 to 2.56 dps on X, -3.03 to -2.94
dps on Y and about -0.84 dps on Z, with 3 Z readings between -2.7 and -5.4 dps
while the acceleration stays at rest. The three axes follow the board when it
is turned and clip at 143.34 dps, raw 32764, either sign, when it is shaken.
The rate scale is not checked against a known rotation rate.
The STM32L476 example EnvSensorDemo (`exemples/sensor/env_sensor_demo.cpp`)
reads the onboard HTS221 (`src/sensors/humi_hts221.cpp`) on the same bus at
0x5F, 1 Hz, and prints the humidity and the temperature on the ST-LINK virtual
COM port at 115200 8N1 in Release, with semihosting in Debug.
`tests/sensors/run.py` also runs this driver against a register model.
B-L475E-IOT01A2 run (2026-10-21), Release: the HTS221 is found and prints one
reading per second, 13.49 % and 30.93 C for the first one, then about 17.8 %
and 31.4 C. Not compared with a reference instrument.
EnvSensorDemo also reads the onboard LPS22HB (`src/sensors/press_lps22hb.cpp`)
at 0x5D, 1 Hz, and prints the pressure in hPa and its temperature. The driver
finds the device at either address, 0x5C or 0x5D, when none is given, and
reads the status and both outputs in one I2C transfer. `tests/sensors/run.py`
runs it against a register model. B-L475E-IOT01A2 run (2026-10-21), Release:
the LPS22HB is found at 0x5D. 43 readings go from 998.48 to 1000.15 hPa while
its temperature goes from 24.55 to 25.04 C. The HTS221 on the same bus reads
44.48 to 45.16 % and 23.66 to 24.14 C, 0.84 to 0.95 C below the LPS22HB. The
temperature registers do come out of the single transfer: read again from the
pressure registers they would print about 265 C. The demo printed 60 HTS221
lines for the 43 LPS22HB lines: 16 HTS221 lines follow another HTS221 line
with the same temperature and a new humidity. The HTS221 humidity was ready a
poll after its temperature, and the demo prints on each new value. Without
those lines the counts match. Not compared with a reference instrument.
The STM32L476 example MagSensorDemo (`exemples/sensor/mag_sensor_demo.cpp`)
reads the onboard LIS3MDL magnetometer (`src/sensors/mag_lis3mdl.cpp`) on the
same bus at 0x1E, 2.5 Hz, ultra high performance mode, 4 gauss full scale, and
prints the field on the 3 axes, its magnitude in uT and the temperature of the
device, on the ST-LINK virtual COM port in Release, with semihosting in Debug.
The scale of the device, 6842 LSB per gauss at 4 gauss, is 14.6 nT per LSB:
the driver keeps it in floating point in the calibration, the integer
Sensitivity of the data is rounded to 15 nT. `tests/sensors/run.py` runs the
driver against a register model. B-L475E-IOT01A2 run (2026-10-21), Release:
the LIS3MDL is found at 0x1E. In 29 readings at rest the field reads X -18.84
to -17.63 uT, Y -2.34 to -1.27 uT and Z -38.42 to -36.86 uT, magnitude 41.04
to 42.56 uT. While the board is moved by hand the magnitude goes from 12.29 to
120.67 uT; the demo applies no calibration. Not compared with a reference
instrument. In that run the 2 bytes read after OUT_Z_H, in a 9 byte read from
STATUS, were the X output again (same raw value on all 75 lines), so the
printed temperature was wrong. The driver now reads TEMP_OUT in its own
transfer. Second run (2026-10-21), Release, with that change: the temperature
reads 23.88 to 24.25 C in steps of 0.125 C, the 8 LSB per C of the device. At
rest in a first position 49 readings give a magnitude of 38.30 to 40.36 uT,
19 readings while the board is turned give 33.63 to 44.78 uT, then 27
readings at rest in a second position 39.41 to 40.98 uT.
The STM32L476 example DistSensorDemo (`exemples/sensor/dist_sensor_demo.cpp`)
reads the onboard VL53L0X time of flight sensor (`src/sensors/dist_vl53l0x.cpp`,
on the new `DistSensor` base, `include/sensors/dist_sensor.h`) on the same bus
at 0x29, after it drives XSHUT (PC6) high. It measures continuously at 5 Hz
with the device default timing budget, 33849 us after the ST default tuning,
and prints the distance in mm, the return signal and the ambient rates in
kcps, or the device status when the distance is not valid. The driver follows
the initialization of the ST VL53L0X API: soft reset, reference SPAD selection,
default tuning settings, VHV and phase reference calibrations. It does not do
the offset and cross talk calibrations for a cover glass. `tests/sensors/run.py`
runs it against a register model. B-L475E-IOT01A2 run (2026-10-21), Release:
the VL53L0X is found at 0x29. With a hand moved over the sensor it gives 55
valid distances from 32 to 382 mm, the return signal from 859 to 24867 kcps,
higher as the hand comes closer, and the ambient rate down to 23 kcps when the
hand covers the sensor. With nothing in front, 70 measurements end with device
status 6 and 3 with status 9, both a phase failure for the ST API, with a
signal of 312 to 734 kcps. 3 more end as valid with 0 mm and the same signal
as those without a target (585 to 617 kcps). The default limit checks of the
ST API pass them too: its sigma estimate computed from the printed rates is
about 6 mm, under its 18 mm limit. The driver now reports a complete range of
0 mm as status 0xFE (`VL53L0X_DEVICE_STATUS_ZERO_CODE`), not as a distance;
not run on the board since. Distances not compared with a reference.
The STM32L476 example NfcTagDemo (`exemples/rftag/nfc_tag_demo.cpp`) uses
the onboard M24SR64-Y dynamic NFC tag on the same bus at 0x56 through the new
`TagM24sr` RFTag facet (`include/rftag/m24sr.h`, `src/rftag/m24sr.cpp`), with
the RF disable pin PE2 held low. The driver opens an I2C session, selects the
NDEF application, and reads the capability container and the ST system file:
the NDEF file id and size, the largest read and update, the UID and the
product code. The memory of the facet is the NDEF file, a 2 byte length then
the message, so the RFTag SetNdef and GetNdef work on it. Each memory read or
write is one I2C session ended by a deselect, so a reader can reach the tag
between them. The demo prints what the tag holds, writes a text record and a
URI record, reads them back, then checks the tag every 2 seconds and prints
the message when a reader changed it. `tests/rftag/run.py` runs the driver
against a model of the chip. B-L475E-IOT01A2 run (2026-10-21): the M24SR is
found at 0x56, UID 02 84 66 95 51 AF AC, product code 0x84, NDEF file 8192
bytes. At start the tag held the 69 byte demo message, its text record and
URI record. The demo wrote the message and read it back the same. An iPhone
reads the message from the tag. A phone then wrote a 23 byte message, one text
record "Hello IOsonata.", and the next check printed it. No check found the
tag held by the phone: the log has no "Held by a reader" line.
L496 and L4S9 I2C4 use virtual index 3. The L432 has no I2C2: index 1 fails
initialization, I2C1 and I2C3 keep indices 0 and 2.

These restrictions identify unsupported modes within the hardware-validated
ports. Hardware validation does not imply that every optional mode is implemented.

## USB

The STM32L476 and STM32L496 libraries build the shared STM32 OTG controller,
`ARM/ST/src/usb_ctrlr_stm32.cpp`, with the IOsonata USB stack; see
[STM32 USB notes](../USB.md). The OTG_FS clock is the MSI at 48 MHz locked to
the LSE, PA11/PA12 AF10 and PA9 VBUS sense, as in ST's USB device examples
for the B-L475E-IOT01A. The STM32L476 examples UsbCdcLoopback and
UsbCdcPrbsTx (`exemples/usb/usb_cdc_loopback.cpp`, `usb_cdc_prbs_tx.cpp`)
select B_L475E_IOT01A; UsbCdcPrbsTx logs on the ST-LINK virtual COM port,
USART1 at 1 Mbaud. Use the USB OTG micro-AB connector, not the ST-LINK one.

B-L475E-IOT01A2 hardware run (2026-10-20), macOS host: UsbCdcPrbsTx Release
with `Python/uartprbs_rx.py` about 791 kB/s, no loss, the port opens at
once. UsbCdcLoopback with `Python/usb_cdc_loopback.py --duration 60`
538 kB/s each direction, 32289792 bytes each way, 0 errors, 0 pending, PASS.
Before 3f6b9868 a port open took up to a minute: the L4 OTG_FS core did not
report SETUP done for a request that followed a control write while the EP0
status phase bit stayed latched. USB on the STM32L496 is not tested on
hardware.

## RNG

The STM32L476 library builds the STM32 RNG, `ARM/ST/src/rng_stm32.cpp`
(`CryptoRngStm32`), clocked by the MSI at 48 MHz that SystemInit selects as
the 48 MHz clock. The STM32L475VG and STM32L476 have no AES, HASH or PKA. The
STM32L476 example CryptoTest runs `exemples/crypto/crypto_engine_test.cpp`
with the hardware RNG and the software AES, SHA-256 and P-256 engines; the
Release build prints on the ST-LINK virtual COM port at 115200 8N1, the Debug
build uses semihosting. B-L475E-IOT01A2 run (2026-10-21): 72 passed, 0
failed, the RNG sample checks, AES-128 known answers, CMAC, CCM, GCM, software
ECB/CBC decryption, 200 AES rounds against the software engine and the P-256
checks. The AES-256 and SHA-256/HMAC engine sections of the test are built only
for boards whose board.h selects such an engine (20 checks on the STM32H753).

## ADC

`AdcStm32` (`ARM/ST/include/adc_stm32.h`, `ARM/ST/src/adc_stm32.cpp`, shared
with the G4) implements the generic AdcDevice API on ADC1 of the L4 and L4+
parts: software triggered scans of up to 16 inputs read by polling,
resolution 6 to 12 bits, hardware oversampling 2 to 256 times, the internal
inputs VREFINT (0), temperature sensor (17) and VBAT (18, through the 1/3
bridge, Data is VBAT), and the pins of inputs 1 to 16, the same map on every
L4 part. Interrupt, DMA, continuous mode and differential inputs are
refused. The clock is HCLK / 2, the sampling time covers AcqTime and the
datasheet minimum of the internal inputs. Each scan begins with a conversion
whose result is not used, the workaround of ES0250 2.8.11 (a conversion
started more than 1 ms after the previous one can be wrong). The four L4
libraries build it. `tests/stm32/run_adc.py` runs the driver for the L476,
L4R5, L412, G474 and G431 in Unicorn against a model of the ADC registers that
checks the waits of RM0351 and of the datasheets. The STM32L476 example
AdcDemo (`exemples/converters/adc_demo.cpp`) prints VREFINT, the temperature
sensor, VBAT and Arduino A0 (PC5) once per second, on the ST-LINK virtual COM
port at 115200 8N1 in Release, with semihosting in Debug. B-L475E-IOT01A2 run
(2026-10-21), reference given as 3.3 V, 11 scans: VREFINT 1.202 to 1.203 V,
in its datasheet range of 1.182 to 1.232 V; temperature sensor 0.748 to
0.750 V, about 25.6 C with the typical datasheet values (0.76 V at 30 C,
2.5 mV/C), the demo does not read the factory calibration; VBAT 3.295 to
3.300 V; A0 (PC5), nothing reported connected, 1.158 V on the first scan, then
0.895 to 0.913 V. No temperature sensor outlier in these scans (ES0250
2.8.12).

## PWM

The four L4 libraries build the shared STM32 PWM driver,
`ARM/ST/src/pwm_stm32.cpp`, on the TIM output channels: the generic PWM API
with DevNo the timer DevNo (TIM2 is 7 on the L476, L496 and L4S9, 6 on the
L432), edge or center aligned, the pin alternate function in
`PwmChanCfg_t::PinOp`. See [STM32 PWM](../PWM.md). The STM32L476 example
PwmDemo (`exemples/pwm/pwm_demo.cpp`) runs TIM2 at 1 kHz on LD1 (PA5,
TIM2_CH1) and Arduino D0 (PA1, TIM2_CH2), both AF1: 50 and 25 percent for one
second, then the two duty cycles sweep in opposite directions, 1 percent
every 5 ms. Not run on hardware yet.

## Watchdog

The four L4 libraries build the shared STM32 watchdog driver,
`ARM/ST/src/wdt_stm32.cpp`, on the IWDG: the generic watchdog API, timeout up
to 32.8 s from the 32 kHz LSI (128 times longer with the LSI predivider of
the L412, L422, L4P5 and L4Q5), window, software reload channels, freeze
while a debugger halts the core. The L4 IWDG has no interrupt: no timeout
event. See [STM32 watchdog](../WDT.md). The STM32L476 example WdtDemo
(`exemples/misc/wdt_demo.cpp`) prints on the ST-LINK virtual COM port
(USART1, 115200 8N1) in Release, with semihosting in Debug: timeout 2000 ms,
ten reloads every 500 ms, then the IWDG resets the MCU and the demo starts
again. Not run on hardware yet.

## RTC

The four L4 libraries build the shared STM32 RTC driver,
`ARM/ST/src/rtc_stm32.cpp`, with `src/miscdev/rtc.cpp`: calendar in the C
runtime types, alarms A and B, wakeup timer, 32 backup registers (in TAMP on
the L412 and L422), from the LSE or the 32 kHz LSI (250 Hz with the LSI
predivider of the L412, L422, L4P5 and L4Q5). `SystemInit` starts the LSE on a
board with the low frequency crystal only when it is not ready, and does not
reset the backup domain (before 2026-10-22 it did at each boot on the L4+). See
[STM32 RTC](../RTC.md). The STM32L476 example RtcDemo
(`exemples/misc/rtc_demo.cpp`) prints on the ST-LINK virtual COM port
(USART1, 115200 8N1) in Release, with semihosting in Debug: the clock in use,
a boot count in backup register 0, the date and time every second from the
wakeup event, an alarm event each minute. Not run on hardware yet.

## Validation

Run the focused host regressions from the repository root:

```sh
python3 tests/stm32l4/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32/run_timer.py
python3 tests/stm32/run_adc.py
```

`tests/stm32l4/run.py` compiles extracted production function bodies against
small register and dispatch models, with UBSan. It covers SPI versus
extended-controller dispatch (L432, L476, L496 and L4S9), OSPI clock masks
and prescaler limits, I2C configuration refusal, and the refusal of an
instance the MCU does not have (SPI2 and I2C2 on the L432). It
also covers direct/PLL clock sources and MSI range selection in both startup
files. It does not simulate the complete I2C or SPI hardware.

`tests/stm32/run_timer.py` runs the shared timer driver: the TIM with a
register model and the LPTIM with plain memory registers, for L432, L476,
L496 (TIM) and L4S9, and checks the timer vectors of each vector file, see
[tests/stm32f0/README.md](../../../tests/stm32f0/README.md).

`tests/stm32/run_uart.py` runs the shared UART driver with the L432, L476,
L496 and L4S9 device headers and a model of the USART, DMA and RCC registers:
instance clocks, reset and kernel clock selection, vectors and NVIC numbers,
polling, interrupt and DMA transfers, the CSELR and DMAMUX1 requests, RX
errors and LPUART1 clock selection. See
[tests/stm32f0/README.md](../../../tests/stm32f0/README.md).

The changed SPI and I2C translation units compile
for L476, L496 and L4S9 with Arm GNU 14.3.1, GNU C++17, Cortex-M4 hard-float,
no exceptions/RTTI, at -O0 and -Os. The changed OSPI translation unit also
compiles for L4S9. Startup compiles for all three targets at both
optimization levels; the alternative system_stm32l4plus.c compiles for L4S9
(the L4S9 project uses system_stm32l4xx.c). These are source compilation
checks with staged headers,
not complete IOC library/application links or on-board tests.

UART hardware run: the STM32L476 examples UartLoopback
(`exemples/uart/uart_loopback.cpp`) and UartPrbsTxTest
(`exemples/uart/uart_prbs_tx.cpp`) select their board with `#define BOARD` in
`board.h`, from the pin maps of `ARM/ST/include/stm32_boards.h`. The project
builds for B_L475E_IOT01A (the default, STM32L475VG: the ST-LINK virtual
COM port, USART1, DevNo 1, PB7 RX and PB6 TX, AF7, since 2026-10-19, Release
linked with nosys; UART4, DevNo 4, on Arduino D0 PA1 and D1 PA0, AF8)
and STM32L476G_DISCO (USART2, DevNo 2, on PD5 TX and PD6 RX, AF7, also the
virtual COM port). The old `src/sysstatus.c` is retired; `syslog.cpp` and `sysstatus.h` replace it.
The STM32L476 library and these
two examples were built for Debug and Release from their `.project` and
`.cproject` settings with GNU Arm 13.2, outside IOcomposer. Every STM32L476
example selects B_L475E_IOT01A in `board.h` (2026-10-21): DfuBoot recovers on
the ST-LINK virtual COM port, with B1 held at reset to force recovery, and
UartSlipPrbsRxTest receives there.

Timer hardware run: TimerDemo (`exemples/timer/timer_demo.cpp`), in
`STM32L476/exemples/TimerDemo` for B_L475E_IOT01A and
`STM32L432/exemples/TimerDemo` for NUCLEO_L432KC. Output on the ST-LINK
virtual COM port at 1000000 8N1 in Release (USART1 PB7/PB6 on the B-L475E-IOT01A,
USART2 PA15/PA2 on the NUCLEO-L432KC), the first LED toggles on trigger 0.
`TIMER_DEMO_DEVNO` in `board.h` selects the timer, LPTIM1 by default, from
the LSE crystal; the TIM run at 100 kHz. Expected output, LPTIM1:

```
Timer 0 trigger 0: 100 ms
Count = 100 ms, T0 = 100097 us
Count = 200 ms, T0 = 100006 us
...
```

100 ms is 3277 ticks of the LSE, 100.006 ms. A four channel TIM prints
triggers 0 to 3 (100 ms, 1 s, 250 ms, 500 ms) and their periods. The
STM32L496 and STM32L4S9 TimerDemo projects keep their board files, without
UART output. The four TimerDemo projects build for Debug and Release; in an
instruction set emulator with LPTIM, TIM, RCC and UART models the L432 image
runs each of its eight timers and the L476 image LPTIM1, LPTIM2, TIM6, TIM17,
TIM5, TIM1 and TIM8 for 3.3 s, with the expected periods and no LPTIM write
order violation. The L432 TimerDemo ran on the NUCLEO-L432KC (2026-10-19):
LPTIM1 100006 us, TIM6 (DevNo 2) 100000 us. The L476 TimerDemo ran on the
B-L475E-IOT01A2 in Debug with semihosting output (2026-10-19).

B-L475E-IOT01A2 hardware run (2026-10-19), STM32L476 project on the
STM32L475VG, shared drivers: Blinky blinks LD1 and LD2 and stops on B1.
UartPrbsTxTest on the ST-LINK virtual COM port, USART1 at 1 Mbaud: polling
101380 B/s, interrupts 101390 B/s, DMA 100940 B/s, DMA with interrupts 100920
B/s, no loss after the receiver locks on the stream. UartLoopback with
interrupts and DMA: about 66 kB/s, 0 errors. The runs recorded as polling
(about 66 kB/s, 0 errors, Debug and Release) ran with DMA: board.h defined
`UART_DMA_MODE` true, which overrides the define at the top of
`uart_loopback.cpp`. The define is removed from board.h (2026-10-19); the
loopback without DMA is still to be run. The rates are 1.4 percent above the 100000 B/s of
1 Mbaud 8N1: the core clock comes from the MSI through the PLL and SystemInit
did not lock the MSI to the LSE (MSIPLLEN), the MSI kept its factory
calibration. The NUCLEO-L432KC measured 99.9 kB/s with the same code.
I2CScan (`exemples/i2c/i2c_scan.cpp`) on I2C2, the onboard sensor bus (SDA
PB11, SCL PB10), polling master at 100 kHz, finds 7 devices: 0x1E LIS3MDL,
0x29 VL53L0X, 0x2D, 0x56 NFC tag, 0x5D LPS22HB, 0x5F HTS221, 0x6A LSM6DSL,
and reads the LSM6DSL WHO_AM_I register 0x0F as 0x6A.
NvmDemo (`exemples/storage/nvm_demo.cpp`, NVM_DEMO_MEDIUM 1) on the onboard
MX25R6435F through the Nvm driver and QUADSPI (SPI DevNo 3, PE10 CLK, PE12
to PE15 IO0 to IO3, PE11 NCS, AF10), 8 MHz, single line read and page
program, polling: JEDEC id C2 28 17, the stamp written and read back, erase,
a write across a page boundary, programming clears bits only, unaligned
erase and read past the end rejected, ALL PASS (2026-10-19). After power
cycles the stamp is found and its boot count rises (boot 3). First run of
QUADSPI on the boards. In quad mode (2026-10-19): QREAD 0x6B (1-1-4, 8 dummy
cycles) and 4PP 0x38 (1-4-4) from the chip config, the Quad Enable bit
(status register 1 bit 6) set by Nvm from `NvmCfg_t.QuadEnable`, status
0x40 logged, ALL PASS, the stamp kept across power cycles. A single line
build (read 0x03, program 0x02) then reads the stamp written in quad mode
(boot 3), so the quad commands store the data where the single line ones
find it. FlashMemoryDemo
(`exemples/storage/flash_memory_demo.cpp`, legacy Flash driver, NVM_MODE 0)
on the same flash: chip erase, sectors 0,
1, 2, 30 and 40 written and read back, sector erase verified, completed
without error (2026-10-19).
SystemInit now sets MSIPLLEN after the LSE is ready when `g_McuOsc` has the
RC core oscillator and the LSE crystal (2026-10-19). On the
B-L475E-IOT01A2 UartPrbsTxTest with DMA then runs at 99560 B/s (100940
before), the core clock locked to the crystal.

STM32L432: `STM32L432/lib/ioc` has the Debug, Release, Debug_FPU and
Release_FPU configurations of the L476 library. It adds the STM32L432xx
vector table, `STM32L432/lib/src/Vector_STM32L432xx.c`, and
`ldscript/gcc_stm32l432xx.ld`: 240 KB of the 256 KB flash for the image, two
8 KB storage partitions above it, 48 KB SRAM1 as RAM and the 16 KB SRAM2 at
0x10000000 as RAM2. SystemInit runs the core at 80 MHz from the MSI PLL, with
PCLK1 and PCLK2 at 40 MHz, and starts the 32.768 kHz LSE crystal by default
(`g_McuOsc`), waiting for it to run. The I2C, SPI and timer sources leave out
the instances the L432 does not have; the code generated for the L476, L496
and L4S9 differs only by the new check in I2CInit and SPIInit.

The STM32L432 examples UartLoopback and UartPrbsTxTest select NUCLEO_L432KC
in `board.h`: the ST-LINK virtual COM port, USART2, DevNo 2, RX PA15 AF3
and TX PA2 AF7 (2026-10-19; the loopback used USART1 before). USART1, DevNo
1, is on Arduino Nano D0 PA10 RX and D1 PA9 TX, AF7.
Like the L476 examples they link `rdimon` semihosting and use the example
defaults at 1 Mbaud: interrupts without DMA for the loopback, DMA without
interrupts for PRBS TX. The modes are set at the top of the example source,
board.h does not set them. The library in its
four configurations and both
examples in Debug and Release were built from their `.project` and `.cproject`
settings with GNU Arm 13.2, outside IOcomposer; all 99 vector table entries
match the STM32L432xx interrupt list. The images ran in an instruction set
emulator with USART1, DMA1 (channels 4 and 5, CSELR request 2) and RCC models:
the loopback echoes in order with no lost character in polling, interrupt,
DMA without interrupts and DMA with interrupts, and the PRBS output matches
`Prbs8()`. This does not replace a run on the board.

The STM32L432 Blinky example (`exemples/misc/blinky.c`) selects NUCLEO_L432KC
in `board.h` and blinks LD3, PB3. The board has no user button, so the LED
blinks until reset. It links `nosys`, no semihosting, and runs without a
debugger. Debug and Release were built from the `.project` and `.cproject`
settings with xPack GNU Arm 15.2 outside IOcomposer. In an instruction set
emulator with an RCC model the image sets PB3 as output, runs the core from
the MSI PLL at 80 MHz and toggles PB3 at a fixed interval. Not tested on the
board.

For a new release-candidate run, record the project, MCU/board, revision,
toolchain and exercised modes alongside the existing hardware-validation
record. Validate changes affecting clocks, GPIO, UART or timers on the
relevant project; optional peripheral modes retain their documented limits.
