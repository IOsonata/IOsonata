# Nordic nRF5340 Audio DK devices

This note records IOsonata device-driver coverage and the application-core pin mapping for the Nordic nRF5340 Audio DK.

## Onboard devices

| Device | Interface on Audio DK | IOsonata support |
| --- | --- | --- |
| Cirrus Logic CS47L63 audio DSP | SPIM4 plus RESET/IRQ GPIO, audio on I2S | Cs47l63 control/register/block driver |
| Nordic nPM1100 PMIC | ISET GPIO | PmnPM1100 safe 100/500 mA input-current selection |
| TI INA230/INA231-compatible power monitors, four instances | TWIM1 | Ina230 calibrated shunt, bus, current and power measurements |
| microSD slot | SPIM4 | Existing generic SDCard/FatFs path |
| Onboard PDM microphone | Connected to CS47L63, then I2S to nRF5340 | Controlled through CS47L63; no separate register device |
| LEDs and buttons | GPIO | Existing GPIO/LED/button support |
| USB, NFC, ADC, I2S/PDM and Bluetooth | nRF5340 peripherals | MCU-peripheral functions, outside this board-device inventory |

The second nRF5340 on the board is the debug interface and is not an application-core peripheral.

## Application-core buses and pins

The board uses TWIM1 for the power monitors:

- SDA: P1.02
- SCL: P1.03
- VBAT monitor: address 0x44, 0.510 ohm shunt
- CS47L63 VDD1 monitor: address 0x45, 2.2 ohm shunt
- CS47L63 VDD2 monitor: address 0x41, 2.2 ohm shunt
- nRF5340 VDD2 monitor: address 0x40, 1.0 ohm shunt

The Nordic board configuration uses a 1 uA current LSB for all four monitors, 1024-sample averaging, 4.156 ms bus conversion, 4.156 ms shunt conversion and continuous shunt-plus-bus measurement.

SPIM4 is shared by the SD card and CS47L63:

- SCK: P0.08
- MOSI: P0.09
- MISO: P0.10
- SD CS: P0.11
- CS47L63 CS: P0.17
- CS47L63 RESET: P0.18, active low
- CS47L63 IRQ: P0.19, active low
- CS47L63 GPIO9: P0.20
- codec-interface select: P0.21, drive low for the onboard codec
- both onboard SPI devices are limited to 8 MHz

Audio serial pins are MCLK P0.12, SCK P0.14, LRCK P0.16, SDOUT P0.13 and SDIN P0.15. The onboard microphone is wired to the CS47L63 PDM input; the codec sends microphone samples to the nRF5340 over the audio serial port.

nPM1100 ISET is P0.23. Keep it low for the 100 mA USB input-current state and only drive it high for 500 mA after the USB host grants that current.

## Driver behavior

The CS47L63 driver implements one R/W bit plus a 31-bit register address, 32 padding clocks and 32-bit data words. ReadBlock/WriteBlock keep chip select asserted for continuous multiword transfers. Apply writes application-specific register sequences. Initialization holds RESET low, waits for the active-low boot-done IRQ after release, and verifies device ID 0x047A63.

The driver does not embed one Nordic LE Audio register profile or Cirrus DSP firmware image. Those are application/audio-profile data and can be loaded through Apply and WriteBlock; keeping them out of the device transport avoids coupling the generic driver to one audio use case. Disable and Reset hold the codec in hardware reset, so the application must reapply its selected register profile and DSP image after a subsequent Enable or Reset.

The nRF5340 application-core library project includes the three new generic drivers. Hardware validation of the Audio DK devices remains to be recorded.
