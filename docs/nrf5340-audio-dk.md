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

The driver now includes a 48 kHz, stereo 16-bit ASP1 audio profile with FLL1/SYSCLK configuration, PDM microphone capture, analog line input, mono headphone playback, digital gain and mute, and audio-routing changes. The profile is retained and reapplied across Disable/Enable or Reset. ReadInterrupts exposes codec fault and FLL events. Advanced Halo DSP firmware images remain application data and can be loaded through WriteBlock. No proprietary firmware is embedded.

The nRF5340 application-core library project includes the three new generic drivers. Hardware validation of the Audio DK devices remains to be recorded.


The AudioDkTest application-core project exercises all four monitors,
CS47L63 48-kHz audio setup, nRF5340 I2S playback and optional SD-card
presence. Debug uses semihosting for printf; Release uses UART0 at 1 Mbaud.
Codec audio and timing have not been validated on physical hardware.

Run the IOcomposer `AudioDkTest` Debug or Release build for nRF5340
application core. Audio output runs at 48 kHz and the test periodically
prints a captured microphone word and power monitor voltage. A headphone
and powered microphone are needed for an audible/recorded check. The
microSD test does not write or format the card.
