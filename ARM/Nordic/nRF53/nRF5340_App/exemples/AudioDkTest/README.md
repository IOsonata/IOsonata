# nRF5340 Audio DK test

This IOsonata example exercises the nRF5340 I2S peripheral and the CS47L63
codec on the nRF5340 Audio DK. It uses IOsonata drivers, not the Nordic nrfx
I2S driver.

## Source layout

The Eclipse project links `exemples/audio/i2s_full_duplex_test.cpp` as its only
application source and `src/board.h` for board configuration and pin maps.
The shared example owns the I2S, SPI, I2C, codec and monitor objects and calls
their IOsonata interfaces directly, as the other generic audio examples do.
There is no board implementation module or board callback interface.

This example uses the CS47L63 codec API. The WM8904 examples use that codec's
own configuration API; changing codec families requires adapting the codec
configuration in the example, not only changing the pin map.

## Audio configuration

- I2S master, 48 kHz, 16-bit stereo frames
- 12.288 MHz audio clock; 6.144 MHz MCLK; MCLK/LRCK ratio 128
- CS47L63 FLL target 49.152 MHz
- Continuous 1.5 kHz headphone test tone at -6 dB codec output gain
- Simultaneous capture from the single onboard PDM microphone
- Microphone report: peak and DC-corrected RMS, computed from 128 frames

The I2S frame contains two slots, but the board has only one onboard microphone.
The test reports that microphone as mono.

## Release build test

Build the IOsonata library and AudioDkTest Release configuration. Flash the
board and connect the UART at 1 Mbaud. Listen to the headphone jack and confirm
that the test tone continues while microphone peak/RMS reports appear.

The following checks were reported PASS on Release hardware during development:

- Continuous 1.5 kHz headphone playback
- Microphone capture alongside headphone playback
- Repeated reset and USB cable disconnect/reconnect playback tests
- Three in-place I2S stop/restart cycles with capture continuing afterward

These hardware results precede the generic-source extraction and architecture
correction. Rebuild and repeat the Release checks after updating.

## Optional restart test

In `src/board.h`, change:

```cpp
#define AUDIO_TEST_I2S_RESTART 0
```

to `1` to stop I2S for 100 ms every ten microphone reports, then resume
without resetting the MCU or reinitializing the codec. The normal configuration
uses `0` to leave continuous playback uninterrupted. A successful restart
prints stop and resume messages, and microphone reporting continues.

## Optional SD card probe

Define `AUDIO_TEST_SD_CARD` to enable the SD card probe. It is disabled by
default so absence of removable media does not block the audio test.
