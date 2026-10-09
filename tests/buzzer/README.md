# Buzzer and melody checks

Run the C++ tests with AddressSanitizer and UBSan:

```sh
make -C tests/buzzer test
```

The tests use the real Buzzer and BuzzerMelody implementations with injected
PWM and timer objects. They check default state, MIDI note conversion and
bounds, named pitches, sharp/flat aliases, dotted and triplet lengths,
legacy blocking playback, volume zero, failed playback, rests,
articulation, tempo, finite/infinite repeat, cancellation, millisecond counter
wrap, oversized gaps and delayed application processing.

Build the nRF52840 example and the real PWM/GPIO register fixture:

```sh
python3 tests/buzzer/build_arm.py --mdk /path/to/nordic/mdk \
  --cmsis ARM/CMSIS/Core/Include --tool-prefix arm-none-eabi-
python3 tests/buzzer/pwm_emu.py \
  tests/buzzer/build/Debug/pwm_nrf52840_test.elf \
  tests/buzzer/build/Release/pwm_nrf52840_test.elf
```

The emulator requires `unicorn` and `pyelftools`. It checks both output
polarities, GPIO configuration, stop/disable, channel retention on restart,
closed-channel exclusion and the stop-timeout fallback. It does not model
sound, transistor behavior or the PWM waveform.

The host suite passed with ASan/UBSan. Arm GNU 14.3.1 compiled and linked
both example configurations; both PWM register fixtures passed with and
without a simulated STOPPED event. The updated PWM source also compiled
for nRF52832, nRF52840, nRF5340 application core, nRF9160, nRF9120,
nRF54L15, nRF54LM20A and nRF54LM20B. These are selected-source checks,
not full IOC library builds. The refactored melody player needs a hardware run.
