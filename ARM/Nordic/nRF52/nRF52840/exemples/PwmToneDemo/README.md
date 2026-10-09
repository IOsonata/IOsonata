# PWM tone example

Build the nRF52840 library and then the matching Debug or Release configuration
of `ioc/PwmToneDemo`.

The shared [source](../../../../../../exemples/pwm/pwm_tone_demo.cpp) uses
PWM device 0, channel 0 at 50% duty. It repeats 440, 554, 659 and 880 Hz,
playing each for about 250 ms with 100 ms between tones and another second
between sequences. Timing uses the application's blocking delay loop.

Set `TONE_PORT` and `TONE_PIN` in [board.h](src/board.h) for your connection.
The default follows the existing BlueIO LED3 mapping, P0.28; `NORDIC_DK`
selects P0.15. These are output-pin defaults, not an onboard buzzer claim.
Use a scope to observe the waveform, or connect a passive buzzer through a
suitable driver circuit. All pin assignments remain in the application.

The source and the real nRF52840 PWM, startup and vector files compile and
link at `-O0` and `-Os` with Arm GNU 14.3.1. Both project configurations
reference their matching library directory. These are selected-source
command-line builds, not full IOC library builds. Hardware testing is pending.
