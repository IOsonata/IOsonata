# PWM tone example

Build the nRF52840 library and then the matching Debug or Release configuration
of `ioc/PwmToneDemo`.

The shared [source](../../../../../../exemples/pwm/pwm_tone_demo.cpp) uses
PWM device 0, channel 0 at 50% duty. It plays the 16-bar chorus of
James Lord Pierpont's public-domain *Jingle Bells* (1857), then pauses for
one second before repeating.

The melody uses C8-G8 (4186-6272 Hz), transposed high for the SMT-0540-S-R
buzzer. The note table stores frequency and duration in eighth notes.
Tempo is 120 quarter notes per minute, set by `s_EighthUs`.
Each note includes a 20 ms articulation gap so repeated notes remain distinct.
Timing uses the application's blocking delay loop. A zero-duty sample is
loaded before stopping between notes.

Set `TONE_PORT` and `TONE_PIN` in [board.h](src/board.h) for your connection.
The default follows the existing BlueIO LED3 mapping, P0.28; `NORDIC_DK`
selects P0.15. These are output-pin defaults, not an onboard buzzer claim.
Use a scope to observe the waveform, or connect a passive buzzer through a
suitable driver circuit. All pin assignments remain in the application.

The source and the real nRF52840 PWM, startup and vector files compile and
link at `-O0` and `-Os` with Arm GNU 14.3.1. Both project configurations
reference their matching library directory. These are selected-source
command-line builds, not full IOC library builds. Hardware testing is pending.
