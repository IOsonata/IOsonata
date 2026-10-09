# PWM tone example

Build the nRF52840 library and then the matching Debug or Release configuration
of `ioc/PwmToneDemo`.

The shared [source](../../../../../../exemples/pwm/pwm_tone_demo.cpp) uses
PWM device 0, channel 0 at 50% duty. It plays eight bars of the opening
repeated rhythm from Gustav Holst's *Mars, the Bringer of War*, then pauses
for one second. This is the single-note accompaniment, not the orchestral
melody. The 5/4 pattern is three triplet eighths, two quarters, two eighths
and a quarter, at 120 quarter notes per minute.

The original G is transposed to Eb8 (4978 Hz), near the frequency the
maintainer found loudest on the SMT-0540-S-R buzzer. Change `s_ToneFreq` to
try another pitch. Each note has a 20 ms articulation gap, included in its
duration. Timing uses the application's blocking delay loop. A zero-duty
sample is loaded before stopping between notes.

Score reference: [Holst, The Planets, Op. 32](https://imslp.org/wiki/The_Planets,_Op.32_(Holst,_Gustav)).
The opening rhythm comes from Holst's public-domain composition.

Set `TONE_PORT` and `TONE_PIN` in [board.h](src/board.h) for your connection.
The default follows the existing BlueIO LED3 mapping, P0.28; `NORDIC_DK`
selects P0.15. These are output-pin defaults, not an onboard buzzer claim.
Use a scope to observe the waveform, or connect a passive buzzer through a
suitable driver circuit. All pin assignments remain in the application.

The source and the real nRF52840 PWM, startup and vector files compile and
link at `-O0` and `-Os` with Arm GNU 14.3.1. Both project configurations
reference their matching library directory. These are selected-source
command-line builds, not full IOC library builds. Hardware testing is pending.
