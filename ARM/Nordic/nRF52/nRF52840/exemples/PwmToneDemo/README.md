# PWM melody example

Rebuild the nRF52840 library, then clean and rebuild the matching Debug or
Release configuration of `ioc/PwmToneDemo`.

The shared [source](../../../../../../exemples/pwm/pwm_tone_demo.cpp) plays
the chorus of James Lord Pierpont's public-domain *Jingle Bells* (1857).
The table uses named notes (C8-G8) and musical lengths such as quarter
and dotted quarter. `BuzzerMelody` converts them using the chosen tempo.
Playback uses 50% duty,
120 quarter notes per minute and 20 ms gaps between notes. A one-second
rest precedes each repeat. There are no blocking note delays in the demo.

PWM device 0/channel 0 supplies the sound. Timer device 2 (RTC2 on nRF52840)
wakes the main loop every 5 ms. The interrupt only wakes the loop; melody
processing runs in the main loop. Override `TONE_TIMER_DEVNO` when needed,
and reserve that timer and trigger 0 for the example.

Set `TONE_PORT` and `TONE_PIN` in [board.h](src/board.h). The repository's
existing default is P0.28 (BlueIO LED3), or P0.15 with `NORDIC_DK` selected.
For BLUEIO-WIZARD's transistor-driven buzzer, select P0.26. Local pin changes
remain in `board.h`; the shared example contains no board wiring.

The Nordic PWM driver now owns GPIO initialization and inactive output levels.
Stopping disconnects PWM and leaves the transistor gate low; restarting restores
the channel. The previous application GPIO workaround has been removed.

See the [Buzzer guide](../../../../../../docs/buzzer.md) for note tables,
tempo, rests, cancellation and RTOS usage, and
[tests](../../../../../../tests/buzzer/README.md) for compilation and register
checks. The earlier blocking demo was heard on hardware; the refactored
nonblocking player still needs hardware validation.
