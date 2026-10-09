# Buzzer melody and effects example

Rebuild the nRF52840 IOsonata library after changing buzzer or PWM source, then
clean and rebuild the matching Debug or Release configuration of
`ioc/BuzzerDemo`.

The shared [source](../../../../../../exemples/audio/buzzer_demo.cpp) demonstrates
the nonblocking `BuzzerPlayer` API. It plays the chorus of James Lord
Pierpont's public-domain *Jingle Bells* (1857) using named notes and musical
lengths. The application specifies notes and tempo; it does not calculate tone
frequencies or timer ticks.

Playback uses 120 quarter notes per minute, 20 ms articulation gaps and a
one-second rest before the chorus. After the melody the demo plays these named
effects for two seconds each:

1. `Chirp`
2. `Laser`
3. `Siren`
4. `Warble`
5. `Pulse`
6. `Fade`

The sequence then starts again at Jingle Bells. There are no blocking
note-duration delays in the player.

## Board configuration

PWM device 0/channel 0 supplies the electrical drive. Set `TONE_PORT` and
`TONE_PIN` in the application's [board.h](src/board.h) for the actual board
wiring. Board pin assignments do not belong in the shared audio example or the
Nordic PWM driver.

Local board pin changes belong in `board.h` and should be preserved when the
IOsonata library is rebuilt.

## Timer and main-loop processing

Timer device 2 is used by default; on nRF52840 this is RTC2. Override
`TONE_TIMER_DEVNO` in the application when another timer is required. Reserve
trigger 0 of the selected timer for this example.

The timer wakes the main loop every 5 ms. Its ISR only signals the wake-up;
`BuzzerPlayer::Process()` and PWM changes run in the main loop.

A 5 ms service interval also determines the step resolution of pitch sweeps and
volume fades. The current effect implementation stops/restarts the tone when a
pitch or volume step changes, so phase continuity is not guaranteed.

## Silence behavior

The Nordic PWM driver remains a general-purpose PWM peripheral driver. The
buzzer audio layer uses it as an output mechanism; LEDs, motors and other PWM
users continue to use the same core PWM API.

For the buzzer, `Stop()` disconnects PWM, disables the peripheral and drives
the inactive GPIO level. `Start()` reconnects retained open channels. This
keeps a transistor-driven buzzer gate inactive during rests and between effects
without an application GPIO workaround.

## What to listen for

The latest player/effect implementation still needs hardware listening tests.
Check:

- silence between notes/effects for any remaining steady background pitch;
- whether Chirp, Laser and Siren sweeps sound smooth or obviously stepped;
- whether any sweep becomes weak at one end of its pitch range;
- whether Warble and Pulse are distinct rather than sounding like accidental
  clicks;
- whether Fade decreases cleanly and the original buzzer volume returns after
  the effect;
- whether the following Jingle Bells pass starts at the expected volume.

Record the effect name and approximate point in the effect if a problem is
audible before changing the PWM implementation.

## Validation already completed

Host playback tests passed with ASan/UBSan and cover melody notation, timing,
wraparound, cancellation, repeats, failures, sweeps, fades, mute preservation,
volume restoration and playback replacement.

Selected-source nRF52840 ARM builds passed with these sizes:

| Configuration | text | data | bss |
|---|---:|---:|---:|
| Debug | 20940 | 624 | 913 |
| Release | 10828 | 624 | 913 |

Nordic PWM register emulation previously passed stop/restart, both polarities,
channel retention and the STOPPED timeout fallback. These are selected-source
and register tests, not full IOC library builds or hardware acoustic results.

See the [Buzzer User Guide](../../../../../../docs/buzzer.md) for notes, tempo,
custom effects, cancellation, RTOS usage and playback rules, and
[buzzer tests](../../../../../../tests/buzzer/README.md) for validation commands.
