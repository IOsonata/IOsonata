# Buzzer User Guide

The buzzer framework is part of IOsonata's audio layer:

```cpp
#include "audio/buzzer.h"
```

`Buzzer` uses a caller-supplied `Pwm` object and channel as its electrical
output mechanism. PWM itself remains a general-purpose core peripheral; LEDs,
motors and other devices use the same PWM API without depending on the audio
layer.

Board wiring belongs to the application. Define the buzzer pin in the
application `board.h`, then use those definitions when opening the PWM
channel:

```cpp
static const PwmChanCfg_t toneChannel = {
    .Chan = 0,
    .Pol = PWM_POL_HIGH,
    .Port = TONE_PORT,
    .Pin = TONE_PIN
};

Pwm pwm;
Buzzer buzzer;

pwm.Init(pwmCfg);
pwm.OpenChannel(&toneChannel, 1);
buzzer.Init(&pwm, toneChannel.Chan);
```

Do not put board pin assignments in the generic buzzer or MCU PWM driver.
Use a dedicated PWM device for a buzzer when practical because frequency and
start/stop apply to all open channels on that PWM device.

`Start(frequency)` starts a continuous tone and returns immediately; zero
frequency means silence. `Stop()` silences it. `Volume(0..100)` selects
0..50 percent duty, with zero stopping the output. This is duty control,
not a calibrated or linear loudness setting. Actual acoustic output depends
on the buzzer, transistor stage, supply and enclosure.

Existing `Play(frequency, durationMs)` and `Play(midiNote, durationMs)` calls
remain available for older blocking use. A nonzero duration waits and then
explicitly stops; zero starts a continuous tone. Use the nonblocking player
below for melodies and application sound effects.

## Nonblocking melodies

`BuzzerPlayer` is an alias for `BuzzerMelody`. It receives an initialized `Buzzer` and a running IOsonata `Timer`.
It neither initializes nor changes that timer. A note table stays in caller-owned
memory for the whole playback and must not be modified while playing:

```cpp
using Note = BuzzerPitch;
using Length = BuzzerDuration;

static const BuzzerNote_t notes[] = {
    {Note::C5,   Length::Quarter},
    {Note::Rest, Length::Eighth},
    {Note::G5,   Length::Half}
};

BuzzerPlayer player;
player.Init(&buzzer, &timer);
player.Play(notes, 3, 120, 1, 20);
```

Write pitches as `C4`, `A4`, `Fs5` or `Bb5`; `Rest` means silence.
C4 is middle C and A4 is 440 Hz. Sharp and flat aliases such as `Cs5`
and `Db5` select the same pitch. Pitches cover C0 through G9.

Lengths include `Whole`, `Half`, `Quarter`, `Eighth`, `Sixteenth` and
`ThirtySecond`, plus dotted and triplet lengths such as `DottedQuarter`
and `EighthTriplet`. The player converts notes to frequency and musical
lengths to time; no frequency or tick calculations are needed.
`Play()` takes tempo in quarter notes per minute, a repeat count (zero means
until stopped), and an articulation gap in milliseconds. The gap is part of
the note duration, not added to it. It is shortened for very short notes.
A rest stays silent for its entire duration. Zero-length notes and zero tempo
are rejected before playback. Changing tempo means starting a new playback.

Call `Process()` regularly from one main-loop context or one RTOS worker.
It reads elapsed time from the supplied timer and advances at most one note
per call. `IsPlaying()` reports active playback; `Failed()` distinguishes a
playback failure from normal completion. `Stop()` cancels immediately and
clears the failure status. Starting another melody stops the old one first.

The library allocates no memory and uses no blocking delay in melody playback.
PWM stop may wait briefly for the end of a hardware period. Do not call these
methods from timer interrupts or concurrently from different threads. A timer
callback should only wake the application or post its event. In TaktOS, the
worker can call `Process()` after a timed wait or timer notification.

Service the player more frequently than the shortest note or gap; 5 ms is
used in the example. Delayed servicing lengthens playback rather than playing
missed notes in a burst. Note durations are rounded down to milliseconds.
The timer must remain running at its configured frequency, without resets,
and servicing must not be interrupted for a full 32-bit millisecond wrap.

## Sound effects

`BuzzerPlayer` is another name for the same `BuzzerMelody` class. Use one
player for both: every new playback stops the previous melody or effect.
Initialization and `Process()` are unchanged.

```cpp
BuzzerPlayer player;
player.Init(&buzzer, &timer);
player.Play(BuzzerEffect::Chirp);       // One chirp
// Or, in response to another application event:
player.Play(BuzzerEffect::Siren, 2000); // Repeat for two seconds
```

Available effects are `Chirp` (rising pitch), `Laser` (falling pitch and
volume), `Siren` (up/down sweep), `Warble` (alternating notes), `Pulse`
(tone/rest) and `Fade` (falling volume). The presets use high notes near
C8-G8; Laser falls to C7. Their sound and loudness depend on the buzzer.
A duration of zero plays the pattern once; a nonzero duration repeats it
until that many milliseconds have elapsed. `Stop()` cancels it immediately.

Custom effects use notes, milliseconds and optional volume endpoints:

```cpp
static const BuzzerEffectStep_t effect[] = {
    {Note::C8,   Note::G8,   300},          // Rising pitch
    {Note::G8,   Note::G8,   200, 100, 0},  // Fade out
    {Note::Rest, Note::Rest, 100}            // Silence
};
player.PlayEffect(effect, 3, 2); // Play the sequence twice; zero repeats forever
```

Equal notes hold a pitch; two rests create silence. To fade a sound to silence,
use a zero volume endpoint rather than sweeping to `Rest`. Mixed note/rest
endpoints, zero durations and volume percentages above 100 are rejected.
The step table must remain valid and unchanged while playing.

Pitch and volume interpolate linearly as `Process()` is called. This is a
stepped sweep, not an audio synthesizer: service it every 5 ms for the demo.
Changed pitch or volume restarts the tone through the existing PWM API;
there is no guarantee of phase continuity. A late call advances at most one
step, while a named effect's total duration still expires on elapsed time.

Volumes are percentages of the buzzer's volume at playback start, so a muted
buzzer stays muted. The player restores that setting when stopped, replaced,
finished or failed. Do not change the buzzer directly during playback.
PWM duty controls volume in coarse steps and is not linear in perceived
loudness. No heap allocation or blocking note delay is used.

## Service interval and sweep quality

Call `Process()` regularly from one application context. The nRF52840 example
uses a 5 ms wake interval. A shorter interval gives finer pitch and volume
steps but also causes more PWM updates; a longer interval makes sweeps more
obviously stepped. Do not call the player from the timer ISR.

Pitch and volume changes currently stop and restart the tone through the
existing PWM API. Phase continuity is therefore not guaranteed. If a sweep
sounds rough on hardware, record which effect and which part of the sweep is
objectionable before changing the PWM driver.

A late `Process()` call advances at most one melody/effect step. For named
effects started with a total duration, the elapsed-time limit still ends the
effect at the requested time.

## Nordic PWM silence

The Nordic PWM driver remains a general-purpose core peripheral driver. Its
stop behavior is useful to the buzzer audio layer but is not audio-specific.

The driver configures each opened pin as an output, with its inactive GPIO
level set from polarity. `Stop()` and `Disable()` disconnect PWM from the
pins, disable the peripheral and drive the inactive GPIO level, including when
the bounded STOPPED wait times out. `Start()` reconnects retained open
channels; `CloseChannel()` prevents a closed channel from being reconnected.

For a transistor-driven buzzer this keeps the gate inactive during silence.
Applications do not need a second GPIO workaround around every rest or stop.

## nRF52840 BuzzerDemo

The nRF52840 [BuzzerDemo](../ARM/Nordic/nRF52/nRF52840/exemples/BuzzerDemo/README.md)
uses PWM0/channel 0 and a caller-selected timer. The shared application source
is [exemples/audio/buzzer_demo.cpp](../exemples/audio/buzzer_demo.cpp). Timer
device 2 is used by default; on nRF52840 that is RTC2. The timer wakes the main
loop every 5 ms and the ISR performs no melody or PWM work.

The demo plays Jingle Bells at 120 BPM, then Chirp, Laser, Siren, Warble, Pulse
and Fade for two seconds each, then repeats. Keep `TONE_PORT` and `TONE_PIN`
in the application's `board.h`.

## Validation status

Host playback tests cover note conversion, musical lengths, tempo, rests,
repeat/cancel behavior, timer wrap, invalid input, named effects, custom pitch
sweeps, fades, mute preservation, volume restoration and playback replacement.
The latest host run passed with ASan/UBSan.

Selected-source nRF52840 ARM builds before the audio-directory move passed:

| Configuration | text | data | bss |
|---|---:|---:|---:|
| Debug | 20940 | 624 | 913 |
| Release | 10828 | 624 | 913 |

Nordic PWM register emulation previously passed stop/restart, both polarities,
channel retention and the STOPPED timeout path. These are selected-source and
register tests, not full IOC library builds or acoustic validation.

The current reusable player and named effects have not yet been listening-tested
on hardware. In particular, sweep smoothness, relative effect loudness and
silence between effects still need the maintainer's hardware result.

See [buzzer tests](../tests/buzzer/README.md) for validation commands.
