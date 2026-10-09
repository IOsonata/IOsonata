# Buzzer and melody playback

`Buzzer` uses a caller-supplied `Pwm` object and channel. The application
initializes PWM and opens the channel using its own pin map before calling
`Buzzer::Init()`. Use a dedicated PWM device for a buzzer: frequency and
start/stop apply to all channels on that device.

`Start(frequency)` starts a continuous tone and returns immediately; zero
frequency means silence. `Stop()` silences it. `Volume(0..100)` selects
0..50 percent duty, with zero stopping the output. This is duty control,
not a calibrated or linear loudness setting. Raising volume after muting
takes effect when the next tone starts.

Existing `Play(frequency, durationMs)` and `Play(midiNote, durationMs)` calls
remain available. A nonzero duration waits and then explicitly stops; zero
starts a continuous tone. Use `uint32_t` for frequency and `uint8_t` for MIDI
to select the overload. MIDI notes are 0..127; an invalid note stops playback.

## Nonblocking melodies

`BuzzerMelody` receives an initialized `Buzzer` and a running IOsonata `Timer`.
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

BuzzerMelody melody;
melody.Init(&buzzer, &timer);
melody.Play(notes, 3, 120, 1, 20);
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

## Nordic PWM silence

The Nordic driver configures each opened pin as an output, with its inactive
GPIO level set from polarity. `Stop()` and `Disable()` disconnect PWM from
the pins and disable the peripheral, including when the bounded STOPPED wait
times out. `Start()` reconnects retained channels; `CloseChannel()` removes a
channel from subsequent starts. The application does not need to switch pins
between PWM and GPIO for silence.

See [PwmToneDemo](../ARM/Nordic/nRF52/nRF52840/exemples/PwmToneDemo/README.md)
and [validation](../tests/buzzer/README.md).
