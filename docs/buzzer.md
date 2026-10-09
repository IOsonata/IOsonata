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
static const BuzzerNote_t notes[] = {
    {440, 24}, // Quarter note
    {0,   12}, // Eighth-note rest
    {660, 48}  // Half note
};

BuzzerMelody melody;
melody.Init(&buzzer, &timer);
melody.Play(notes, 3, 120, 1, 20);
```

Durations use 24 ticks per quarter note, allowing straight and triplet rhythms.
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

## Nordic PWM silence

The Nordic driver configures each opened pin as an output, with its inactive
GPIO level set from polarity. `Stop()` and `Disable()` disconnect PWM from
the pins and disable the peripheral, including when the bounded STOPPED wait
times out. `Start()` reconnects retained channels; `CloseChannel()` removes a
channel from subsequent starts. The application does not need to switch pins
between PWM and GPIO for silence.

See [PwmToneDemo](../ARM/Nordic/nRF52/nRF52840/exemples/PwmToneDemo/README.md)
and [validation](../tests/buzzer/README.md).
