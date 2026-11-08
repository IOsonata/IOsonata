# PeiPwmDemo — nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project. No semihosting
or UART output; inspect `g_PeiPwmStatus` in the debugger.

The PWM driver configures PWM0, static RAM sequence buffers and
channel 0 on RGB red (P0.28): 1 kHz, 30% duty, active low.
The source buttons and indicator outputs use GPIOTE, with event-to-task
delivery through PPI only.

| Action | Expected hardware behavior |
| --- | --- |
| After boot | `g_PeiPwmStatus = 1`, both routes armed |
| Press BUT1 (P0.02) | PWM0 SEQSTART[0] executes; RGB red generates a dim PWM waveform |
| PWM SEQSTARTED[0] | Standalone LED1 (P0.30) toggles via a GPIOTE task |
| Press BUT2 (P0.13) | PWM0 TASKS_STOP executes, ending the PWM waveform |
| PWM STOPPED | RGB blue (P0.27) toggles via a GPIOTE task |

The discrete LED1 is **not** one of the three RGB colors. The blue
channel is an event indicator, not PWM output. RGB red shows the PWM
waveform; it may hold an output state when STOP is issued, depending
on the peripheral. Judge PWM activity by the start event and subsequent
STOPPED event, not only by perceived red brightness.

The nRF52 PWM specification states that with LOOP.CNT=0, a sequence is
loaded once and PWM generation continues with its last duty value until
stopped. This test deliberately leaves LOOP.CNT=0 and does not install
automatic playback shortcuts.

Neither comparator nor GPIO ISR relays PWM events. No extra PEI
sequence or DMA buffer is allocated. Do not reconfigure PWM pins,
duty or decoder while PEI routes are connected; disconnect source and
task relationships before PWMDisable or re-initialization.

Hardware result (November 8, 2026): **functional PASS** on the nRF52832
BLYST-NANO-DK, confirmed by the maintainer. The test demonstrates the
BUT1 PWM start and BUT2 stop hardware paths, with PWM output and event
indications. No timing trace, waveform measurement or independent count
for every event was supplied.

Still pending: second sequence and multi-channel tests, NEXTSTEP decoder
mode, timer input event sources, repeated enable/release cycles, and other
nRF52 SKUs.
