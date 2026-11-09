# ESP32C6 PulseTrain

Uses the shared `exemples/misc/pulse_train_test.c` source. Pin assignments are in
`src/board.h`. This is an external GPIO test; it does not drive the devkit's
onboard addressable RGB LED.

Observe GPIO4, GPIO5, GPIO6 and GPIO7 with a scope or logic analyzer,
using a common ground. The generic pulse train toggles one pin every
500 us, in that order. With four pins, each output stays high for about
2 ms and low for about 2 ms, with edges staggered by 500 us. Software
execution adds a small delay. Several outputs can be high together.

Build the matching IOsonata library and PulseTrain managed project in IOcomposer,
using the same Debug or Release configuration. Flash the raw binary at
offset 0. No UART console or semihosting is required.

Disconnect SPI loopback jumpers and other peripheral test wiring before
running this GPIO test. Hardware validation is pending.
