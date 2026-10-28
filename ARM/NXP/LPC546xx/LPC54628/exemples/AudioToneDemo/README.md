# LPCXpresso54628 WM8904 tone

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/audio/codec_tone.cpp`. `src/board.h` gives the
WM8904 codec of the board: I2C on Flexcomm 2 (P3_23 SDA, P3_24 SCL, address
0x1A), I2S0 on Flexcomm 6 to the codec (P4_1 SCK, P4_3 WS, P4_2 data) and MCLK
on P3_11, the pins of the NXP SDK I2S examples. Output on the LPC-Link2 VCOM,
Flexcomm 0, 1000000 baud 8N1 in Release; the Debug build uses semihosting,
which stops the CPU while it prints and gives short silences.

The I2S driver (`ARM/NXP/LPC546xx/src/i2s_lpc546xx.cpp`) is the master, 16
bit stereo, DMA and interrupts, 48 kHz asked. The audio PLL makes 12.288 MHz
from the 12 MHz FRO (N 125, M 1536, P 12), MCLK is 256 x 48 kHz and SCK 32 x
48 kHz. The WM8904 driver (`src/audio/wm8904.cpp`) sets up the headphone
output at -18 dB, SYSCLK from MCLK.

Plug headphones in the Audio HP / Line-Out jack. The tone changes every
second: 440 Hz on the left, 880 Hz on the right, 660 Hz on both, silence. The
UART shows:

```
WM8904 tone
I2S 48000 Hz, MCLK 12288000 Hz, headphones -18 dB
440 Hz left
880 Hz right
660 Hz left and right
silence
...
```

"WM8904 init failed" means no answer of the codec on I2C, or a register not
taken. "I2S stream stopped" means a DMA error.

Status: Debug and Release build and link. The I2S driver passes the host
test `tests/lpc/lpc546xx/i2s_test.cpp`, the audio PLL `audpll_test.c`, the
WM8904 driver `tests/audio/run_wm8904.py`. No hardware result yet.
