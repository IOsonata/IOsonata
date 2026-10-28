# LPCXpresso54628 WM8904 line input to headphones

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/audio/codec_line_loopback.cpp`. `src/board.h`
gives the WM8904 codec of the board: I2C on Flexcomm 2 (P3_23 SDA, P3_24 SCL,
address 0x1A), I2S0 on Flexcomm 6 to the codec (P4_1 SCK, P4_3 WS, P4_2
data), I2S1 on Flexcomm 7 from it (P2_19 data, its SCK P2_18 and WS P2_20 on
the same clocks), MCLK on P3_11, the line input IN2L/IN2R. These are the pins
and the record source of the NXP SDK I2S record and playback examples. Output
on the LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1 in Release; the Debug
build uses semihosting, which stops the CPU while it prints: use Release to
listen.

The I2S driver (`ARM/NXP/LPC546xx/src/i2s_lpc546xx.cpp`) runs full duplex:
Flexcomm 6 master sends and makes SCK and WS, Flexcomm 7 slave receives on
them. 16 bit stereo, DMA and interrupts, 48 kHz asked: the audio PLL makes
12.288 MHz from the 12 MHz FRO, MCLK is 256 x 48 kHz. The I2S is stopped
while the codec starts (about 0.4 s for its bias and the headphone DC servo),
MCLK runs. The WM8904 driver (`src/audio/wm8904.cpp`) sets up the line input
PGAs at 0 dB, the ADCs and the headphone output at -12 dB.

Plug a sound source (a phone or a PC line output) in the Audio Line-In jack
and headphones in the Audio HP / Line-Out jack. The sound of the source is
heard on the headphones. Each second the UART shows the peak level of each
channel and the frames moved:

```
WM8904 line input to headphones
I2S 48000 Hz, MCLK 12288000 Hz, line input 2, headphones -12 dB
L -18.3 dBFS, R -19.1 dBFS, 48032 frames
L -16.9 dBFS, R -17.4 dBFS, 96064 frames
...
```

With no source the levels stay low (-60 dBFS or less). The example stops
with a message on a stream failure ("receive FIFO full", "FIFO or DMA error")
or when no frame comes for 0.5 s.

Status: Debug and Release build and link. The I2S driver passes the host
test `tests/lpc/lpc546xx/i2s_test.cpp`, the audio PLL `audpll_test.c`, the
WM8904 driver with the line input `tests/audio/run_wm8904.py`. No hardware
result yet.
