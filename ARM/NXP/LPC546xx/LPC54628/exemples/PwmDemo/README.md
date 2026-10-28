# LPCXpresso54628 PWM demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/pwm/pwm_demo.cpp` on the SCTimer PWM driver
(`ARM/NXP/LPC546xx/src/pwm_lpc546xx.cpp`, SCT0, DevNo 0). No serial output.

| Channel | Output | Pin |
| --- | --- | --- |
| 0 | SCT0_OUT4 | P3_14, LED1 |
| 1 | SCT0_OUT6 | P2_2, LED3 |

Both LEDs are active low, the channels use PWM_POL_LOW: the duty cycle is the
time the LED is on. The demo runs at 1 kHz: LED1 at 50 percent and LED3 at 25
percent for 1 s, then LED1 ramps from off to full on and LED3 from full on to
off, 1 percent every 5 ms, and starts over. On a scope, P3_14 and P2_2 are low
for the duty cycle at the start of each 1 ms period.

Status: Debug and Release build and link, the driver passes the host register
model in `tests/lpc/lpc546xx/pwm_test.cpp`. No hardware result yet.
