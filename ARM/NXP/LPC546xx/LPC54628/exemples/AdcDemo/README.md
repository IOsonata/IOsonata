# LPCXpresso54628 ADC demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/converters/adc_demo.cpp` on the ADC driver
(`ARM/NXP/LPC546xx/include/adc_lpc546xx.h`, class `AdcLpc546xx`). Release
prints on the LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1; Debug prints
through semihosting.

Every second the demo converts three inputs and prints them in volt:

| Input | Signal |
| --- | --- |
| 12 | Temperature sensor (ADC_LPC546XX_INPUT_TEMP) |
| 4 | P0_16, ADC0_4, Arduino A0 |
| 5 | P0_31, ADC0_5, Arduino A1 |

The reference is VREFP on the 3.3 V supply of the board (default solder
jumper setting), 12 bits. Connect a voltage between 0 and 3.3 V to A0 or A1;
an open input reads an undefined value. The temperature sensor reads about
0.584 V at 0 C and goes down by about 2.04 mV per C (datasheet values quoted
by NXP, +/-5 C): T = (0.584 - V) / 0.00204.

Status: Debug and Release build and link, the driver passes the host register
model in `tests/lpc/lpc546xx/adc_test.cpp`. No hardware result yet.
