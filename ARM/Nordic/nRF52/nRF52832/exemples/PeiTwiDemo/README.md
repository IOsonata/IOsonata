# PeiTwiDemo — nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

The example tests **TWIM1 PPI-driven error handling**, not successful
communication with an I2C peripheral. No I2C target is attached.
TWIM1 uses P0.14 (SDA) and P0.15 (SCL) with internal pull-ups.
For reliable I2C waveforms add external pull-ups (typically 4.7K
to 3.3 V) and verify that the pins are not tied to other circuitry.

## Expected hardware sequence

1. Debugger variable `g_PeiTwiStatus=1` means the routes are armed.
2. Press **BUT1** (P0.02) once. GPIOTE IN[n] -> TWIM1 STARTTX.
3. A write to address 0x7E should receive NACK because no slave is
   present. TWIM1 ERROR -> both the STOP task and **separate LED1**
   (P0.30) toggle in hardware through PPI.
4. TWIM1 STOPPED -> RGB **blue** (P0.27) toggle, also through PPI.

LED1 and RGB blue are different physical outputs. No ISR forwards
the error or completion events, and the application does not write
TWIM task or DMA registers directly.

This example deliberately arms a **single** one-byte TX transaction.
After the first press, stop the test rather than assuming the same
DMA/event configuration is ready for repeated START events. Do not
claim a pass based solely on `g_PeiTwiStatus=1`; observe both
error and STOPPED indicators. Without external pull-ups, the
bus may not meet I2C electrical timing and NACK behavior may vary.

To reuse the bus: disable and disconnect every PEI task destination,
unbind the TWIM event sources, then call
`I2CReleasePeiTransfer()` after STOPPED to release the bus
busy flag and DMA pointers.

The main I2C driver still owns target addressing, EasyDMA and bus
configuration. This test does not prove RX/TX byte accuracy or TWIS
slave behavior; those need real I2C hardware and a target-side test.
Hardware execution and nRF52 cross-variant builds remain pending.
