# Renesas-wide shared source directory

This directory is reserved for source implementations proven compatible across
multiple supported Renesas MCU targets.

The current RA4M1 and RE01 review found common IOsonata interfaces but no
register-level GPIO, interrupt, SCI/DMAC, clock, I2C, startup or USB backend that
is safe to share at this level. Do not move target drivers here solely because
the vendor is Renesas or the devices belong to the RA family.

Use the narrowest verified scope:

- `ARM/Renesas/src`: proven across multiple Renesas families/targets.
- `ARM/Renesas/<series>/src`: proven within compatible devices in a series.
- `ARM/Renesas/<series>/<target>/src`: target-specific peripheral IP.

Common declarations may live in `ARM/Renesas/include` without implying that
the hardware implementation is common. `usb_ctrlr.h` is one such controller
ABI; RA4M1 USBFS remains target-specific.
