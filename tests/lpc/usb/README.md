# LPC11Uxx USB interface checks

Run `python3 tests/lpc/usb/run.py`. C and C++ builds include the production
LPC11Uxx controller header through the generic `usb/usb.h`. Checks cover
endpoint counts, transfer allocation bounds and invalid controller indices.

The capability values follow the LPC11U3x data sheet section 7.9.1.1:
https://www.nxp.com/docs/en/data-sheet/LPC11U3X.pdf

There are five endpoint numbers per direction, including control endpoint 0.
Each of endpoints 1 through 4 supports bulk, interrupt or isochronous transfers.
Packet constants are hardware allocation bounds; they do not guarantee that
all endpoints can simultaneously fit their buffers in USB SRAM.

This is the interface preparation for the controller port. The legacy
usb_lpc11uxx.c remains an empty ROM-header stub; no UsbCtrlr implementation
or application USB support is supplied by this change. No register map is
introduced. Driver code will use the existing vendor CMSIS LPC_USB type.

The current Nordic and SAM4L ports provide the implementation pattern:
generic USB owns descriptors and control requests; the controller reports
EP0 events and invokes bound data endpoint callbacks. Application USB is
independent of the built-in ROM DFU bootloader.
