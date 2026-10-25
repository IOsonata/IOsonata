# Renesas USB controller integration

Status: **interface groundwork only**. There is currently NO functional
Renesas USB controller backend in IOsonata. Do not link the Renesas USB
stack or advertise USB support until the controller driver and hardware
tests pass.

## Architecture

The existing IOsonata generic USB stack uses `usb_ctrlr.h` selected by the
MCU library. Renesas-wide declarations are in `ARM/Renesas/include`.
Implementation will use the existing controller contract; no application
or new generic USB interface is needed.

Renesas USBFS and USBHS must be treated as distinct controller IP types.
Do not copy the STM32 OTG FIFO backend: the Renesas USB controller uses
DCP and USB pipes with PIPESEL, FIFO selection/handshakes, BRDY/BEMP/NRDY
and device state interrupts. Pipe allocation must account for IN and OUT
sharing available physical pipes.

The initial RA4M1 target provides USBFS. The proposed USBFS implementation
belongs in `ARM/Renesas/src/usbfs_ra.cpp` only if it can be reused with
compatible Renesas controllers; otherwise place it under the appropriate
series directory. USBHS is future work for targets with confirmed USBHS
register compatibility and PHY support; do not claim high-speed operation
on RA4M1.

## Required completion checklist

1. Confirm RA4M1 USB clock, PFS, VBUS, module-stop and ICU event routing.
2. Implement USBFS register initialization, connect/disconnect, reset,
   suspend/resume, SETUP handling and EP0 control/status transactions.
3. Implement endpoint-to-pipe allocation; enforce hardware pipe type and
   size restrictions. Reclaim pipes and FIFOs on close/reset.
4. Implement FIFO writes/reads and interrupt handling for BRDY, BEMP,
   NRDY, device-state events, stalls, cancellations and remote wakeup.
5. Preserve the existing IOsonata endpoint callback and queue contract;
   controller completion must occur before launching the next transfer.
6. Add only completed USB controller sources to the RA4M1 IOC/e2 library
   projects; add the generic USB sources required by the stack.
7. Add CDC loopback, PRBS, bulk, interrupt, HID and MSC examples using the
   existing generic examples and application-local board pin maps.
8. Compile Debug and Release; then test enumeration, multi-endpoint
   concurrent traffic, reset, suspend/resume, VBUS cycling, ZLP and
   EP0 stall recovery on EK-RA4M1.
9. Add USBHS only after choosing a physical target, verifying that target's
   headers and clock/PHY requirements, and repeating the regression tests.

The `usb_ctrlr.h` declaration file currently selects RA4M1 USBFS
capabilities through the existing `RA4M1` target macro. Its endpoint count
is the maximum logical address range; the implementation must enforce
the shared physical-pipe limit. ISO is deliberately disabled pending testing.
