# PeiGroupDemo - nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

TIMER3 generates a compare event every 500 ms using its normal driver
route for periodic clearing. The example binds the *same hardware
compare register* as a second logical PEI event and connects that
event to the standalone LED1 (P0.30). Only this LED connection
belongs to a PPI channel group.

The group is initially disabled: TIMER3 runs, but LED1 does not
toggle. Press **BUT1** to trigger the group's hardware EN task.
LED1 begins toggling approximately every 500 ms. Press **BUT2**
to trigger the group's hardware DIS task; blinking stops even though
TIMER3 continues running. Press BUT1 again to re-enable.

`g_PeiGroupStatus=1` means the connections and PPI group were armed.
A negative value locates setup failure; neither is a hardware PASS.

No CPU interrupt forwards any events or toggles the LED. On teardown,
disconnect the two button->group task links before GroupDeinit;
disable any grouped route before changing group membership. The
PEI route destructor removes a channel's group bit before freeing
its hardware channel, so a recycled channel cannot be accidentally
activated by an old group.

The test uses only the onboard buttons and LED. MCU compilation,
board behavior, S132/S140, SDC and other nRF52 variant tests
are pending.
