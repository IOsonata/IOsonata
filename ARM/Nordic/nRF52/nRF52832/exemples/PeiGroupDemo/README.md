# PeiGroupDemo - nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

The example demonstrates **PPI channel-group one-shot behavior** using
only BUT1, BUT2 and the physically separate LED1 (P0.30). There is no
timer, no encoder and no external wiring.

1. After boot, `g_PeiGroupStatus=1` means PPI setup succeeded.
2. Press **BUT2** first: LED1 should remain unchanged, because
   the group is initially disabled.
3. Press **BUT1**: GPIOTE/PPI triggers the group's **EN** task.
4. Press **BUT2**: two group-owned PPI routes execute together:
   the LED1 TOGGLE task and the group's **DIS** task.
5. Press BUT2 again: nothing happens. Press BUT1 to rearm the group,
   then BUT2 to toggle LED1 exactly once more.

No CPU ISR forwards events or toggles an LED. The PPI group owns two
routes from the same BUT2 hardware event; the BUT1->group.EN route
remains outside the group.

The application never enables the BUT2 event's routes through the
ordinary `pei.Enable(fireEvt)` call: doing so would bypass the initial
hardware group-disable state. Group EN/DIS is the only controller for
those two routes.

To clean up, disable and disconnect the BUT1->group.EN link first,
then release BUT2 and its grouped destinations, and finally call
`PeiGroupDeinit()` (or `pei.GroupDeinit(group)`). Releasing a PEI route
also removes its membership before that PPI channel becomes reusable.

MCU compilation and physical button/LED validation remain pending.
