# AppEvt RTOS notification

AppEvt keeps the default CFifo queue and nonblocking dispatch functions.
After every successful enqueue has published its complete event, it calls
`AppEvtHandlerNotify()`. The default weak definition is empty. An application
links a strong definition to integrate its RTOS; no configuration define or
runtime registration selects the implementation. Compile the override into
an application object so static archive extraction cannot omit it.

The hook must be nonblocking and safe in both interrupt and thread context.
Initialize its resources before enabling event producers. The short enqueue
critical section preserves the caller's interrupt state and prevents a
preemptive consumer from seeing an incomplete event. Notification happens
after that section. This queue has one coordinated consumer on a single MCU
core; host interrupt stubs do not provide host-thread synchronization.

AppEvt does not coalesce notifications, wait, select a task, or notify from
Dispatch/Exec. The RTOS override and consumer own those decisions. Failed
queues do not notify. Consumers must account for Exec's bounded drain before
sleeping indefinitely.

UsbComboStressTaktOS provides a strong hook which gives a binary semaphore.
The semaphore retains a wake arriving during processing or before the wait;
multiple pending wakes share its token. The service task keeps its four-pass
budget and yield, and waits at most one tick (1 ms with the example's 1 kHz
tick). This application deadline also services polled USB cable and class work
which does not enqueue an AppEvt. No USB-specific wake hooks or controller
changes are required. Worker scheduling and single-byte PRBS calls are retained.

The branch is based on release_0.13 commit e140b8b and preserves that release's
removal of the AppEvt idle-handler list. Earlier USB wake experiments are not
part of this branch's new history.

Host checks cover the default weak hook, a strong override, publication before
notification, failed enqueue, and bounded dispatch without extra notifications.
The TaktOS example passes host syntax checking. Target compilation and hardware
performance are not verified. Clean rebuild IOsonata_nRF52840 and
UsbComboStressTaktOS, then run the same 200-second host workload. The previous
experiment measured 641739.47 B/s; the original TaktOS 200-second run measured
658596.19 B/s. Also check cable reconnect and traffic after idle periods.
