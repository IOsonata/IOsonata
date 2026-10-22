# H753 USB CDC PRBS diagnostics

Rebuild the STM32H753 library and UsbCdcPrbsTx in the same configuration.
UART diagnostics are enabled in both Debug and Release on this diagnostic branch.

Connect a terminal to the ST-LINK virtual COM port at 1000000 baud, 8 data bits,
no parity, 1 stop bit, no flow control. USART3 uses PD8 TX and PD9 RX through the
existing board VCP mapping. Keep ST-LINK connected and use the native USB
connector CN13 for the CDC test. Remove USB breakpoints before running.

The startup banner confirms the UART logger is initialized. The controller
queues bus-event diagnostics through AppEvtHandler; SysLog formatting and UART
output execute in AppRun, not in the USB interrupt. The UART uses a TX FIFO and
interrupts. AppCheckStatus retries any log bytes the UART could not yet accept.

The `irq` field is the masked GINTSTS snapshot captured at interrupt entry:
- 00000004: OTG event
- 00000800: suspend
- 00001000: USB reset
- 00002000: reset/speed detection complete (ENUMDNE)
- 40000000: session request
- 80000000: wakeup

Bits can be combined. Packet-only interrupts are not logged. `dropped` counts
diagnostic events refused by the application queue. Lines marked `now` contain
live register values when the queued diagnostic runs, not an interrupt-time
snapshot. No FIFO-pop register is read by diagnostics.

Capture the banner and connected register dump, then unplug and reconnect CN13
once and capture the resulting bus-event lines. This trace does not establish
successful enumeration or hardware validation.

The no-sleep diagnostic enumerated on hardware. Normal AppWait/WFE is now
restored. USB2 initialization keeps USB2OTGFSLPEN enabled and clears
USB2OTGFSULPILPEN for the embedded FS PHY. Enumeration with this sleep-clock
fix still needs hardware confirmation.
