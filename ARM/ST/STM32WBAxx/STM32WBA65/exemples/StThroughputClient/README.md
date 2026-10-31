# ST throughput client in IOcomposer

This is an Eclipse managed ARM GCC application for STM32WBA65I-DK1.
It builds ST's STM32CubeWBA v1.10.0 BLE_DataThroughput_Client sources,
HAL, sequencer, host timers, host library and Basic controller library.
It does not link IOsonata. This is a separate firmware for comparing the
ST reference integration with the IOsonata integration.

## Setup

1. Install the complete STM32CubeWBA v1.10.0 package, including its drivers,
   CMSIS, utilities and STM32_WPAN middleware. A Git checkout needs the
   submodules for those dependencies at the revisions pinned by v1.10.0.
2. Put it at `~/IOcomposer/external/STM32CubeWBA`, alongside
   `~/IOcomposer/IOsonata`. Alternatively, after importing, edit the project's
   `CUBE_WBA` linked-resource path variable under Properties > Resource >
   Linked Resources > Path Variables to point at your Cube package.
3. In IOcomposer, use File > Import > General > Existing Projects into
   Workspace and select this `ioc` directory. Do not copy the project into
   the workspace: its local board files are in the adjacent `src` directory.
4. Select Release and build `StThroughputClient`. No IOsonata library build
   is required. Use the existing DK1 ST-LINK/OpenOCD setup to program it.
   The ST linker script reserves the last 16 KB of flash for ST NVM;
   this firmware does not preserve IOsonata bond-storage compatibility.

The project uses the same GNU ARM managed toolchain as the other `ioc`
examples, rather than requiring STM32CubeIDE plugins. The tracked
`ioc/.settings/org.eclipse.core.runtime.prefs` associates lowercase `.s` files
with CDT assembly sources. Keep this file: the managed builder checks the
physical SDK filenames even when the project links use `.S` names.
`CubeRoot` is a browsing
link excluded from compilation; the individual source links select exactly
which ST files are built. Debug and Release configurations are supplied.

## Automatic comparison

Use the existing BlePrbs_SDC streamer. No ST throughput server, button action,
serial command or Python payload receiver is required. Open the first DK1
ST-LINK virtual COM port at 115200 baud, 8N1, without flow control. USART1
uses PB12 TX / PA8 RX. Start the streamer before resetting this client.

The client automatically:

1. Scans for the complete advertised name `BlePrbs_SDC`.
2. Connects at 15 ms with zero latency, explicitly requests 1M PHY, and waits
   for PHY completion before requesting data length 251 / 2120 us.
3. Exchanges MTU (247), discovers the full BlueIO service and notification
   characteristic UUIDs, locates its CCCD, and subscribes.
4. Measures 60 seconds from the first notification, checking PRBS7 in firmware
   and printing status every second. UART carries results only.
5. Prints the final result, disconnects, and repeats for 60 seconds at 2M.

An incomplete trial stops the benchmark. A complete trial whose throughput
misses the target still advances to the next PHY. The targets are four and
eight notifications per 15 ms at 1M and 2M respectively; these are averages,
not measurements of individual connection events. Reset the receiver to rerun.
A setup or first-notification timeout of 30 seconds reports an aborted trial.

After startup, expect:

```text
ST PRBS comparison: BlePrbs_SDC; 60 s at 1M then 60 s at 2M
Interval=15 ms MTU=247; firmware PRBS, UART results only
Starting 60-second 1M trial; target=BlePrbs_SDC
```

## What is being compared

This is ST's integration with a replacement benchmark application, not the
unmodified interactive ST throughput demo. Startup, HAL, radio scheduling,
clock configuration, host/controller libraries, ST sequencer, ST timers and
ST AMM event storage remain from CubeWBA. No IOsonata library is linked.
The application uses a fixed 12-entry notification queue and an ST sequencer
consumer for firmware PRBS checking, rather than IOsonata CFIFO/AppEvt.

`dk1_config.h` disables Nucleo buttons/LEDs, selects one link, MTU 247,
a 4096-byte host-event FIFO and initial default 1M PHY. It enables the general
SVCCTL event callback needed for automatic GAP procedure handling. ST's host
memory-block margin, Basic controller library and native clock/power setup
remain in use. The connection request retains ST's CE length hints (0/80,
in 0.625 ms units); actual throughput must be measured on hardware.

`src/app_ble.c` is copied from STM32CubeWBA v1.10.0, commit
`4ccfd1526870263657ec8e0854c64cda6ed2b5da`, directory
`Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_DataThroughput_Client/STM32_WPAN/App`.
It retains ST's license notice. Its changes select `PrbsClientInit` and copy
both spans of host event payloads into ST's event storage with length checks.
Other ST sources are linked from the installed SDK and retain their licenses.
The original GATT demo source remains available for ST application symbol
references, but its initialization and interactive event handling are not run.

## Validation

Run the application regression test with the complete pinned SDK:

```sh
python3 tests/stm32wba/run_st_comparison.py /path/to/STM32CubeWBA
```

It compiles the real benchmark application and ST SVCCTL event router against
ST's command/event headers, with mocked commands, timer and sequencer hooks.
It checks automatic two-trial sequencing, discovery, command parameters,
notification ownership, PRBS, measurement boundaries, aborts and retries at
O0/O2 with UBSan. It does not emulate the closed ST host/controller libraries.

Eclipse Embedded CDT 2025-12 managed Debug/Release builds are checked on Linux
with Arm GNU GCC 13.2.1 and the v1.10.0 pinned dependencies. Both images include
ST startup assembly and define Reset_Handler and g_pfnVectors. ST's packed-list
pointer alignment warnings remain in app_ble.c and ble_timer.c.
The automatic benchmark still requires DK1 radio validation. Its measured
results, rather than host tests, determine the comparison with IOsonata.
