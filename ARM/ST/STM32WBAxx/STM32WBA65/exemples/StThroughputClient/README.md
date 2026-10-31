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
examples, rather than requiring STM32CubeIDE plugins. `CubeRoot` is a browsing
link excluded from compilation; the individual source links select exactly
which ST files are built. Debug and Release configurations are supplied.

## DK1 controls

USART1 is PB12 TX / PA8 RX on the first ST-LINK virtual COM port, as in the
IOsonata DK1 examples. Open it at 115200 baud, 8N1, without flow control.
Wait for initialization to finish, then send commands terminated by CR:

- `B1`: start scanning when disconnected; toggle client-to-server writes
  when connected.
- `B2`: clear ST bonds when disconnected; request security when connected.
- `B3`: retained for compatibility; ST's current handler does nothing.

These are ST's existing button actions. The small local adapter schedules
commands through the ST sequencer because serial reception can run in an IRQ.
It disables the Nucleo LED/button pins; DK1 LEDs and joystick are not used.
The forced `dk1_config.h` includes ST's configuration first, then disables
those board features consistently for all C compilation units. UART pins,
LSE, HSE tuning, power setup and the Bluetooth implementation remain ST's.

## Peer and comparison limits

Use ST's matching BLE_DataThroughput_Server on a supported board. The client
expects service 0xFE80 and characteristics 0xFE81/0xFE82/0xFE83, including ST's
control and payload protocol. It cannot subscribe to BlePrbs_SDC as supplied.
For server-to-client notification testing, start transmission using the
matching server's controls; the client's connected `B1` enables the opposite
direction and should remain off for a receive-only comparison.

The reference client requests a 7.5 ms connection interval, supports ATT MTU
251, and prefers 2M PHY. It is not the IOsonata 15 ms / MTU 247 / 60-second
PRBS benchmark. Record negotiated parameters and traffic direction with the
result. A difference between these tests alone does not establish the cause
of IOsonata's throughput deficit.

Source reference: STM32CubeWBA v1.10.0, commit
`4ccfd1526870263657ec8e0854c64cda6ed2b5da`, directory
`Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_DataThroughput_Client`.
ST source files are linked from the installed SDK and retain its licenses.

Validation: project XML and source selection checked; local command adapter
checked with host mocks. No full Arm target build, Eclipse import, or DK1
radio test has been performed for this project.
