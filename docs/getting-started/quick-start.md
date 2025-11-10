# Quick Start Guide

Get up and running with IOsonata in under 10 minutes.

## Prerequisites

Before you begin, ensure you have:

- ✅ A supported development board (nRF52832 DK, STM32 Nucleo, etc.)
- ✅ USB cable for flashing and debugging
- ✅ Basic C++ knowledge
- ✅ 20 minutes of time

## Installation

### Option A: Automated Setup (Recommended)

Run the installer for your platform:

=== "macOS"

    ```bash
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_macos.sh)"
    ```

=== "Linux"

    ```bash
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_linux.sh)"
    ```

=== "Windows"

    ```powershell
    # Run PowerShell as Administrator
    powershell -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_win.ps1'))"
    ```

This will install:
- ARM GCC Toolchain
- Eclipse Embedded CDT
- OpenOCD debugger
- IOsonata library and examples

### Option B: Manual Setup

See the [detailed installation guide](installation.md) for step-by-step manual setup.

## Your First Project: Blinky

### 1. Open Eclipse

Launch Eclipse Embedded CDT installed by the setup script.

### 2. Import the Blinky Example

1. Go to **File → Open Projects from File System...**
2. Click **Directory...** and navigate to:
   ```
   IOsonata/ARM/Nordic/nRF52/nRF52832/exemples/Blinky/Eclipse
   ```
3. Click **Finish**

### 3. Build the Project

1. Select the `Blinky` project in Project Explorer
2. Click the **hammer icon** 🔨 in the toolbar
3. Wait for the build to complete

You should see:
```
Finished building target: Blinky.elf
```

### 4. Connect Your Hardware

1. Connect your development board via USB
2. Verify the LED powers on

### 5. Configure the Debugger

1. Click the **bug icon** dropdown → **Debug Configurations...**
2. Expand the appropriate debug type:
   - **GDB SEGGER J-Link** (for Nordic DKs)
   - **GDB OpenOCD** (for CMSIS-DAP probes)
3. Double-click to create a new configuration
4. In the **Debugger** tab:
   - **For J-Link**: Set Device name to `nRF52832_XXAA`
   - **For OpenOCD**: Add config options:
     ```
     -f interface/cmsis-dap.cfg
     -f target/nrf52.cfg
     ```
5. Click **Debug**

### 6. See It Blink!

1. Eclipse switches to Debug perspective
2. Program pauses at `main()`
3. Click **Resume** button ▶️
4. Watch your LED blink! 🎉

## What Just Happened?

You've successfully:

1. ✅ Compiled ARM firmware from C++ source
2. ✅ Flashed it to a microcontroller
3. ✅ Debugged live embedded code
4. ✅ Controlled hardware with software

## Next Steps

### Understand the Code

Open `src/blinky.c` and examine:

```c
// Configure LED pins
IOPinCfg(s_Leds, s_NbLeds);

// Main loop
while (1) {
    IOPinClear(s_Leds[i].PortNo, s_Leds[i].PinNo);  // Turn on
    msDelay(250);
    IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);    // Turn off
    msDelay(250);
}
```

**Key concepts**:
- `IOPinCfg()` - Configure pins once at startup
- `IOPinClear()` / `IOPinSet()` - Control output state
- `msDelay()` - Simple blocking delay

### Explore More Examples

=== "Communication"

    - [UART Printf](../examples/uart.md#uart-retarget) - Print to terminal
    - [I2C Sensor](../examples/i2c.md#i2c-master) - Read from device

=== "Sensors"

    - [BME280 TPH](../examples/sensors-env.md) - Temperature/Pressure/Humidity
    - [IMU Demo](../examples/sensors-motion.md) - Motion sensing

=== "Advanced"

    - [BLE Bridge](../examples/ble-bridge.md) - Wireless communication
    - [File System](../examples/storage-fs.md) - LittleFS on flash

### Learn the Architecture

Read about the "Orchard" design philosophy:

- [Architecture Overview](../architecture/overview.md)
- [The Orchard Model](../architecture/orchard.md)
- [Core Patterns](../architecture/patterns.md)

### Create Your Own Project

Follow the [First Project Tutorial](first-project.md) to create a project from scratch.

## Troubleshooting

### Build Errors

!!! failure "undefined reference to `g_McuOsc`"
    You need to build the IOsonata library first. Import and build:
    ```
    IOsonata/ARM/Nordic/nRF52/nRF52832/lib/Eclipse
    ```

### Flash/Debug Issues

!!! failure "Could not connect to target"
    **Check**:
    - USB cable is connected
    - Correct device selected in debug config
    - No other debugger is using the probe

!!! failure "HardFault on startup"
    **Common cause**: Buffer in flash instead of RAM (Nordic nRF5x EasyDMA requirement)

    **Fix**: Ensure buffers are `static`:
    ```c
    static uint8_t buffer[256];  // ✅ Correct
    const uint8_t buffer[256];   // ❌ Wrong (in flash)
    ```

### LED Doesn't Blink

!!! failure "Program runs but LED stays off"
    **Check**:
    - LED pin number in `board.h` matches your hardware
    - LED polarity (active-low vs active-high)
    - Compile with Release config and remove debug probe

## Get Help

- 💬 [GitHub Issues](https://github.com/IOsonata/IOsonata/issues)
- 📖 [Full Documentation](../index.md)
- 📧 Email: info@i-syst.com

---

**Congratulations!** You've taken your first step beyond blinky. Ready to build something real? Explore the [examples](../examples/index.md) →
