# Examples

IOsonata includes over 60 working examples demonstrating everything from basic GPIO to complex sensor fusion. All examples are production-quality code you can use as templates for your projects.

## Learning Path

### Level 1: Foundations (Start Here)

Master the basics before moving to complex examples.

| Example | What You Learn | Path |
|---------|----------------|------|
| **Blinky** | GPIO control, board config | `exemples/misc/blinky.c` |
| **UART Retarget** | Printf to terminal, UART basics | `exemples/uart/uart_retarget_demo` |
| **Timer Demo** | Timers, interrupts, callbacks | `exemples/timer/timer_demo` |
| **I2C Master** | I2C communication fundamentals | `exemples/i2c/i2c_master_demo` |

**Time**: 2-4 hours • **Prerequisites**: None

### Level 2: Real Devices

Connect real sensors and peripherals.

| Example | Hardware | Concepts | Path |
|---------|----------|----------|------|
| **BME280 TPH** | Temp/Pressure/Humidity sensor | DeviceIntrf, Sensor lifecycle | `exemples/sensor/env_tph_demo` |
| **LCD Display** | ST7789/HX8357 TFT | Display class, SPI | `exemples/display/lcd_display_demo` |
| **Flash Memory** | External SPI flash | Storage abstraction | `exemples/storage/flash_memory_demo` |

**Time**: 4-8 hours • **Prerequisites**: Level 1 complete

### Level 3: Integration

Build complete systems combining multiple components.

| Example | What You Build | Technologies | Path |
|---------|----------------|--------------|------|
| **UART-BLE Bridge** | Wireless serial port | BLE, protocol bridging | `ARM/Nordic/.../UartBleBridge` |
| **IMU Fusion** | Orientation tracking | Multi-sensor composition | `exemples/sensor/mot_sensor_demo` |
| **File System** | Data logging | LittleFS, flash, storage | `exemples/storage/flash_littlefs_demo` |

**Time**: 8-16 hours • **Prerequisites**: Level 2 complete

## By Category

### Communication

UART, I2C, SPI, BLE - the roots of your orchard.

<div class="grid cards" markdown>

-   **UART Examples**

    ---

    - [UART PRBS TX/RX](uart.md#prbs-test) - Throughput benchmarking
    - [UART Retarget](uart.md#retarget) - Printf to terminal
    - [UART BLE Bridge](uart.md#ble-bridge) - Wireless serial

    [:octicons-arrow-right-24: UART Examples](uart.md)

-   **I²C Examples**

    ---

    - [I2C Master](i2c.md#master) - Basic I2C communication
    - [I2C Slave](i2c.md#slave) - Device implementation
    - [Multi-Master](i2c.md#multi) - Bus arbitration

    [:octicons-arrow-right-24: I²C Examples](i2c.md)

-   **SPI Examples**

    ---

    - [SPI Master](spi.md#master) - High-speed transfers
    - [SPI Slave](spi.md#slave) - Peripheral mode
    - [SPI BLE Bridge](spi.md#ble) - Wireless SPI

    [:octicons-arrow-right-24: SPI Examples](spi.md)

-   **BLE Examples**

    ---

    - [BLE Advertiser](../examples/uart.md#ble-advertiser) - Broadcasting
    - [BLE Central](../examples/uart.md#ble-central) - Scanning/connecting
    - [Custom Service](../examples/uart.md#custom-service) - GATT services

    [:octicons-arrow-right-24: BLE Examples](uart.md#bluetooth)

</div>

### Sensors

Environmental, motion, and composite sensor examples.

<div class="grid cards" markdown>

-   **Environmental Sensors**

    ---

    - BME280/BME680 - Temp/Pressure/Humidity/Gas
    - MS8607 - High-accuracy TPH
    - Gas sensors - Air quality

    [:octicons-arrow-right-24: Environmental](sensors-env.md)

-   **Motion Sensors**

    ---

    - IMU - 9-axis sensor fusion
    - Accelerometer - Impact detection
    - Gyroscope - Rotation sensing

    [:octicons-arrow-right-24: Motion](sensors-motion.md)

-   **Multi-Sensor**

    ---

    - Thingy:52 - Complete IoT device
    - SlimeVR - VR body tracking
    - Sensor fusion examples

    [:octicons-arrow-right-24: Multi-Sensor](sensors-multi.md)

</div>

### Display & UI

Text, graphics, and touch interfaces.

| Example | Display Type | Features | Difficulty |
|---------|--------------|----------|------------|
| Console Display | TFT LCD | Text output, fonts | ⭐⭐ |
| Graphics Demo | TFT LCD | Pixels, shapes, bitmaps | ⭐⭐⭐ |
| LVGL GUI | TFT + Touch | Modern UI framework | ⭐⭐⭐⭐ |
| Dual Display | 2× TFT | Multi-screen coordination | ⭐⭐⭐⭐ |

[:octicons-arrow-right-24: Display Examples](display-lcd.md)

### Storage & File Systems

Flash memory, EEPROM, file systems.

| Example | Storage | File System | Use Case |
|---------|---------|-------------|----------|
| EEPROM Demo | I2C EEPROM | None | Configuration storage |
| Flash Memory | SPI Flash | None | Raw block access |
| Flash + FatFS | SPI Flash | FAT32 | PC-compatible logging |
| Flash + LittleFS | SPI Flash | LittleFS | Wear-leveling, resilience |

[:octicons-arrow-right-24: Storage Examples](storage-flash.md)

## By Platform

### Nordic nRF52832 (62 examples)

The most extensive example collection. Perfect for learning.

**Highlights**:
- Complete Thingy:52 firmware
- BLE mesh examples
- DFU (firmware update) examples
- Multiple IDE support (Eclipse, IAR, Keil)

**Browse**: `ARM/Nordic/nRF52/nRF52832/exemples/`

### Nordic nRF52840 (61 examples)

Nearly identical to nRF52832, with USB support.

**Unique examples**:
- USB CDC (serial over USB)
- USB HID (keyboard/mouse)

**Browse**: `ARM/Nordic/nRF52/nRF52840/exemples/`

### STM32 (Limited)

Basic examples for porting reference.

**Available**:
- Blinky variants
- UART examples
- Basic sensor demos

**Browse**: `ARM/ST/STM32*/exemples/`

### Cross-Platform (`exemples/`)

Portable examples that work everywhere.

**Advantage**: Same code builds for Nordic, STM32, NXP, desktop.

**Browse**: `exemples/`

## Quick Reference

### Find by Sensor Part Number

| Sensor | Type | Example Path |
|--------|------|-------------|
| BME280 | Temp/Press/Humi | `exemples/sensor/env_tph_demo` |
| BME680 | TPH + Gas | `exemples/sensor/env_tph_demo` |
| BMI160 | IMU | `exemples/sensor/mot_sensor_demo` |
| ICM-20948 | IMU | `exemples/sensor/mot_sensor_demo` |
| ADXL362 | Accelerometer | `ARM/Nordic/.../AccelAdxl362Demo` |
| MS8607 | TPH | `ARM/Nordic/.../Ms8607Demo` |

### Find by Communication Protocol

| Protocol | Path |
|----------|------|
| UART | `exemples/uart/*` |
| I²C | `exemples/i2c/*` |
| SPI | `exemples/spi/*` |
| BLE | `exemples/bluetooth/*` |
| USB | `ARM/Nordic/nRF52/nRF52840/exemples/Usb*` |

### Find by IDE

All examples include projects for multiple IDEs:

- **Eclipse** (recommended): `*/Eclipse/` folders
- **IAR**: `*/IAR/` folders
- **Keil**: `*/uVision/` folders
- **SEGGER ES**: `*/SEGGER/` folders

## How to Use Examples

### 1. Choose an Example

Browse by learning level, category, or sensor.

### 2. Import to IDE

**Eclipse**:
1. File → Open Projects from File System
2. Navigate to example's `Eclipse/` folder
3. Click Finish

### 3. Configure for Your Board

Edit `src/board.h`:
```c
// Change pin assignments to match YOUR hardware
#define LED_PIN    17   // Your LED pin
#define UART_TX    6    // Your UART TX pin
```

### 4. Build and Flash

1. Click hammer 🔨 to build
2. Configure debug settings
3. Click bug 🐛 to flash and debug

### 5. Learn from the Code

Every example demonstrates key concepts. Look for:

- **Init patterns**: How dependencies are injected
- **Lifecycle**: Enable/Disable/Reset usage
- **Error handling**: Return code checking
- **Configuration**: How structs are populated

## Common Patterns

### Pattern: Sensor Reading

```cpp
// 1. Create instances
I2C i2c;
TempBme280 sensor;

// 2. Initialize bus
i2c.Init(i2c_config);

// 3. Graft sensor to bus
sensor.Init(sensor_config, &i2c);

// 4. Enable and read
sensor.Enable();
if (sensor.UpdateData()) {
    float temp = sensor.ReadTemperature();
}
```

### Pattern: Display Output

```cpp
// 1. Create instances
SPI spi;
LcdHX8357 display;

// 2. Initialize bus
spi.Init(spi_config);

// 3. Graft display to bus
display.Init(display_config, &spi);

// 4. Draw
display.Clear();
display.printf("Hello!\n");
```

### Pattern: Protocol Bridge

```cpp
// 1. Create source and destination
UART uart;
BtIntrf ble;

// 2. Initialize both
uart.Init(uart_config);
ble.Init(ble_config);

// 3. Bridge in application
uint8_t buff[32];
int len = uart.Rx(buff, sizeof(buff));
if (len > 0) {
    ble.Tx(0, buff, len);
}
```

## Troubleshooting Examples

### Build Fails

!!! failure "Undefined reference errors"
    Build the library first: `ARM/Nordic/nRF52/nRF52832/lib/Eclipse`

### Hardware Not Working

!!! failure "Sensor not responding"
    **Check**:
    - I2C address in code matches sensor datasheet
    - Pull-up resistors on I2C lines (4.7kΩ typical)
    - Sensor powered (check VDD pin)
    - Correct I2C pins in board.h

### Strange Behavior

!!! failure "Works in debug, fails standalone"
    **Likely**: Using semihosting (printf over debug probe)

    **Fix**: Use UART for printf, or disable semihosting for Release build

## Next Steps

1. **Start simple**: Begin with Level 1 examples
2. **Read the code**: Don't just run it, understand it
3. **Modify**: Change timing, pins, parameters
4. **Combine**: Merge examples to build your application
5. **Contribute**: Share your examples back to the community

---

**Questions?** Check the [API Reference](../api/overview.md) or [report an issue](https://github.com/IOsonata/IOsonata/issues)
