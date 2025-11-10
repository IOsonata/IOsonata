# IOsonata Documentation

Welcome to the IOsonata documentation - a cross-platform C++ HAL for embedded systems.

## What is IOsonata?

IOsonata is an object-oriented, portable, and high-performance C++ framework for embedded systems development. It provides:

- ✅ **Vendor-Agnostic Design** - Write once, run on Nordic, ST, NXP, Microchip, and more
- ✅ **Zero Build Complexity** - No Devicetree, no Kconfig, just standard C++ headers
- ✅ **Production-Ready** - Used in FDA-approved medical devices and industrial systems
- ✅ **High Performance** - Meets or exceeds traditional C HAL performance
- ✅ **MIT Licensed** - Commercial-friendly open source

## Quick Links

<div class="grid cards" markdown>

-   :material-clock-fast:{ .lg .middle } __Getting Started__

    ---

    Install IOsonata and build your first project in minutes

    [:octicons-arrow-right-24: Quick Start](getting-started/quick-start.md)

-   :material-book-open-variant:{ .lg .middle } __Architecture__

    ---

    Learn the "Orchard" philosophy and design patterns

    [:octicons-arrow-right-24: Learn the Philosophy](architecture/overview.md)

-   :material-code-braces:{ .lg .middle } __API Reference__

    ---

    Complete API documentation for all classes and functions

    [:octicons-arrow-right-24: Browse API](api/overview.md)

-   :material-file-code:{ .lg .middle } __Examples__

    ---

    Over 60 working examples for sensors, displays, communication

    [:octicons-arrow-right-24: See Examples](examples/index.md)

</div>

## The Philosophy

IOsonata uses an "Orchard" model for firmware architecture:

- **The Land** - GPIO pins and low-level hardware
- **The Roots** - Communication interfaces (DeviceIntrf)
- **The Trees** - Device objects with lifecycles
- **The Fruit** - The useful data you harvest

This mental model makes complex firmware easier to understand and maintain.

!!! tip "Read Beyond Blinky"
    For a deep dive into the design philosophy and architectural patterns, read the companion book **Beyond Blinky: A Living Architecture for Embedded Systems** (link coming soon).

## Featured Examples

### Swap I²C ↔ SPI with One Line

```cpp
// Initialize sensor on I2C
I2C i2c;
i2c.Init(i2c_config);
TempBme280 sensor;
sensor.Init(sensor_config, &i2c);

// Later, swap to SPI - change ONE line
SPI spi;
spi.Init(spi_config);
sensor.Init(sensor_config, &spi);  // Same sensor, different bus
```

### Cross-Platform UART

```cpp
// Same code works on nRF52, STM32, LPC, desktop
UART uart;
uart.Init(uart_config);
uart.printf("Hello from any platform!\n");
```

### Protocol Composition

```cpp
// Wrap UART with SLIP framing - transparently
UART uart;
uart.Init(uart_config);

Slip slip;
slip.Init(&uart);  // SLIP "wears" UART

// Application speaks to abstract interface
DeviceIntrf* channel = &slip;
channel->Write(0, data, len);  // Automatically framed
```

## Performance

IOsonata meets or exceeds traditional C HAL performance:

| Platform | IOsonata (C++) | Zephyr (C) | nrfx (C) |
|----------|----------------|------------|----------|
| nRF54L15 | 102.2 KB/s | 82.9 KB/s | 87.0 KB/s |
| nRF52832 @ 2 MBaud | 203 KB/s | Not supported | 183.7 KB/s |

*UART PRBS throughput benchmark - higher is better*

## Supported Platforms

### ARM Cortex-M
- **Nordic**: nRF51, nRF52, nRF53, nRF54, nRF91 (62+ examples for nRF52832)
- **STMicroelectronics**: STM32F0, F3, F4, L4 series
- **NXP**: LPC11xx, LPC17xx, LPC546xx
- **Microchip**: SAM4E, SAM4L
- **Renesas**: RE01 series

### RISC-V
- Nordic nRF54 series
- Espressif ESP32C6

### Desktop
- Linux, macOS, Windows (for rapid testing)

## Community & Support

- 📖 [GitHub Repository](https://github.com/IOsonata/IOsonata)
- 💬 [Report Issues](https://github.com/IOsonata/IOsonata/issues)
- 📧 Contact: info@i-syst.com

## License

IOsonata is released under the [MIT License](https://github.com/IOsonata/IOsonata/blob/master/LICENSE), making it free for both commercial and non-commercial use.

---

**Ready to start?** Head to the [Quick Start Guide](getting-started/quick-start.md) →
