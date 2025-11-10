# Architecture Overview

IOsonata is built on object-oriented design principles that create firmware which is portable, maintainable, and surprisingly fast.

## The Core Problem

Traditional embedded development faces three challenges:

1. **Vendor Lock-In**: Code written for one MCU family doesn't work on another
2. **Tight Coupling**: Drivers are tangled with hardware registers
3. **Poor Reusability**: Copy-paste is the primary code reuse mechanism

IOsonata solves these through disciplined architecture.

## The Solution: The Orchard Model

Think of firmware as a living ecosystem, not a rigid machine.

### The Land 🌍

**What**: GPIO pins, core clocks, low-level resources

**Philosophy**: Keep it simple. Don't "objectify the dirt."

**Code**:
```cpp
// Configure pins directly
IOPinConfig(LED_PORT, LED_PIN, PINOP_GPIO,
            IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

// Control state
IOPinSet(LED_PORT, LED_PIN);    // High
IOPinClear(LED_PORT, LED_PIN);  // Low
```

### The Roots 🌱

**What**: Communication interfaces - the abstract `DeviceIntrf`

**Philosophy**: All buses (UART, I²C, SPI, BLE) speak the same language: moving bytes.

**Code**:
```cpp
class DeviceIntrf {
public:
    virtual uint32_t Rate(uint32_t hz) = 0;
    virtual int Read(uint32_t DevAddr, uint8_t* pBuff, int Len) = 0;
    virtual int Write(uint32_t DevAddr, const uint8_t* pData, int Len) = 0;
    virtual bool Enable() = 0;
    virtual void Disable() = 0;
};
```

**Impact**: Write a driver once, swap buses with one line.

### The Trees 🌳

**What**: Device objects with lifecycles

**Philosophy**: Every device has seasons - Init, Enable, Disable, Reset

**Code**:
```cpp
class Device {
public:
    virtual bool Enable() = 0;
    virtual void Disable() = 0;
    virtual void Reset() = 0;
protected:
    DeviceIntrf* vpIntrf;  // The "soil"
    Timer* vpTimer;        // The "rhythm"
};
```

**Inheritance hierarchy**:
```
Device (trunk)
├── Sensor (branch)
│   ├── TempSensor
│   ├── AccelSensor
│   └── TphBme280 (composite: Temp+Press+Humi)
├── Display
│   └── LcdHX8357
└── PowerMgnt
    └── PmNpm1300
```

### The Fruit 🍎

**What**: Living instances that produce useful data

**Code**:
```cpp
// Create the fruit
TempBme280 sensor;

// Graft it to the soil
I2C i2c;
i2c.Init(i2c_config);
sensor.Init(sensor_config, &i2c);

// Harvest the juice
float temp = sensor.ReadTemperature();
```

## Key Design Patterns

### 1. Dependency Injection

Dependencies are "injected" at initialization, not hardcoded.

```cpp
// ❌ Bad: Hardcoded dependency
class Sensor {
    I2C i2c;  // Always uses I2C - not portable
};

// ✅ Good: Injected dependency
class Sensor {
    DeviceIntrf* vpIntrf;  // Any bus works

    bool Init(Config cfg, DeviceIntrf* intf) {
        vpIntrf = intf;  // Injected at runtime
    }
};
```

### 2. Interface Segregation

Small, focused interfaces instead of monolithic ones.

```cpp
// Each device promises only what it can deliver
class TempSensor : public Sensor {
    virtual float ReadTemperature() = 0;
};

class PressSensor : public Sensor {
    virtual float ReadPressure() = 0;
};

// Multi-function device inherits multiple promises
class TphBme280 : public TempSensor,
                  public PressSensor,
                  public HumiSensor {
    // Fulfills all three promises
};
```

### 3. Static Allocation

No heap, no malloc, predictable memory.

```cpp
// Memory allocated at compile time
static uint8_t s_UartRxBuff[256];
static uint8_t s_UartTxBuff[256];

UARTCfg_t cfg = {
    .pRxMem = s_UartRxBuff,
    .RxMemSize = sizeof(s_UartRxBuff),
    // ...
};
```

### 4. Composition Over Inheritance

Build complex objects by composing simpler ones.

```cpp
// IMU "has-a" Accelerometer, Gyroscope, Magnetometer
class Imu {
private:
    AccelSensor* vpAccel;
    GyroSensor* vpGyro;
    MagSensor* vpMag;
public:
    void Init(AccelSensor* a, GyroSensor* g, MagSensor* m) {
        vpAccel = a;
        vpGyro = g;
        vpMag = m;
    }

    void Read(ImuQuat_t& quat) {
        // Fuses data from all three sensors
    }
};
```

## Portability Without Compromise

### Same Code, Different Hardware

**Application code** stays identical:

```cpp
TempBme280 sensor;
sensor.Init(cfg, &bus);
float temp = sensor.ReadTemperature();
```

**Only the bus initialization changes**:

=== "Nordic nRF52 (I²C)"

    ```cpp
    I2C i2c;
    I2CCfg_t cfg = {
        .DevNo = 0,
        .Type = I2CTYPE_STANDARD,
        .Rate = 100000,
        .pIOPinMap = s_I2cPins,
        .NbIOPins = 2
    };
    i2c.Init(cfg);
    ```

=== "STM32F4 (I²C)"

    ```cpp
    I2C i2c;
    I2CCfg_t cfg = {
        .DevNo = 0,
        .Type = I2CTYPE_STANDARD,
        .Rate = 100000,
        .pIOPinMap = s_I2cPins,  // Different pins
        .NbIOPins = 2
    };
    i2c.Init(cfg);
    ```

=== "Same Sensor on SPI"

    ```cpp
    SPI spi;
    SPICfg_t cfg = {
        .DevNo = 0,
        .Rate = 1000000,
        .pIOPinMap = s_SpiPins,
        .NbIOPins = 4
    };
    spi.Init(cfg);
    ```

**The sensor code never changes.**

## Performance

### Zero-Cost Abstractions

C++ features used by IOsonata have **zero runtime overhead**:

- **Inline functions**: Compiler removes call overhead
- **Templates**: Compile-time code generation
- **Virtual calls**: Only where polymorphism is needed

### Benchmarks

UART PRBS throughput test (higher is better):

| Framework | Language | nRF54L15 | nRF52832 @ 2 MBaud |
|-----------|----------|----------|--------------------|
| **IOsonata** | C++ OOD | 102.2 KB/s | 203 KB/s |
| Zephyr | C | 82.9 KB/s | Not supported |
| nrfx | C | 87.0 KB/s | 183.7 KB/s |

**Key insight**: Disciplined C++ is as fast or faster than C.

## Memory Footprint

Example firmware sizes (ARM Cortex-M4, -Os):

| Application | Flash | RAM |
|-------------|-------|-----|
| Blinky (GPIO only) | 2.1 KB | 0.5 KB |
| UART printf | 8.4 KB | 1.2 KB |
| BLE Temperature | 78 KB | 12 KB |
| Full Thingy:52 | 145 KB | 24 KB |

**Includes** framework overhead - competitive with bare-metal C.

## What This Buys You

✅ **Write sensor driver once** - works on any bus, any MCU

✅ **Swap I²C ↔ SPI** - one line change, zero driver edits

✅ **Test on desktop** - same code runs on Linux/macOS for rapid iteration

✅ **Layer protocols** - SLIP over UART or BLE transparently

✅ **Predictable memory** - all static, no heap fragmentation

✅ **C and C++ coexist** - use C for simple code, C++ for drivers

## Next Steps

- **Learn the patterns**: [Core Patterns](patterns.md)
- **See it in action**: [Examples](../examples/index.md)
- **Deep dive**: Read "Beyond Blinky" book
- **Build drivers**: [API Reference](../api/overview.md)

---

> "C is the language of the chip. C++ is C plus enhancements—by definition, it remains the language of the chip."
>
> The outcomes you care about—performance, predictability, maintainability—flow from architectural discipline, not language choice.
