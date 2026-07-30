# Device Inheritance and Polymorphic Composition

`DeviceIntrf` and `Device` model two independent questions:

```text
Device hierarchy       what the object is and can do
DeviceIntrf injection  how that object is reached
```

A sensor does not become an I2C sensor or SPI sensor through inheritance. It remains the same sensor and receives the required `DeviceIntrf` during initialization.

Inheritance is used when callers need a real behavioural base. It is not used merely because two chips have different registers, command sets or transports.

## `Device` as the shared base

`Device` supplies state and behaviour shared by device families:

- lifecycle: `Enable`, `Disable`, `Reset` and optional `PowerOff`;
- validity and identity;
- device or endpoint address;
- injected `DeviceIntrf`;
- timer and event integration.

Derived families add behaviour appropriate to the device type. `Sensor` adds sampling and data update. `CryptoEngine` adds crypto operation and completion behaviour. `DiskIO` adds block-device behaviour.

## Behavioural branches

A branch represents a distinct usable behaviour, not an implementation variant.

Examples include:

- `AccelSensor`;
- `GyroSensor`;
- `MagSensor`;
- `TempSensor`;
- `CipherEngine`;
- `HashEngine`;
- `KeyAgreeEngine`.

Application code can depend on the narrow behaviour it needs:

```cpp
void ProcessAcceleration(AccelSensor &Sensor);
void EstablishKey(KeyAgreeEngine &Engine);
```

Any concrete object implementing that branch can be supplied.

## One physical component with several behaviours

A physical component may expose several behaviours that share one register map, address, lifecycle, FIFO and internal state.

`AgmMpu9250`, for example, combines accelerometer, gyroscope, magnetometer and temperature behaviour:

```cpp
class AgmMpu9250 :
    public AccelMpu9250,
    public GyroMpu9250,
    public MagMpu9250,
    public TempSensor
{
    // One physical package with coordinated shared state
};
```

The same object can be used through different behavioural views:

```cpp
AgmMpu9250 Imu;

AccelSensor *pAccel = &Imu;
GyroSensor  *pGyro  = &Imu;
MagSensor   *pMag   = &Imu;
TempSensor  *pTemp  = &Imu;
Device      *pDev   = &Imu;
```

These pointers refer to different polymorphic views of one physical object.

## Shared-state composite versus packaged independent devices

Multiple functions in one package do not automatically justify one composite object.

Use one shared-state composite when the functions must coordinate:

- one address or register map;
- one power and reset state;
- one FIFO or interrupt source;
- one internal sampling schedule;
- one hardware operation that affects several behaviours.

Use separate objects when the package contains independent logical devices with separate addresses, register maps, initialization or lifecycle.

The LSM303AGR is a useful counterexample: its accelerometer and magnetometer are separate logical devices. Packaging them together does not require one multiple-inheritance object.

The question is not whether the functions share plastic. The question is whether they share device state that must remain consistent.

## Why virtual inheritance is used

Behavioural branches eventually reach `Device` through family bases such as `Sensor` or `CryptoEngine`. Those family bases use virtual inheritance so a multi-behaviour concrete object contains one shared `Device` subobject.

This preserves one set of:

- lifecycle state;
- validity;
- identity and address;
- `DeviceIntrf` pointer;
- timer and event state.

Without the shared virtual base, a composite object could contain several conflicting `Device` bases.

## Initialization ownership

The most-derived concrete object owns coordination of the shared physical device.

A composite implementation must ensure that:

- the shared virtual `Device` base is initialized once;
- branch initialization uses the same interface and physical identity;
- one branch does not reset hardware already configured by another branch;
- shared Enable/Disable state is not duplicated;
- shared FIFO and interrupt state remain coherent;
- overloads are explicitly forwarded when several bases expose similarly named operations.

`AgmMpu9250` demonstrates branch-specific `Init()` overloads, explicit forwarding for the different `Read()` operations and one private initialization path for the shared component.

## One generic NVM behaviour

Flash, EEPROM, FRAM, MRAM, RRAM and internal non-volatile memory adapters expose the same application behaviour: addressed persistent byte storage.

Their differences are implementation properties:

- transport or internal controller;
- command set;
- address format;
- page, sector and erase geometry;
- write-enable and status behaviour;
- write protection;
- timing and transfer mode.

Those differences belong in `NvmCfg_t`, the injected `DeviceIntrf` and the NVM implementation.

```text
Nvm
  + NvmCfg_t describes memory technology and geometry
  + DeviceIntrf selects SPI, I2C, QSPI, OSPI or an internal adapter
  + Read, Write, Erase, Sync and lifecycle remain one behaviour
```

Do not create `FlashNvm`, `EepromNvm`, `FramNvm` or similar behavioural branches when the only difference is command protocol or memory technology.

`NvmDiskIO` is different. It adapts an `Nvm` object to the block-device behaviour expected by a filesystem. That is object composition and adaptation, not another memory technology.

## Forms of composition

IOsonata uses several forms of composition where each fits.

### Transport composition

A `Device` holds an injected `DeviceIntrf` describing how the device is reached.

### Behavioural composition through inheritance

One physical component presents several distinct polymorphic behaviours that share physical state.

### Object composition

A higher-level object contains or references separate objects when the hardware or algorithm is genuinely assembled from independent parts. `NvmDiskIO` referencing an `Nvm` object is an example.

### Configuration-driven variation

One device class represents several implementations that expose the same behaviour. `Nvm` uses this form.

These forms are not interchangeable. Use the smallest model that accurately represents the hardware and the required polymorphic behaviour.

## When to derive

Use inheritance when:

- callers need substitution through a behavioural base;
- the object provides a distinct device behaviour;
- one physical component genuinely combines several shared-state behaviours.

Do not add another behavioural branch when:

- configuration and `DeviceIntrf` already describe the variation;
- a subclass would only encode another command set or register map;
- several implementations expose the same application behaviour;
- independent devices are merely packaged in one component.

A concrete chip driver still derives from the appropriate behavioural family. The rule is not “never derive for a chip.” The rule is “do not create a new behaviour or another layer of subclassing merely because the chip implementation differs.”

## Design-review checklist

Before adding or changing a `Device`-derived class, determine:

- which behaviour callers actually need;
- whether a new polymorphic base is required;
- whether the physical functions share lifecycle and state;
- whether they are independent devices despite sharing one package;
- whether configuration and interface injection already model the variation;
- who owns initialization of the shared physical component;
- which operations coordinate several branches;
- whether object composition is clearer than inheritance.

Do not move transport behaviour into the device inheritance tree. Do not split a true shared-state device into unrelated objects. Do not force independent packaged devices into one composite class.
