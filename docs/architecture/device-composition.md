# Device inheritance and polymorphic composition

`DeviceIntrf` defines how a transfer is opened, selected, moved, and closed.
`Device` defines what a hardware or software device is and what behaviour it
provides.

These are separate axes:

```text
Device hierarchy       what the object is and can do
DeviceIntrf injection  how that object is reached
```

A sensor does not become an I2C sensor or an SPI sensor through inheritance. It
remains the same sensor and receives the required `DeviceIntrf` during
initialization.

Inheritance is also not required merely because several hardware technologies
implement the same behaviour. Derive only when a separate polymorphic device
behaviour exists.

## Device as the shared base

`Device` supplies state and behaviour shared by device families:

- lifecycle: `Enable`, `Disable`, `Reset`, and optional `PowerOff`;
- validity and identity;
- the selected device or endpoint address;
- the injected `DeviceIntrf`;
- timer and event integration.

Derived families add behaviour appropriate to the device type. `Sensor`, for
example, adds sampling and data update. `CryptoEngine` adds crypto operation and
completion behaviour.

## Capability branches

A branch represents a distinct usable device behaviour, not an implementation
variant and not only a code-sharing helper.

Examples include:

- `AccelSensor`;
- `GyroSensor`;
- `MagSensor`;
- `TempSensor`;
- `CipherEngine`;
- `HashEngine`;
- `KeyAgreeEngine`.

Application and library code can accept only the behaviour it needs:

```cpp
void ProcessAcceleration(AccelSensor &Sensor);
void EstablishKey(KeyAgreeEngine &Engine);
```

Any concrete device implementing that branch can be supplied.

## One physical component, several device behaviours

Some physical components combine different devices and behaviours. An IMU can
contain an accelerometer, gyroscope, magnetometer, and temperature sensor. An
environmental sensor can combine temperature, humidity, and pressure devices.
Motion processing can combine several sensor or fusion behaviours.

IOsonata can represent those distinct behaviours through multiple inheritance.

`AgmMpu9250`, for example, inherits the accelerometer, gyroscope, magnetometer,
and temperature branches:

```cpp
class AgmMpu9250 :
	public AccelMpu9250,
	public GyroMpu9250,
	public MagMpu9250,
	public TempSensor
{
	// One physical package containing several sensor devices
};
```

The same object can then be passed through the particular device view required
by a caller:

```cpp
AgmMpu9250 Imu;

AccelSensor *pAccel = &Imu;
GyroSensor  *pGyro  = &Imu;
MagSensor   *pMag   = &Imu;
TempSensor  *pTemp  = &Imu;
Device      *pDev   = &Imu;
```

These pointers are different polymorphic views of the same composite object.
The concrete implementation coordinates the shared package, register map,
interface, power state, FIFO, and sampling.

## Why virtual inheritance is used

Capability branches eventually reach `Device` through family bases such as
`Sensor` or `CryptoEngine`. Those family bases use virtual inheritance so a
multi-capability concrete object contains one shared `Device` subobject rather
than one independent `Device` for every branch.

This preserves one shared set of:

- lifecycle state;
- validity;
- device identity and address;
- `DeviceIntrf` pointer;
- timer and event state.

Without the shared virtual base, a multi-function device could contain several
conflicting `Device` bases.

## NVM is deliberately not a composite device hierarchy

Flash, EEPROM, FRAM, MRAM, RRAM, and other non-volatile memory technologies all
provide the same device behaviour to the application: store and retrieve an
array of bytes.

Their differences are properties of the memory implementation:

- transport or internal controller;
- command set;
- addressing format;
- page and erase geometry;
- write-enable and status behaviour;
- write protection;
- timing and transfer mode.

Those differences belong in `NvmCfg_t`, the injected `DeviceIntrf`, and the NVM
implementation. They do not justify separate polymorphic device branches.

One `Nvm` class therefore represents all supported memory types:

```text
Nvm
  + NvmCfg_t describes memory technology and geometry
  + DeviceIntrf selects SPI, I2C, QSPI, OSPI, or an internal adapter
  + the same Read, Write, Erase, Sync, and lifecycle behaviour
```

Do not create `FlashNvm`, `EepromNvm`, `FramNvm`, or similar subclasses when the
only difference is memory technology or command protocol. That would duplicate
one behaviour across unnecessary classes and reduce the value of the generic
configuration-driven implementation.

`NvmDiskIO` is different: it adapts an `Nvm` object to the `DiskIO` behaviour
required by filesystems. It does not represent another memory technology.

## Polymorphism is the purpose, not the default

Inheritance is useful when callers need to use an object through different
behavioural bases. It should not be added simply because implementations differ.

Use inheritance when:

- the object provides a distinct device behaviour;
- callers need polymorphic substitution through that behaviour;
- a composite physical component genuinely contains several device functions.

Do not use inheritance when:

- implementations provide the same behaviour with different commands or
  hardware technology;
- configuration and an injected interface fully describe the variation;
- a derived class would only rename or specialize the same operations.

## Composition is case by case

IOsonata uses several forms of composition where each fits:

1. **Transport composition**
   A `Device` holds an injected `DeviceIntrf` describing how it is reached.

2. **Device-behaviour composition through inheritance**
   One physical component combines several distinct polymorphic device
   behaviours, such as accelerometer, gyroscope, magnetometer, and temperature.

3. **Object composition**
   A higher-level device contains or references separate device objects when the
   hardware or algorithm is genuinely assembled from separate objects.

4. **Configuration-driven variation**
   One device class represents several hardware technologies that expose the
   same behaviour, as `Nvm` does.

These forms are not interchangeable. Use the smallest design that accurately
represents the hardware and its behaviour.

## Design review rule

Before adding a `Device`-derived class, determine:

- whether it introduces a genuinely different device behaviour;
- whether callers need a new polymorphic base;
- whether the hardware contains several independent device functions;
- whether configuration and `DeviceIntrf` injection already model the
  variation;
- which state belongs to the one physical component;
- which operations coordinate several branches.

Do not derive merely to represent a different chip, memory technology, command
set, transport, or register layout. Do not flatten a true multi-device component
into unrelated objects when its behaviours must share one physical state. Do not
move transport behaviour into the device inheritance tree.