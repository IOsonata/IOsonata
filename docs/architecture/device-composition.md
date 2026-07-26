# Device inheritance and polymorphic composition

`DeviceIntrf` defines how a transfer is opened, selected, moved, and closed.
`Device` defines what a hardware or software device is and how device behaviour
is assembled.

These are separate axes:

```text
Device hierarchy       what the object is and can do
DeviceIntrf injection  how that object is reached
```

A sensor does not become an I2C sensor or an SPI sensor through inheritance. It
remains a sensor device and receives the required `DeviceIntrf` at initialization.
The same device implementation can therefore use different transports.

## Device as the shared base

`Device` supplies state and behaviour shared by device families:

- lifecycle: `Enable`, `Disable`, `Reset`, and optional `PowerOff`;
- validity and identity;
- the selected device or endpoint address;
- the injected `DeviceIntrf`;
- timer and event integration.

Derived families add behaviour appropriate to the device type. `Sensor`, for
example, adds sampling, data update, operating mode, and interrupt handling.
`CryptoEngine` adds crypto completion and engine health behaviour.

## Capability branches

A branch represents a usable device capability, not only a code-sharing helper.
Examples include:

- `AccelSensor`;
- `GyroSensor`;
- `MagSensor`;
- `TempSensor`;
- `CipherEngine`;
- `HashEngine`;
- `KeyAgreeEngine`.

Application and library code can accept the capability base it needs. It does
not need to know the concrete chip or every other capability implemented by the
same object.

```cpp
void ProcessAcceleration(AccelSensor &Sensor);
void EstablishKey(KeyAgreeEngine &Engine);
```

Any concrete device implementing the requested branch can be supplied.

## One physical device, several polymorphic views

A physical component can implement several capabilities. IOsonata represents
that directly with multiple inheritance.

`AgmMpu9250`, for example, inherits the accelerometer, gyroscope, magnetometer,
and temperature branches:

```cpp
class AgmMpu9250 :
	public AccelMpu9250,
	public GyroMpu9250,
	public MagMpu9250,
	public TempSensor
{
	// One physical MPU-9250 implementation
};
```

The same object can therefore be passed as an `AccelSensor`, `GyroSensor`,
`MagSensor`, `TempSensor`, `Sensor`, or `Device`, depending on what the caller
needs.

```cpp
AgmMpu9250 Imu;

AccelSensor *pAccel = &Imu;
GyroSensor  *pGyro  = &Imu;
MagSensor   *pMag   = &Imu;
TempSensor  *pTemp  = &Imu;
Device      *pDev   = &Imu;
```

These pointers do not identify separate physical devices. They are different
polymorphic views of one composite object.

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

Without the shared virtual base, a multi-function sensor could contain several
independent `Device` bases with conflicting interface pointers, validity, and
lifecycle state.

## Polymorphism is the purpose

Inheritance is not used merely to reuse implementation. Its main value is that
generic code can work with the narrow device behaviour it needs.

A processing algorithm requiring acceleration accepts `AccelSensor`. A storage
consumer accepts the storage branch it needs. A Bluetooth security operation can
accept `KeyAgreeEngine` or `RngEngine`. The concrete device may implement one
capability or many.

The composite object owns the relationships between its branches and coordinates
shared hardware behaviour. The caller sees only the selected base interface.

## Composition is case by case

IOsonata uses several forms of composition where each fits:

1. **Transport composition**
   A `Device` holds an injected `DeviceIntrf` describing how it is reached.

2. **Capability composition through inheritance**
   One concrete physical device combines several polymorphic device branches.

3. **Object composition**
   A higher-level device may contain or reference other device objects when the
   hardware is genuinely made of separate controllable parts.

These forms are not interchangeable. A capability belonging to one physical
component is often best represented as a branch of the same object. A separate
component is normally held as another object. A communication path remains a
`DeviceIntrf`, not a device capability.

## Design review rule

Before changing a `Device`-derived class, inspect:

- every base branch;
- whether the base uses virtual inheritance;
- every polymorphic view used by callers;
- which state belongs to the one physical device;
- which operations coordinate several branches;
- the injected `DeviceIntrf` and its selector semantics;
- examples using the object through different base pointers.

Do not flatten a multi-capability device into unrelated per-capability driver
objects unless the hardware actually contains independently controlled devices.
Do not move transport behaviour into the device inheritance tree.