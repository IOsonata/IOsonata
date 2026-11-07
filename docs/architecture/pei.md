# Peripheral Event Interconnect (PEI)

PEI is a programmable hardware interconnect matrix. It connects peripheral
**event sources** to peripheral **task destinations**. It does not transfer data
and is not derived from `DeviceIntrf` or `Device`.

```text
Timer compare -------+
GPIO edge -----------+--> PEI interconnect matrix --> GPIO task
Peripheral event ----+                           +--> Timer task
                                                 +--> ADC sample
```

The arrows represent possible connections, not universal MCU capabilities.
A connection is created by programming the MCU's event-routing hardware.
After configuration, event delivery does not require a CPU interrupt or
software forwarding callback.

## Generic connection model

`include/coredev/pei.h` defines the C interface and a C++ wrapper. Each
`PeiConn_t` or `Pei` instance owns **one logical event-to-task connection**.
Multiple connection instances can be used to form a one-to-many or many-to-one
routing graph if the MCU supports those connections. Application code never
chooses a PPI, DPPI or EVSYS channel directly.

- `PeiConnect` reserves resources and connects one event to one task; the
  connection is initially disabled.
- `PeiEnable` and `PeiDisable` switch the connection without reallocating it.
- `PeiDisconnect` disables the connection and releases its resources.
- A connection state starts with `Handle == 0`; successful allocation assigns
  an opaque handle. The target implementation clears it on disconnect.
- Repeated enable/disable and disconnect operations are safe. Connecting an
  already-connected object fails without affecting its current connection.

`PeiCfg_t` identifies the interconnect instance (`DevNo`), one event source,
one task destination, and optional target-specific settings (`Options == 0`
selects the default). `PeiEndpoint_t` is an opaque endpoint identifier. On one
MCU it may represent a peripheral register address, on another an event
number or hardware user ID. The MCU implementation validates the identifiers.

The C++ `Pei` object owns the connection and disconnects it when destroyed;
it uses the same C functions as applications written in C. Connection storage
is caller-owned. Hardware resource tables must be statically allocated.

## MCU implementation rules

- Select only hardware-supported event/task combinations. Return failure when
  a connection cannot be represented; do not use an ISR as a substitute.
- Keep channel numbers and allocation tables in the MCU port. A logical
  connection must not expose a hardware channel or group number.
- Reserve channels already used by Bluetooth stacks, vendor libraries and
  other drivers. In particular, the nRF52 PPI mask depends on the SoftDevice
  variant and whether it is enabled; never assume all channels are free.
- Allocate and release atomically with respect to competing callers and
  interrupts, using the target's established critical-section method.
- Do not overwrite existing routes; on failed setup, release everything
  allocated during that attempt.
- Do not disable or reconfigure an event source or task peripheral when
  disconnecting the matrix connection. The peripheral's driver owns it.
- Reject cross-domain routes unless the MCU has the required hardware
  bridge between the domains.
- One-to-many and many-to-one behavior is available only when supported by
  the particular matrix and selected endpoint pair.

## Initial implementation sequence (0.14)

1. nRF52 PPI: an event-to-task connection, allocation/release, enable/disable,
   and no-ISR timer-to-GPIOTE validation on nRF52832/BLYST-NANO-DK.
2. Nordic DPPI on supported nRF53 and nRF54 devices, including publisher,
   subscriber and security-domain ownership.
3. Microchip SAMD EVSYS: adapt the existing `SamdEvsysOpen` and
   `SamdEvsysClose` implementation without breaking timer or EIC users.
4. ESP32 ETM and other interconnect hardware where present, subject to each
   port's actual capabilities and maturity.

## Validation requirements

- C and C++ compilation of the generic header.
- Allocate, enable, disable, release and reuse connections; check resource
  exhaustion and repeated lifecycle calls.
- Connect one source to two destinations and two sources to one destination
  where the target supports it.
- Verify that failed connections preserve existing hardware routes.
- On hardware, route timer compare to a GPIOTE output task with no event
  forwarding interrupt. Verify that toggling stops on disable and resumes on
  enable without reallocation.
- Repeat with Bluetooth active on the target, verifying that existing
  stack-owned channels remain untouched.

## Status

The first 0.14 step defines the generic connection interface and its intended
behavior. Hardware routing, target project links and on-board validation are
separate subsequent steps; the header alone does not provide a working PEI port.
