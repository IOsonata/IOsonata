# Pin core assignment

`IOPinAssignCore(PortNo, PinNo, CoreNo)` uses fixed, zero-based core numbers.
The main core is 0, regardless of which core calls it. Pin assignment does not
start a processor or assign a peripheral. Stop the pin's users on both cores
before changing its owner.

The nRF5340 secure App port supports App (0) and Net (1). Assigning a pin to
Net clears that pin's SPU secure permission, then sets MCUSEL. A locked secure
permission returns false. Returning a pin to App preserves its security
attribute. Non-secure App and Net callers return false without register access.

The single-core GPIO ports accept core 0 for valid port/pin numbers and reject
other cores. Existing package validation is reused where provided; board maps
must still select bonded pins. nRF54 auxiliary-core routing and ESP32-C5/C6
LP-core pad routing are not implemented and return false, including for core 0.

NetCoreStart, IpcIntrfTest and the four nRF5340 App BLE examples assign the Net console pins
before releasing the Net core. The default Audio DK map is RX P1.08, TX P1.09.
`NRF5340_DK_PCA10095` selects RX P1.00, TX P1.01 for the standard DK.
The BLE library continues to start the Net core through IPC.

Run from the repository root:

```sh
python3 tests/gpio/run_assign_core.py
```

These host checks compile the production assignment functions extracted from
their port sources, using mock registers and production validation helpers.
They check C and C++ use, valid/invalid core and pin numbers, secure permission
locks, repeated assignments, returning ownership, unrelated pin/configuration
preservation, and interrupt-state restoration. Nordic checks run at O0 and Os
with undefined-behavior sanitization. All six board mappings are compiled for
Audio DK and standard DK.

These are not full MCU builds or hardware tests. Rebuild the nRF5340 App library
and BLE example, then power-cycle the Audio DK with HciController on Net and
verify both consoles without first running NetCoreStart.

## Board-defined assignments

Define only an initializer macro in board.h:

```c
#define CORE_PINS { \
	{ 1, NET_UART_PORT, NET_UART_RX_PIN }, \
	{ 1, NET_UART_PORT, NET_UART_TX_PIN } \
}
```

The example declares storage at file scope:

```c
#ifdef CORE_PINS
static const IOPinCoreCfg_t s_CorePins[] = CORE_PINS;
#endif
```

Before starting other cores, call IOPinCoreCfg under the same CORE_PINS guard
and check its result. Each entry is ordered CoreNo, PortNo, PinNo.
Boards without assignments omit CORE_PINS; no table or call is compiled.
Single-core boards may supply entries for core 0.

The helper follows IOPinCfg's inline batch pattern. It returns true for zero
entries, false for a negative count or NULL with a positive count, and stops
on the first failed assignment. Earlier assignments remain applied.

The tests also check batch argument order, empty/invalid inputs, stopping at
each failure, and the real board initializer tables in C and C++.
