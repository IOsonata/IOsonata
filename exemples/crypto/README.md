# Generic CryptoTest

Import the MCU library project and its CryptoTest project into IOcomposer.
Rebuild the library and application in matching Debug or Release configurations.
The nine configured examples (nRF52832, nRF52840, nRF5340 application/network, nRF9160, nRF91x1,
STM32H753, STM32WBA65 and STM32L476) link the same `crypto_engine_test.cpp`.

The test declares `Crypto s_Crypto` and calls `s_Crypto.Init()`. The MCU library
binds its internal hardware implementation; `board.h` supplies console configuration only. Software objects in the test are independent comparison references.
No hardware factory names, target headers or linker forcing options are needed.

Release uses the configured UART; check the board's baud rate and pins. Debug
may use semihosting. Expect zero failures. AES-128/256 ECB/CBC/CTR and P-256
private-key import are required: the library uses software when an accelerator lacks an operation.
Only secure-entropy-dependent operations can be skipped on targets without
secure entropy. Busy or initialization
failures are failures, not capability skips. RNG sample tests do not certify
entropy quality.

Host validation:

```
python3 tests/crypto/run_objects.py
python3 tests/crypto/run_aes256.py
python3 tests/crypto/run_pka_models.py
python3 tests/crypto/run_ccrng.py
python3 tests/crypto/run_ccintrf.py
python3 tests/crypto/check_targets.py
python3 tests/stm32wba/run_crypto_link.py
python3 tests/stm32h7/run_crypto_accel.py
python3 tests/stm32h7/run_crypto_accel.py --l4s9
```

The first builds an actual software archive at O0/O2 and checks direct global/local construction,
object identity, independent RNG state, disabled-object rejection and the shared known-answer tests.
The AES-256 differential test compares software, an AES-128-only inherited
engine and hardware override inheritance against OpenSSL at O0/O2 under ASan/UBSan.
It also verifies that busy and failed operations are returned without retry.
The target check compiles target initialization definitions against real driver headers;
small MCU header shims omit register definitions. It is not an MCU build.
The H753 test exercises production H753 AES/HASH drivers against an OpenSSL-backed
register model, including initialization failure/retry and shared-object independence, with ASan/UBSan.
Hardware compilation and board tests remain required.

The PKA host test links the production Nordic App-core initialization and CC3xx
implementation as an archive. OpenSSL models register arithmetic; the driver
runs its actual point operations. The same test exercises BA414EP through an
injected register model. It checks hardware CC3xx import/agreement, inherited signing,
boundary scalars, invalid peers, key consumption, busy handling, RNG rebinding,
and a CC3xx cleanup timeout. Both optimization levels use ASan/UBSan.

Port review after the App-core import failure:

| Port | Software behavior and hardware overrides | Validation available |
|---|---|---|
| CC3xx: nRF52840, nRF5340 App, nRF91 | Hardware public-key import/key generation/ECDH; software ECDSA | Audio DK App core, 2026-10-25: mixed-coordinate optimization `4fdd11e` passed 101, failed 0, skipped 0; other targets retain host-model coverage |
| BA414EP: nRF54 | Software import/ECDSA; hardware key generation/ECDH | Host model; board rerun pending |
| CryptoMaster: nRF54 | Hardware AES-128/256 encryption/decryption; shared CMAC/CCM/GCM composition | DMA register model; board rerun pending |
| nRF5340 Net ECB | Inherits software modes and AES-256 | Audio DK, 2026-10-24: 101 passed, 0 failed, 0 skipped after the AES/CMAC/CCM size reductions |
| STM32WBA65 | Hardware P-256; software AES-256 and SHA | Production-driver host link and initialization-failure checks; ARM emulator/board rerun pending |
| STM32H753 | Hardware AES-128/256 and one-shot hash/HMAC; software P-256 and streaming hash | Host register model; board rerun pending |
| Other configured MCU libraries | Software algorithms, hardware RNG where implemented | Software archive and target initialization checks |

The ported operations above have working implementations. Software-only entropy
is intentionally non-secure and cannot supply key generation/signing. Desktop
and the skeletal RISC-V nRF54H20 project remain outside this MCU integration.
The host models do not validate silicon timing, DMA, or physical entropy quality.

All MCU builds run the same performance section. The test declares a `Timer`,
selects the first high-frequency device with `TimerGetHighFreqDevNo()`, and
requests 1 MHz through `Timer::Init`. The timer driver selects the actual
frequency. `TickCount()` supplies 64-bit counts; `nSecond()` and `uSecond()`
perform all time conversion. Results are nanoseconds per operation (averaged
over 64 operations) and microseconds per P-256 key import/public-key calculation.
These are elapsed times, not CPU cycle counts. Timing initialization failure
fails the test. No board timing macros, DWT register access or target selection
is present in the generic test.

Host and register-model runners supply a deterministic test-only TimerInit
implementation. It exercises conversion at 32768 Hz and a 32-bit rollover;
its printed times are simulated and must not be used as performance results.
MCU projects link the existing timer driver from their MCU library.

## Checking software linkage

CryptoTest includes software reference objects intentionally, so its map cannot
prove that a normal application discards overridden software implementations.
The PKA and STM32 accelerator runners additionally link a minimal `Crypto`
application from the production target archive at `-O0` and `-Os` with section garbage
collection, without the generic test or software reference objects.

- CryptoCell: software AES, SHA-256 and PRNG symbols must be absent.
- WBA: the production drivers link without software P-256 or PRNG object files.
  Software AES-256 and SHA must remain. The host fixture exercises construction
  and initialization failure; it does not simulate accelerator arithmetic.
- H753/L4S9: software AES and PRNG symbols must be absent. Software SHA streaming
  remains required; one-shot SHA and HMAC use hardware.

The STM32 model also verifies that initializing, disabling and re-enabling one
application object does not reset the shared engines used by another object.
These are host link and register-model checks, not measurements of MCU flash
usage or proof of silicon behavior.

## nRF52832 project

Import `ARM/Nordic/nRF52/nRF52832/lib/ioc` and
`ARM/Nordic/nRF52/nRF52832/exemples/CryptoTest/ioc`. Build the library and test
with matching Debug or Release configurations. The application links the same
`exemples/crypto/crypto_engine_test.cpp` used by the other MCU projects.

Debug uses semihosting. Release uses UART0 at 1 Mbaud, 8N1, without flow control.
The default BLUEIO console mapping is TX P0.07 and RX P0.08; adjust the example's
`src/board.h` for a different board. The application uses the full nRF52832 XXAA
512 KB Flash / 64 KB RAM linker layout, with no running Bluetooth stack required.
Hardware/software crypto selection remains entirely in the MCU library.

## Recorded network-core validation

The nRF5340 Audio DK run on 2026-10-24 passed all 101 checks, including the
benchmark timer at 1 MHz. HciController also restored a bond and enabled
encryption after reset and reconnection with BleSecurityDemo on the App core.
See [crypto size measurements](../../tests/crypto/size_notes.md) for the measured
688-byte HciController flash reduction and the separate CryptoTest footprint.

CryptoCell performance changes retain the generic test and automatic selection.
AES keys are loaded once per complete operation; contiguous key/hash register
writes use one interface transfer. nRF5340 App and nRF91 random generation uses
HMAC_DRBG/SHA-256 seeded from 48 bytes of hardware entropy. It reseeds after
1024 requests or 64 KiB, whichever comes first; Reset and Disable erase its
state. Entropy or hash failure erases the requested output and invalidates the
state. First use and reseeding still pay the hardware entropy collection cost.
The raw entropy interface and hardware health checks are unchanged. The network
core and other Nordic random sources retain their existing behavior.

`run_ccrng.py` compares the production DRBG with an independent Python HMAC
reference at -O0/-Os/-O2 for both target defines, including request/byte reseeds,
reset, disable/enable, short entropy reads, busy and injected hash failures.
`run_pka_models.py` runs the production RNG with CryptoCell SHA register models
and checks minimal linkage for unwanted software AES, SHA-256 and PRNG code.
The maintainer subsequently validated the initial AES/DRBG changes on the
Audio DK App core: 101 passed, no failures or skips. Flash increased by 972
bytes and static RAM by 72 bytes. RNG at 1024 bytes improved from 2830.38 ms
to 176.09 ms; AES ECB at 16 bytes improved from 934.06 us to 738.20 us.

The next transport change overrides complete register Read/Write in Cc3xxIntrf
using word MMIO under the existing transfer lock. It avoids generic command
buffer packing while retaining the C transfer path and operation exclusion.
The Nordic wrapper waits for power-up only when transitioning from disabled.
`run_ccintrf.py` covers the production transfer paths, unaligned buffers, busy,
invalid transfers and wrapper power transitions. The Audio DK App-core run passed 101 checks with no failures or skips:
AES ECB at 16 bytes took 358.890 us, GCM at 1024 bytes took 138.228 ms and
RNG at 1024 bytes took 88.479 ms. Flash grew by 224 bytes; static RAM was
unchanged.

The shared GCM GHASH multiply now uses four 32-bit words with fixed iteration
counts and masked selection, without secret-indexed tables. `run_ghash.py`
checks it against the former byte implementation and reports isolated host
timing. The generic CryptoTest is unchanged. Rebuild both library and test
to measure this change on the MCU; see the size notes for host measurements.


The Audio DK App-core GHASH run passed 101 checks with no failures or skips.
GCM at 1024 bytes fell from 138.228 ms to 21.462 ms, with only 32 additional
flash bytes and unchanged static RAM. The software comparison also benefits
from the shared GHASH implementation.

CryptoCell key import now uses the hardware P-256 multiply when a public
key is requested, including projective blinding from the secure RNG. Import
without public output validates and stores the scalar without a multiply or
RNG draw. Key generation shares the public-key import path.

The first P-256 board run after `7c6ae5d` exposed an operand-transfer
regression (83 passed, 18 failed). Revision `d245e43` withdrew both transfer
shortcuts while retaining hardware import. The Audio DK App-core rerun then
passed 101 checks with no failures or skips. Public-key calculation remained
too slow: the mean of eight imports was 2,058,809 us in CryptoCell versus
487,297 us in software, so the hardware path was 4.23x slower.

The nRF5340 CryptoCell 312 specification states that PKA memory mappings may
change during operations, so operand access continues to read the hardware
mapping instead of deriving a physical slot. Operand writes also retain the
separate clear and write passes. The current performance pass uses the
documented auto-incrementing PKA SRAM ports, clears the SRAM write buffer when
changing read/write direction, builds scalar-selection masks and small
constants with PKA operations instead of SRAM transfers, and uses the native
ModInv operation for the final Jacobian conversion.

The regularized scalar has an implicit leading one. For loop bits 255..2 its
current prefix is below 2^254, while the P-256 order is above 2^255; therefore
the doubled accumulator is finite and cannot equal P or -P. The full
exceptional-point alternatives are retained only for the final two iterations.
This choice depends only on the public loop index. The host PKA model now
changes result mappings, enforces SRAM direction clearing and checks bounded
PKA/SRAM traffic so the failed fixed-map assumption is no longer hidden.
The Audio DK App-core board run after `4fdd11e` passed 101 tests with no
failures or skips. P-256 public-key import averaged 516,627 us over eight
operations versus 532,576 us for the software reference. The verified
CryptoTest linker map occupies 90,168 bytes of Flash and 6,440 bytes of
static RAM. See the size notes for preceding measurements and limitations.
Other target MCU builds still require separate hardware validation.

## nRF52840 project

Import `ARM/Nordic/nRF52/nRF52840/lib/ioc` and
`ARM/Nordic/nRF52/nRF52840/exemples/CryptoTest/ioc`. Rebuild the
matching Debug or Release library before building the example. This project
links the unchanged generic `exemples/crypto/crypto_engine_test.cpp` and
selects its crypto implementation inside the MCU library. The included
`src/board.h` uses the BLUEIO-WIZARD UART pin definitions.

Release uses `UART_RETARGET` and the configured UART console; Debug uses
semihosting. The maintainer's nRF52840 DK Release run passed 101 tests with no failures or skips after the CC310 DOUT and AES-256 software-fallback fixes. Other CC310 targets still require their own validation.

## nRF52832 validation

The nRF52832 BlueIO Release CryptoTest passed **101 tests, 0 failures,
0 skips** after standalone-image ECB SoftDevice SVC handling was fixed in
`6956ebe`. The existing project at
`ARM/Nordic/nRF52/nRF52832/exemples/CryptoTest/ioc` links the unchanged
generic test and uses `blueio_board.h` UART TX/RX mappings. Hardware ECB
accelerates AES-128, CMAC and CCM; P-256 uses software. The measured mean of
eight public-key imports was 246,751 us versus 246,746 us in the software
reference. No MCU linker map was supplied for this run.

## STM32L475 B-L475E-IOT01A validation

The maintainer's STM32L475 CryptoTest passed **101 tests, 0 failures,
0 skips** with a 1 MHz timer. AES ECB at 1024 bytes measured 10.418 ms
versus 10.405 ms in the software reference; P-256 import averaged
194,147 us for both the target and reference. The 1024-byte RNG draw
measured 224,703 ns. These timings are from the generic CryptoTest;
no linker map was supplied.

## nRF9160 validation

The maintainer's nRF9160 CC310 CryptoTest run passed **101 tests, 0 failures,
0 skips**, after the device-macro correction in `446eeac`. The previous
73-pass/52-fail failure was caused by CC312 register selection in the
CC310 symmetric engine. The 1024-byte AES ECB timing was 7.087 ms versus
22.715 ms for software, and P-256 public-key import averaged 360,728 us
versus 365,688 us for software.
