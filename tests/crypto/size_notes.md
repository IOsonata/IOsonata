# Crypto size measurements

`python3 tests/crypto/measure_size.py` builds one direct `Crypto` object,
initializes it and runs its self-test. It excludes the generic test program's
software reference objects. Use `--root /path/to/other/checkout` to compare
revisions with the same compiler and fixture.

The following comparison used x86-64 GCC 13.3.0, `-Os`, function/data sections,
linker section garbage collection, no LTO, no RTTI and no exceptions. Baseline:
`ab8dfd9acb66824632197670c4ca655ed491274b`.

| Host measurement | Before | After | Reduction |
| --- | ---: | ---: | ---: |
| GNU size text | 86,767 | 81,273 | 5,494 bytes |
| GNU size data | 25,728 | 23,016 | 2,712 bytes |
| GNU size bss | 32 | 32 | 0 bytes |
| SHA compression stack frame | 384 | 192 | 192 bytes |
| HMAC stack frame | 416 | 320 | 96 bytes |

These are host executable section sizes and compiler-reported per-function
stack frames, not MCU flash/RAM measurements or complete call-chain stack use.
Host data includes C++ tables and relocations. Rebuild the target Release library
and application and compare their linker maps to establish device savings.

The changes preserve the public classes and automatic hardware overrides:

- Default software hash methods use the existing hash-only implementation,
  avoiding the larger SHA/HMAC object's construction tables and self-test.
- Software key-context size and alignment queries use their existing context
  type directly, without constructing an ECC engine.
- SHA-256 uses a rolling 16-word schedule instead of keeping all 64 words.
- HMAC converts its key buffer from ipad to opad in place and reuses the digest
  buffer after synchronous HashUpdate consumes it. Hardware streaming hash
  overrides still perform the digest work. Temporary secrets are still wiped.

`run_sha256.py` compares SHA-256 and HMAC against OpenSSL at `-O0`, `-Os` and
`-O2` under address/undefined-behavior sanitizers. It covers lengths 0..4096,
streaming chunks, key sizes around the block boundary, truncation, output
bounds, overlapping output, finalized-context rejection and failure at every
streaming call in HMAC.

Additional validation: `run_objects.py` and `run_ccsym_models.py` at `-O0` and
`-O2`. The software-only generic run skips ECDH/signing because its RNG is not
secure; the CryptoCell model supplies a secure RNG fixture and passes all 101
checks. These runs do not constitute hardware validation.

## nRF5340 network-core hardware result, 2026-10-24

The maintainer rebuilt HciController with the shared AES round, CMAC buffer and
CCM block-loop changes through `5bf5ac7b6258b287d6f31322305d7f05efa5bd53`.
The two supplied Release linker maps use the network core's 256 KB flash and
64 KB RAM regions.

| HciController measurement | Before batch | After batch | Reduction |
| --- | ---: | ---: | ---: |
| Flash occupied, including vectors and initialized data | 256,416 | 255,728 | 688 bytes |
| Flash remaining | 5,728 | 6,416 | |
| Static RAM, including section alignment | 61,884 | 61,884 | 0 bytes |

The earlier shared cipher-mode loop had already reduced HciController flash
from 256,720 to 256,416 bytes. Across these two measured batches, flash decreased
by 992 bytes. Earlier optimization work is outside this comparison.

The final batch's host object reduction was 1,093 bytes; this is not the target
saving. LTO, instruction selection and section layout change the result.
The target reduction is 688 bytes, entirely in text; rodata, data and bss
sizes are unchanged. Static RAM excludes the reserved heap and main stack and
does not measure maximum runtime stack consumption.

On the nRF5340 Audio DK network core, the generic CryptoTest completed with
**101 passed, 0 failed, 0 skipped**. Its timer initialized at 1 MHz. This covers
AES-128/256, CMAC, CCM, GCM, SHA-256/HMAC, secure RNG operation and P-256
known-answer, boundary and cross-engine checks. The CryptoTest map occupies
71,808 bytes of flash and 5,960 bytes of static RAM, including alignment.
CryptoTest contains software reference objects and is not a production-image
software-linkage measurement.

BleSecurityDemo on the application core, using HciController on the network
core, restored a saved bond and enabled encryption after reset and reconnection.
The supplied reconnect logs do not establish a new pairing or application-data
transfer after this batch.

The changes retain the public inheritance, automatic hardware overrides,
software support for operations absent from hardware, and existing failure
handling. An isolated experiment removing virtual Device inheritance was not
adopted because it would change shared-Device composition with other families.


## nRF5340 application-core baseline, 2026-10-24

The maintainer's CryptoTest(10).map occupies 88,784 bytes of flash and 6,372
bytes of static RAM including alignment. The accompanying generic CryptoTest
run passed 101 checks with no failures or skips and a 1 MHz benchmark timer.
This is the baseline before the CryptoCell performance changes, not a measured
result of those changes. AES ECB at 16 bytes took 934,062 ns; RNG requests of
1 and 1024 bytes took 65,014,187 ns and 2,830,375,515 ns respectively.

The subsequent changes reuse the AES key setup within an operation, combine
contiguous register writes and use an entropy-seeded HMAC_DRBG for the App-core
and nRF91 RNG. The DRBG adds 72 bytes of state per CryptoRngNrf object on those
targets. It uses the hardware SHA implementation and existing HMAC composition.
Its algorithm follows NIST SP 800-90A rev. 1 section 10.1.2:
https://nvlpubs.nist.gov/nistpubs/SpecialPublications/NIST.SP.800-90Ar1.pdf

The first draw and periodic reseeds still require fresh entropy. The following
hardware result measures this batch; runtime stack high-water usage is unknown.


## CryptoCell AES/DRBG hardware result, 2026-10-24

After `ad1129a737c84761cd2a89aa6541b386c40569a3`, the maintainer's App-core
CryptoTest run passed 101 checks with no failures or skips. Its supplied
CryptoTest(20261024-234832).map uses the same flash and RAM origins as the
previous App-core map.

| Measurement | Before | After | Change |
| --- | ---: | ---: | ---: |
| Flash occupied | 88,784 bytes | 89,756 bytes | +972 bytes |
| Static RAM including alignment | 6,372 bytes | 6,444 bytes | +72 bytes |
| AES ECB, 16 bytes | 934.062 us | 738.203 us | 21.0% less time |
| AES ECB, 1024 bytes | 24.819 ms | 19.239 ms | 22.5% less time |
| RNG, 1 byte | 65.014 ms | 15.375 ms | 4.23x faster |
| RNG, 1024 bytes | 2830.376 ms | 176.086 ms | 16.07x faster |

The 16-byte software AES comparison remained about 564 us, so the hardware
path still had excessive setup and register-transfer overhead.

The following transport optimization gives Cc3xxIntrf its own complete
Read/Write implementations. Word copies preserve unaligned caller buffers;
MMIO remains 32-bit. The framework transfer lock and the separate operation
hold remain in use. The same word-copy loops serve the C callbacks. The
Nordic wrapper no longer repeats its startup spin when already powered.

Host tests compare both production transfer paths and exercise contention,
malformed requests and power recovery. A RAM-register benchmark at -Os/-O2
shows about one third of the generic path's CPU time, under host sanitizers.
This is not an MCU speedup measurement. The maintainer subsequently supplied the App-core result below.


## CryptoCell transport hardware result, 2026-10-24

After `c628fa0c7c915fca0b6055adf8a1639727eaff54`, the Audio DK App-core
CryptoTest passed 101 checks with no failures or skips. The supplied
CryptoTest(20261025-000223).map occupies 89,980 bytes of flash and 6,444 bytes
of static RAM including alignment: 224 additional flash bytes, unchanged RAM.

| Operation | Before transport change | After |
| --- | ---: | ---: |
| AES ECB, 16 bytes | 738.203 us | 358.890 us |
| AES ECB, 1024 bytes | 19.239 ms | 10.053 ms |
| CCM, 1024 bytes | 39.304 ms | 20.613 ms |
| GCM, 1024 bytes | 147.693 ms | 138.228 ms |
| RNG, 1024 bytes | 176.086 ms | 88.479 ms |

## Shared GHASH word multiply

The generic GCM multiply now processes the 128-bit accumulator in four
32-bit words instead of two 16-byte loops per input bit. The 128-step loop,
masked selection, masked polynomial reduction and explicit temporary wiping
remain. Big-endian byte loads/stores accept unaligned buffers on either host
byte order. No lookup tables, persistent state or public API changes are added.
The two temporary arrays still occupy 32 bytes; compiled stack usage depends
on the target compiler.

`run_ghash.py` compares all 16,384 single-bit basis pairs, 4,096 mixed inputs,
zero/ones and unaligned bounds with the previous byte-serial implementation.
ASan/UBSan checks pass at -O0/-Os/-O2. Separate unsanitized host timings measured
3,804 versus 322 ns per multiply at -Os and 2,475 versus 334 ns at -O2. These
are isolated host results, not a prediction of full GCM or MCU throughput.
Host -Os AES-modes object text grows from 10,311 to 10,456 bytes (+145);
object data and BSS are unchanged. Target flash and timings remain unmeasured.

The OpenSSL CMAC/CCM/GCM differential and failure-injection suite passes at
-O0/-Os/-O2. CryptoCell, Nordic ECB, Renesas RA4, Microchip SAM and Espressif
register-model suites also exercise the shared implementation. Hardware
validation should rebuild the library and the unchanged generic CryptoTest
with matching configurations, comparing GCM timings and the linker map.


## Shared GHASH hardware result, 2026-10-24

After `196bd5504c4bcc1ec1bcb73c2b1b1d12bbbc5b10`, the Audio DK App-core
CryptoTest passed 101 checks with no failures or skips. The supplied
CryptoTest(20261025-002055).map occupies 90,012 bytes of flash (+32) and
6,444 bytes of static RAM including alignment (unchanged).

| GCM message | Before | After |
| --- | ---: | ---: |
| 16 bytes | 6.497 ms | 1.189 ms |
| 64 bytes | 12.770 ms | 2.155 ms |
| 256 bytes | 37.862 ms | 6.016 ms |
| 1024 bytes | 138.228 ms | 21.462 ms |

The 1024-byte software comparison also improved, to 47.566 ms. P-256 public
key import remained software at this point: engine 487,307 us, reference
487,306 us. Those timings do not measure the following P-256 change.

## CryptoCell public-key import and operand-transfer attempt

KeyImport overrides inherited software public-key calculation with the same
blinded P-256 multiply used for key generation. KeyGen generates a scalar and
shares KeyImport's calculation and failure handling. Context-only import needs
only range validation and storage. Public-key calculation requires a secure
blinding RNG; failure clears the destination key and public output.

Revision `7c6ae5d` also tried two operand-transfer changes: it combined
preliminary clearing and zero-extension into one SRAM write pass, and derived
physical PKA operand addresses from the initial register layout instead of
reading MEMORY_MAP. The host -Os CryptoCell object text changed from 13,444 to
13,709 bytes (+265), with unchanged object data/BSS. This is not an ARM flash
measurement. Both transfer changes were later withdrawn after the board
regression below.

The production register-model suite exercises the generic test, boundary
scalars, OpenSSL comparisons, import contention, missing/failed blinding RNG,
repeated zero blinding values, arithmetic and cleanup timeouts, recovery and
disabled objects. Minimal archive checks cover unwanted software
AES/SHA/PRNG linkage. The model success on `7c6ae5d` did not reproduce the
silicon mapping behavior and therefore did not establish hardware correctness.

## P-256 board regression and restored baseline

The Audio DK App-core run after `7c6ae5d` returned 83 passed, 18 failed.
Public-key import and valid ECDH comparisons failed; AES/GCM/hash/RNG checks
continued to pass. CryptoTest(20261025-003032).map occupies 90,100 bytes of
flash (+88 from the preceding GHASH result) with unchanged 6,444-byte static
RAM including alignment. That map belongs to the failed revision.

Revision `d245e43` restored mapping-register reads and the separate clear then
write operand sequence. The next Audio DK App-core run passed 101 checks with
0 failures and 0 skips, confirming correctness recovery. No linker map was
supplied for that restored run, so no target flash delta is recorded for
`d245e43`.

The restored P-256 path was not a performance success. Mean public-key
calculation over eight imports was 2,058,809 us in CryptoCell and 487,297 us
in software: hardware was 4.23x slower. Other timings from the same run were:

| Operation | CryptoCell/target | Software reference |
| --- | ---: | ---: |
| AES ECB, 16 bytes | 362,828 ns | 522,156 ns |
| AES ECB, 1024 bytes | 10,133,718 ns | 32,114,703 ns |
| GCM, 1024 bytes | 21,810,359 ns | 45,057,687 ns |
| RNG, 1024 bytes | 89,541,000 ns | n/a |
| P-256 public-key calculation | 2,058,809 us | 487,297 us |

The generic P-256 soak now counts executed ECDH and signature rounds. A
public-key failure previously bypassed those checks while allowing them to
print PASS. Each result now requires all requested rounds to have executed.

## P-256 transaction and arithmetic reduction

The nRF5340 CryptoCell 312 product specification documents three relevant PKA
properties that the previous model did not represent:

- MEMORY_MAP entries may be changed dynamically by the PKA engine.
- PKA SRAM read and write data ports auto-increment their address.
- The SRAM write buffer must be cleared when changing between read and write
  access.

The fixed-address shortcut from `7c6ae5d` is therefore invalid. The optimized
driver continues to read the current hardware mapping for operand access and
retains the separate full clear before each partial operand write.

The new pass reduces the valid scalar-multiply path without changing the
generic application, key validation, projective blinding, scalar
regularization, bounded waits, ownership, cleanup, or result wiping:

- SRAM words are streamed through the documented auto-incrementing ports.
  Per-word PKA_DONE polling and repeated read-address programming are removed;
  a bounded completion check remains around each transfer.
- PKA_SRAM_WCLEAR is issued when the access direction changes.
- Final physical SRAM erasure is one contiguous auto-incrementing write over
  the same 32 allocated 48-byte slots.
- Selection masks and small temporary constants are formed with PKA logical
  and immediate operations instead of repeated 48-byte SRAM mask writes.
- The final Jacobian-to-affine conversion uses the PKA ModInv opcode instead
  of exponentiating Z to p-2.
- The regularized scalar has an implicit leading one. Before loop bits 255..2,
  its processed prefix r is below 2^254. Since the P-256 order is above
  2^255, 2*r*P is finite and cannot equal P or -P. The expensive exceptional
  PointAdd alternatives are therefore retained only for bits 1 and 0. This
  branch depends only on the public loop index.

For a successful 256-bit multiply, source-level helper expansion reduces the
scalar loop from roughly 44.8k PKA opcode submissions to roughly 24.5k, before
setup and affine conversion. It also removes the old 1,280 per-iteration
selection-mask SRAM writes; those masks alone previously caused 30,720
PKA_SRAM_WDATA word writes (12 clear plus 12 mask words per selection).
These are structural command/transaction counts, not MCU timing measurements.

The CC3xx PKA model now changes a result register's mapping after arithmetic,
auto-increments SRAM reads, requires PKA_SRAM_WCLEAR on direction changes,
models ModInv, and adds a successful-import traffic guard: fewer than 30,000
PKA operations, fewer than 1,024 SRAM word writes, fewer than 512 PKA_DONE
reads, two read-address programs, at least two direction clears, and observed
dynamic remapping. This closes the model gap that allowed the fixed-map
optimization to pass.

A fresh host runner result, MCU build, linker map and Audio DK timing are still
required for this performance pass. No new target flash/RAM size or hardware
speedup is claimed here.


## CryptoCell P-256 optimized hardware validation, 2026-10-25

The Audio DK nRF5340 application-core CryptoTest completed **101 passed,
0 failed, 0 skipped** after commit `4fdd11e729df1dd0c97d6b7ef4e190535d42afa8`.
It includes mixed Jacobian-affine additions to the prior PKA transaction and
arithmetic reductions. The fixed input remains affine, while the scalar
accumulator retains randomized Jacobian coordinates. The full generic
CryptoTest covers known-answer P-256 multiplication, ECDH, boundary scalars,
invalid peers and software/hardware comparisons.

| P-256 public-key import, mean of eight | CryptoCell | Software reference |
| --- | ---: | ---: |
| Restored baseline before optimization | 2,058,809 us | 487,297 us |
| First transaction/arithmetic reduction | 567,540 us | 487,297 us |
| Mixed-coordinate board run | 516,627 us | 532,576 us |

The final CryptoCell timing is 74.9% below the restored baseline (3.99x
speedup) and 3.0% below the software reference in the same final run. These
are individual eight-operation means, not a statistical speed comparison.
The mixed-coordinate change reduced the observed hardware time by 9.0%
relative to the first optimized run.

The supplied `CryptoTest(20261025-012944).map` has the following linker
measurements, including vector/startup sections, alignment and initialized
RAM data's Flash load image:

| App-core CryptoTest footprint | Bytes |
| --- | ---: |
| Flash occupied | 90,168 |
| .data, initialized static RAM | 796 |
| .bss, zero-initialized static RAM | 5,644 |
| Total static RAM | 6,440 |
| `.text` section | 59,896 |
| `.rodata` section | 28,564 |

Compared with the earlier failed transfer-attempt map (90,100 bytes Flash,
6,444 bytes static RAM), this final tested map is 68 bytes larger in Flash
and 4 bytes smaller in static RAM. Those two maps represent different code
revisions; the comparison does not isolate any one optimization. The generic
test deliberately includes software comparison objects; the map does not
represent production-only crypto linkage. Peak stack usage is not measured.

The board validation establishes function and performance correctness for the
compiled Audio DK App-core build only. Other CC3xx targets retain their
separate validation requirements.

## nRF52832 BlueIO hardware validation, 2026-10-24

The maintainer's Release CryptoTest completed **101 passed, 0 failed,
0 skipped** on the nRF52832 after `6956ebe` guarded the ECB SoftDevice
SVC for standalone applications. Benchmark timer: 1 MHz.

| Operation | Target | Software reference |
| --- | ---: | ---: |
| AES ECB, 16 B | 29,156 ns | 212,843 ns |
| AES ECB, 1024 B | 1,304,921 ns | 13,099,656 ns |
| CMAC, 1024 B | 1,772,859 ns | 13,761,937 ns |
| CCM, 1024 B | 3,163,984 ns | 27,519,781 ns |
| GCM, 1024 B | 5,826,296 ns | 18,001,765 ns |
| P-256 public-key import (8 mean) | 246,751 us | 246,746 us |

The nRF52832 has hardware AES-128 ECB but no hardware P-256 accelerator.
The P-256 default path therefore matches software timing. No linker map
or peak-stack measurement was supplied for this run.

## STM32L475 B-L475E-IOT01A hardware validation, 2026-10-24

The maintainer's generic CryptoTest completed **101 passed, 0 failed,
0 skipped**, with a 1 MHz benchmark timer. All AES-128/256, SHA-256/HMAC,
CMAC, CCM, GCM, P-256, ECDH/ECDSA, and RNG tests passed.

| Operation | Target | Software reference |
| --- | ---: | ---: |
| AES ECB, 16 B | 185,171 ns | 171,640 ns |
| AES ECB, 1024 B | 10,418,234 ns | 10,404,671 ns |
| CCM, 1024 B | 21,620,718 ns | 21,607,703 ns |
| GCM, 1024 B | 14,250,218 ns | 14,237,078 ns |
| P-256 public-key import (8 mean) | 194,147 us | 194,147 us |
| RNG, 1024 B | 224,703 ns | n/a |

The target and software comparisons have nearly equal execution time for
large operations, consistent with software execution for those operations.
No linker map or peak-stack measurement was supplied for this run.

## nRF9160 CryptoCell CC310 hardware validation, 2026-10-24

Following the explicit nRF9160 CC310 compile-time selection correction
(`446eeac`), the maintainer's nRF9160 CryptoTest completed **101 passed,
0 failed, 0 skipped** (1 MHz timer). The previous result was 73 passed,
52 failed. All AES-128/256, SHA-256/HMAC, CMAC, CCM, GCM, RNG and
P-256/ECDH/ECDSA checks now pass.

| Operation | Crypto engine | Software reference |
| --- | ---: | ---: |
| AES ECB, 1024 B | 7,086,687 ns | 22,715,078 ns |
| AES CTR, 1024 B | 7,384,750 ns | 22,860,343 ns |
| AES CBC, 1024 B | 7,338,078 ns | 22,932,796 ns |
| CMAC, 1024 B | 7,615,921 ns | 23,877,796 ns |
| CCM, 1024 B | 14,706,468 ns | 47,849,921 ns |
| GCM, 1024 B | 15,541,218 ns | 32,053,765 ns |
| P-256 public key (8 mean) | 360,728 us | 365,688 us |

No linker map or peak stack measurement was supplied for this run.

## nRF91x1 CC310 hardware validation, 2026-10-25

The maintainer's generic CryptoTest on nRF91x1 completed **101 passed,
0 failed, 0 skipped** with a 1 MHz benchmark timer. AES-128, AES-256
software fallback, SHA-256/HMAC, CMAC, CCM, GCM, RNG, P-256,
ECDH/ECDSA and randomized comparisons all passed.

| Operation | Target | Software reference |
| --- | ---: | ---: |
| AES ECB, 1024 B | 7,111,171 ns | 22,715,078 ns |
| AES CTR, 1024 B | 7,317,500 ns | 22,860,359 ns |
| AES CBC, 1024 B | 7,361,546 ns | 22,932,796 ns |
| CMAC, 1024 B | 7,634,500 ns | 23,877,796 ns |
| CCM, 1024 B | 14,742,625 ns | 47,849,921 ns |
| GCM, 1024 B | 15,560,375 ns | 32,053,781 ns |
| P-256 public-key import (8 mean) | 362,360 us | 371,158 us |

No linker map or peak-stack measurement was supplied.
