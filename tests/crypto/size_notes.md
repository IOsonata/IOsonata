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
This is not an MCU speedup measurement. Target flash size and timings for the
transport optimization still require a rebuilt App-core library and CryptoTest.
