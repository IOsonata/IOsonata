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
