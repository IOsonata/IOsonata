/**-------------------------------------------------------------------------
@file	crypto_sce5.h

@brief	IOsonata crypto sce5.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/

#ifndef IOSONATA_CRYPTO_SCE5_H
#define IOSONATA_CRYPTO_SCE5_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
// Internal serialized operations. The caller owns the shared SCE lock.
// Random words and an AES session exclude each other.
bool Sce5Init(void);
bool Sce5Ready(void);
bool Sce5RandomWords(uint32_t output[4]);
// AES session: Begin generates the key index and initializes the engine for
// one key and direction (a repeat with the same key and direction keeps the
// session), Update runs one 16 byte block, End finalizes the engine.
bool Sce5AesBegin(const uint8_t *key, size_t keyLen, bool encrypt);
bool Sce5AesUpdate(const uint8_t input[16], uint8_t output[16]);
bool Sce5AesEnd(void);
#ifdef __cplusplus
}
#endif
#endif
