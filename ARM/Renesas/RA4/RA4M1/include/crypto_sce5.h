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
