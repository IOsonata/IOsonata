#ifndef IOSONATA_CRYPTO_SCE5_H
#define IOSONATA_CRYPTO_SCE5_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
// Internal serialized operations. The caller owns the shared SCE lock.
bool Sce5Init(void);
bool Sce5AesBlock(const uint8_t *key, size_t keyLen, bool encrypt,
	const uint8_t input[16], uint8_t output[16]);
bool Sce5RandomWords(uint32_t output[4]);
bool Sce5Ready(void);
#ifdef __cplusplus
}
#endif
#endif
