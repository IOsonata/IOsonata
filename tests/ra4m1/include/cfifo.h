/* REDUCED TEST CONTRACT. Production uses include/cfifo.h and src/cfifo.c. */
#ifndef TEST_CFIFO_H
#define TEST_CFIFO_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#pragma pack(push,4)
typedef struct {
 uint32_t PutIdx,GetIdx,BlkSize;
 uint8_t *pMemStart;
 int32_t MaxIdxCnt;
 uint32_t Mask;
 bool bBlocking;
 uint32_t DropCnt;
} CFifo_t;
#pragma pack(pop)
typedef CFifo_t *hCFifo_t;
#define CFIFO_MEMSIZE(n) ((n)+sizeof(CFifo_t))
#ifdef __cplusplus
extern "C" {
#endif
hCFifo_t CFifoInit(uint8_t *p, uint32_t size, uint32_t block, bool blocking);
uint8_t *CFifoGet(hCFifo_t p);
uint8_t *CFifoPut(hCFifo_t p);
void CFifoFlush(hCFifo_t p);
int CFifoUsed(hCFifo_t p);
int CFifoAvail(hCFifo_t p);
#ifdef __cplusplus
}
#endif
#endif
