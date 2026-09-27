/**--------------------------------------------------------------------------
@file 	cfifo.h

@brief	Implementation of a simple circular FIFO buffer.

There is no queuing implementation and non blocking to be able to be use in
interrupt. User must ensure thread safety when used in a threaded environment.

@author Hoang Nguyen Hoan
@date 	Jan. 3, 2014

@license

MIT License

Copyright (c) 2014 I-SYST inc. All rights reserved.

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

#ifndef __CFIFO_H__
#define __CFIFO_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup FIFO
  * @{
  */

#pragma pack(push,4)

/// Header defining a circular fifo memory block.
typedef struct __CFIFO_Header {
	uint32_t PutIdx;			//!< Index to start of empty data block
	uint32_t GetIdx;			//!< Index to start of used data block
	uint32_t BlkSize;			//!< Block size in bytes. Note: This must be adjacent to pMemStart for compiler optimization
	uint8_t *pMemStart;			//!< Start of FIFO data memory
	int32_t MaxIdxCnt;			//!< Max block count
	uint32_t Mask;				//!< NbBlk-1 (pow2) or 0 (non-pow2)
	bool bBlocking;          	//!< False to push out when FIFO is full (drop)
	uint32_t DropCnt;           //!< Count dropped block
} CFifo_t;

#pragma pack(pop)

//typedef CFifo_t			CFIFOHDR;

/// @brief	CFIFO handle.
///
/// This handle is used for all CFIFO function calls. It is the pointer to to CFIFO memory block.
///
//typedef CFifo_t* HCFIFO;
typedef CFifo_t* hCFifo_t;

/// This macro calculates total memory require in bytes including header for byte based FIFO.
#define CFIFO_MEMSIZE(FSIZE)					((FSIZE) + sizeof(CFifo_t))

/// This macro calculates total memory require in bytes including header for block based FIFO.
#define CFIFO_TOTAL_MEMSIZE(NbBlk, BlkSize)		((NbBlk) * (BlkSize) + sizeof(CFifo_t))

/**
 * @brief	Initialize FIFO.
 *
 * This function must be called first to initialize FIFO before any other functions
 * can be used.
 *
 * @param	pMemBlk 		: Pointer to memory block to be used for FIFO
 * 							  NOTE : This memory block must be word aligned
 * @param	TotalMemSize	: Total memory size in byte
 * @param	BlkSize 		: Block size in bytes
 * @param   bBlocking  		: Behavior when FIFO is full.\n
 *                    			false - Old data will be pushed out to make place
 *                            		for new data. Put always succeed\n
 *                    			true  - New data will not be pushed in. Put will
 *                    					return fail.
 *
 * 	@return CFifo Handle
 */
hCFifo_t CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bBlocking);

/**
 * @brief	Inspect the next FIFO block without consuming it.
 *
 * The returned pointer remains owned by the FIFO. A later non-blocking put may
 * overwrite it when the FIFO is full, so callers must not retain the pointer.
 * This is intended for checking a block header before deciding whether to
 * consume the block with CFifoGet().
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Pointer to the next FIFO block, or NULL when empty.
 */
uint8_t *CFifoPeek(hCFifo_t const hFifo);

/**
 * @brief	Inspect consecutive FIFO blocks without consuming them.
 *
 * The returned span is limited by both the number of used blocks and the
 * physical end of the circular buffer. GetIdx is not modified.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Maximum number of blocks to inspect\n
 * 					On return number of consecutive blocks available
 *
 * @return	Pointer to first FIFO block, or NULL when empty.
 */
uint8_t *CFifoPeekMultiple(hCFifo_t const hFifo, int *pCnt);

/**
 * @brief	Retrieve FIFO data by returning pointer to FIFO memory block for reading.
 *
 * This function returns a direct pointer to FIFO memory to quickly retrieve data.
 * User must ensure to transfer data quickly to avoid data being overwritten by a
 * new FIFO put. This is to allows FIFO handling within interrupt.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Pointer to the FIFO buffer.
 */
uint8_t *CFifoGet(hCFifo_t const hFifo);

/**
 * @brief	Retrieve FIFO data in multiple blocks by returning pointer to FIFO memory blocks
 * for reading.
 *
 * This function returns a direct pointer to FIFO memory to quickly retrieve data.
 * User must ensure to transfer data quickly to avoid data being overwritten by a
 * new FIFO put. This is to allows FIFO handling within interrupt.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of block to get\n
 * 					On return number of blocks available to read
 *
 * @return	Pointer to first FIFO block. Blocks are consecutive.
 */
uint8_t *CFifoGetMultiple(hCFifo_t const hFifo, int *pCnt);

/**
 * @brief	Insert FIFO data by returning pointer to FIFO memory block for writing.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPut(hCFifo_t const hFifo);

/**
 * @brief	Insert multiple FIFO blocks by returning pointer to memory blocks for writing.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of block to put\n
 * 					On return pCnt contains the number of blocks available for writing
 *
 * @return	pointer to the first FIFO block. Blocks are consecutive.
 */
uint8_t *CFifoPutMultiple(hCFifo_t const hFifo, int *pCnt);

/**
 * @brief	Reserve the next FIFO block for writing without publishing it.
 *
 * Put-side counterpart of CFifoPeek. The block is not visible to the reader
 * until the producer publishes it with CFifoPut, which returns this same
 * block. Lets a DMA fill the block in place before it is published.
 * Single producer only: two callers reserving at once receive the same block.
 * A full FIFO returns NULL in either mode, leaving existing data untouched.
 * CFifoPut still discards the oldest block when the FIFO is non-blocking.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	pointer to the reserved FIFO block, NULL if none.
 */
uint8_t *CFifoResv(hCFifo_t const hFifo);

/**
 * @brief	Reserve consecutive FIFO blocks for writing without publishing them.
 *
 * Put-side counterpart of CFifoPeekMultiple. Reserve the most that could be
 * written, fill some or all of it, then publish the count actually written
 * with CFifoPutMultiple, which returns this same address. Single producer only.
 * A full FIFO returns NULL in either mode without discarding data.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of blocks wanted\n
 * 					On return the number of consecutive blocks reserved
 *
 * @return	pointer to the first reserved block, NULL if none.
 */
uint8_t *CFifoResvMultiple(hCFifo_t const hFifo, int *pCnt);

/**
 * @brief	Reset FIFO
 *
 * @param	hFifo : CFIFO handle
 */
void CFifoFlush(hCFifo_t const hFifo);

/**
 * @brief	Get available blocks in FIFO
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block available for writing
 */
int CFifoAvail(hCFifo_t const hFifo);

/**
 * @brief	Get number of used blocks
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block used
 */
int CFifoUsed(hCFifo_t const hFifo);

/**
 * @brief	Get block size
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Block size in bytes
 */
static inline uint32_t CFifoBlockSize(hCFifo_t const hFifo) { return hFifo->BlkSize; }

/**
 * @brief	Get the full behavior of the FIFO
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	true when a full FIFO refuses data and the caller is expected to
 * 			wait, false when a full FIFO pushes out the oldest data instead
 */
static inline bool CFifoIsBlocking(hCFifo_t const hFifo) { return hFifo->bBlocking; }

#ifdef __cplusplus
}
#endif

/** @} end group FIFO */

#endif // __CFIFO_H__
