/**--------------------------------------------------------------------------
@file 	cfifo.h

@brief	Implementation of an overly simple circular FIFO buffer.

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

#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup FIFO
  * @{
  */

#pragma pack(push,4)

/// Header defining a circular fifo memory block.
typedef struct __CFIFO_Header {
	volatile int32_t PutIdx;	//!< Index to start of empty data block
	volatile int32_t GetIdx;	//!< Index to start of used data block
	int32_t MaxIdxCnt;			//!< Max block count
	bool bBlocking;          	//!< False to push out when FIFO is full (drop)
	uint32_t DropCnt;           //!< Count dropped block
	uint32_t BlkSize;			//!< Block size in bytes
	uint32_t MemSize;			//!< Total FIFO memory size allocated
	uint8_t *pMemStart;			//!< Start of FIFO data memory
} CFifo_t;

#pragma pack(pop)

typedef CFifo_t			CFIFOHDR;

/// @brief	CFIFO handle.
///
/// This handle is used for all CFIFO function calls. It is the pointer to to CFIFO memory block.
///
typedef CFifo_t* HCFIFO;

/// This macro calculates total memory require in bytes including header for byte based FIFO.
#define CFIFO_MEMSIZE(FSIZE)					((FSIZE) + sizeof(CFifo_t))

/// This macro calculates total memory require in bytes including header for block based FIFO.
#define CFIFO_TOTAL_MEMSIZE(NbBlk, BlkSize)		((NbBlk) * (BlkSize) + sizeof(CFifo_t))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize FIFO.
 *
 * This function must be called first to initialize FIFO before any other functions
 * can be used.
 *
 * @param	pMemBlk 		: Pointer to memory block to be used for FIFO
 * @param	TotalMemSize	: Total memory size in byte
 * @param	BlkSize 		: Block size in bytes
 * @param   bBlocking  		: Behavior when FIFO is full.\n
 *                    			false - Old data will be pushed out to make place
 *                            			for new data.\n
 *                    			true  - New data will not be pushed in
 *
 * 	@return CFifo Handle
 */
HCFIFO const CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bBlocking);

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
uint8_t *CFifoGet(HCFIFO const hFifo);

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
uint8_t *CFifoGetMultiple(HCFIFO const hFifo, int *pCnt);

/**
 * @brief	Insert FIFO data by returning pointer to FIFO memory block for writing.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPut(HCFIFO const hFifo);

/**
 * @brief	Insert multiple FIFO blocks by returning pointer to memory blocks for writing.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of block to put\n
 * 					On return pCnt contains the number of blocks available for writing
 *
 * @return	pointer to the first FIFO block. Blocks are consecutive.
 */
uint8_t *CFifoPutMultiple(HCFIFO const hFifo, int *pCnt);

/**
 * @brief	Retrieve FIFO data into provided buffer
 *
 * @param	hFifo : CFIFO handle
 * @param	pBuff : Pointer to buffer container for returned data
 * @param	BuffLen : Size of container in bytes
 *
 * @return	Number of bytes copied into pBuff
 */
int CFifoPop(HCFIFO const hFifo, uint8_t *pBuff, int BuffLen);

/**
 * @brief	Insert FIFO data with provided data
 *
 * @param	hFifo : CFIFO handle
 * @param	pData : Pointer to data to be inserted
 * @param	DataLen : Size of data in bytes
 *
 * @return	Number of bytes inserted into FIFO
 */
int CFifoPush(HCFIFO const hFifo, uint8_t *pData, int DataLen);

/**
 * @brief	Reset FIFO
 *
 * @param	hFifo : CFIFO handle
 */
void CFifoFlush(HCFIFO const hFifo);

/**
 * @brief	Get available blocks in FIFO
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block available for writing
 */
int CFifoAvail(HCFIFO const hFifo);

/**
 * @brief	Get number of used blocks
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block used
 */
int CFifoUsed(HCFIFO const hFifo);

/**
 * @brief	Get block size
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Block size in bytes
 */
static inline uint32_t CFifoBlockSize(HCFIFO const hFifo) { return hFifo->BlkSize; }

#ifdef __cplusplus
}
#endif

/** @} end group FIFO */

#endif // __FIFO_H__ 
