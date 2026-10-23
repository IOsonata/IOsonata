/**-------------------------------------------------------------------------
@file	ipc_intrf.h

@brief	Inter-core DeviceIntrf over shared RAM.

IpcIntrf is a data pipe between two CPU cores of one chip. It is one device
that owns a memory block placed in RAM both cores can read and write. Init
partitions the block into the device data and two CFifo, one for each
direction, and formats the two CFifo inside it:

	IpcIntrfHdr_t		device data: interlock and geometry
	CFifo owner to peer	header, then the blocks, rounded up to a word
	CFifo peer to owner	header, then the blocks, rounded up to a word

IPC_INTRF_MEMSIZE() gives the block size for a pair of rings, the same way
CFIFO_TOTAL_MEMSIZE() does for one CFifo. Linker scripts declare the block
with a MEMORY entry and export its start and size; no C define repeats the
address.

One core is the owner. It partitions and formats the block, then sets the
magic of the device data last. The other core is the peer. It is given only
the block, not the sizes: it reads the geometry from the device data and uses
the two CFifo where the owner put them, once the magic reads valid and the
geometry fits its block. The peer can start before or after the owner. The
owner formatting again is seen by the peer as a new epoch: the interface
reports itself down until the owner has formatted again. Data in the rings at
that time is lost.

Each ring is configured like a CFifo, as a byte stream or as a packet stream.
The block size of a ring selects its mode, and the two rings may differ:

	block size 0 or 1	Byte stream. The block count is the ring size in
						bytes. A write is split or merged freely, a read
						returns what is there up to the buffer size. Message
						boundaries are the business of the protocol above.
	block size > 2		Packet stream. A block is one packet slot: a 16 bit
						length followed by the payload, so a packet carries at
						most BlkSize - 2 bytes. One TxData writes one packet or
						nothing. One RxData returns one whole packet. Packet
						boundaries are kept end to end, any size up to the
						slot size, and a full ring takes no partial packet.

A packet that is longer than the slot is refused at the write (0 returned, the
IpcIntrfDev_t TxOversizeCnt counts it). A packet longer than the read buffer
is dropped (0 returned, RxDropCnt counts it) rather than left to block the
ring. Both calls return 0 for these because the framework keeps the interface
busy after a negative return.

One core produces and one consumes in each ring, and each index has a single
writer, so no lock is needed. A put reserves the space, fills it, then
publishes the count; a get peeks the bytes, copies them, then releases them.
The other core never sees a byte before it is written, and never sees one
overwritten while it is being read.

The IPC peripheral of the target only signals. After a put the producer
raises its TxChan. The consumer takes the interrupt on its RxChan and the
interface reports DEVINTRF_EVT_RX_DATA to the configuration EvtCB. A producer
that found its ring full sets a wait flag in the device data. The consumer
raises its TxChan after it frees room, and the interface reports
DEVINTRF_EVT_TX_READY on the waiting side.

DevAddr is not used. One IpcIntrf is one endpoint, one ring pair and one
channel pair. A protocol layer (HCI, modem, application framing) derives from
IpcIntrf the way the USB classes derive from UsbIntrf.

StartRx and StartTx take no state. The framework busy flag already serializes
transfers, so TxData has one producer and RxData has one consumer. The
interrupt handler never produces and never consumes.

Hardware access is in the target port, see the port seam at the end of this
file.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __IPC_INTRF_H__
#define __IPC_INTRF_H__

#include <stdint.h>
#include <stdbool.h>

#include "cfifo.h"
#include "device_intrf.h"

/** @addtogroup IPC
  * @{
  */

/// Magic of the device data, set by the owner after the block is formatted
#define IPC_INTRF_MAGIC				0x49504331UL
/// Layout version, checked by the peer
#define IPC_INTRF_VERSION			3U

/// Bytes of the length field at the start of a packet slot
#define IPC_INTRF_PKT_HDR_SIZE		2U
/// Largest packet slot, the length field is 16 bits
#define IPC_INTRF_PKT_BLK_MAX		(65535U + IPC_INTRF_PKT_HDR_SIZE)
/// Payload bytes of one packet in a slot of BlkSize bytes
#define IPC_INTRF_PKT_PAYLOAD(BlkSize)	((BlkSize) - IPC_INTRF_PKT_HDR_SIZE)

/// Block size as the CFifo sees it: 0 is a byte stream like 1
#define IPC_INTRF_BLK(BlkSize)		((BlkSize) == 0U ? 1U : (BlkSize))

/// Memory of one ring of NbBlk blocks of BlkSize bytes, CFifo header
/// included, rounded up to a word. For a byte stream NbBlk is the byte count.
#define IPC_INTRF_RING_MEMSIZE(NbBlk, BlkSize) \
	((CFIFO_TOTAL_MEMSIZE((NbBlk), IPC_INTRF_BLK(BlkSize)) + 3U) & ~3U)

/// Memory block size for the device: device data and the two rings.
/// Tx is the ring the owner writes, Rx the ring the peer writes.
#define IPC_INTRF_MEMSIZE(TxNbBlk, TxBlkSize, RxNbBlk, RxBlkSize) \
	((uint32_t)sizeof(IpcIntrfHdr_t) + IPC_INTRF_RING_MEMSIZE(TxNbBlk, TxBlkSize) + \
	 IPC_INTRF_RING_MEMSIZE(RxNbBlk, RxBlkSize))

/// Geometry of one ring, written by the owner
typedef struct __Ipc_Intrf_Ring {
	uint32_t Off;				//!< Offset of the CFifo from the start of the block
	uint32_t NbBlk;				//!< Block count
	uint32_t BlkSize;			//!< Block size, 1 for a byte stream
} IpcIntrfRing_t;

/// Device data at the start of the memory block: the interlock and the
/// geometry. Each word has one writer: the owner for all of them but Wait,
/// and the producer of that ring for its own Wait word.
typedef struct __Ipc_Intrf_Hdr {
	uint32_t Magic;				//!< IPC_INTRF_MAGIC when the block is formatted, written last
	uint32_t Version;			//!< IPC_INTRF_VERSION
	uint32_t Epoch;				//!< Counts the owner formats
	uint32_t Wait[2];			//!< [0] owner waits for room, [1] peer waits for room
	IpcIntrfRing_t Ring[2];		//!< [0] ring the owner writes, [1] ring the peer writes
} IpcIntrfHdr_t;

/// Configuration. The peer needs only the block and the channels.
typedef struct __Ipc_Intrf_Config {
	uint8_t *pMemBlk;			//!< Memory block, word aligned, same address on both cores
	uint32_t MemSize;			//!< Block size in bytes. Owner: at least IPC_INTRF_MEMSIZE(). Peer: the size of the block
	bool bOwner;				//!< true on the core that formats the block
	uint32_t TxNbBlk;			//!< Owner: blocks of the ring it writes. Ignored by the peer
	uint32_t TxBlkSize;			//!< Owner: 0 or 1 byte stream, else packet slot size. Ignored by the peer
	uint32_t RxNbBlk;			//!< Owner: blocks of the ring it reads. Ignored by the peer
	uint32_t RxBlkSize;			//!< Owner: 0 or 1 byte stream, else packet slot size. Ignored by the peer
	int TxChan;					//!< IPC channel this core raises toward the other core
	int RxChan;					//!< IPC channel the other core raises toward this core
	int IntPrio;				//!< IPC interrupt priority
	DevIntrfEvtHandler_t EvtCB;	//!< Events, NULL for none. pBuffer is NULL, Len is the block count
								//!< (bytes in a byte stream) for DEVINTRF_EVT_RX_DATA.
								//!< Also DEVINTRF_EVT_TX_READY.
} IpcIntrfCfg_t;

/// Interface state. The ring handles point into the memory block.
typedef struct __Ipc_Intrf_Device {
	DevIntrf_t DevIntrf;		//!< Base device interface
	IpcIntrfHdr_t *pHdr;		//!< Device data in the memory block
	uint32_t MemSize;			//!< Size of the memory block
	hCFifo_t hTxFifo;			//!< Ring this core writes
	hCFifo_t hRxFifo;			//!< Ring this core reads
	uint32_t TxBlk;				//!< Block size of the Tx ring, 1 for a byte stream
	uint32_t RxBlk;				//!< Block size of the Rx ring, 1 for a byte stream
	uint32_t TxOversizeCnt;		//!< Packets refused for being longer than the slot
	uint32_t RxDropCnt;			//!< Packets dropped for being longer than the read buffer
	uint32_t Epoch;				//!< Epoch this core attached to
	int TxChan;
	int RxChan;
	uint8_t Me;					//!< Index of this core in Wait, 0 owner, 1 peer
	bool bOwner;
	bool bUp;					//!< Block formatted and attached, atomic accesses only
	IpcIntrfCfg_t Cfg;			//!< Kept for the owner to format again
} IpcIntrfDev_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the interface.
 *
 * The owner formats the block and raises its TxChan once, so a peer that is
 * already running notices. The peer does not touch the rings: it waits for
 * the owner, and Tx and Rx return 0 until the link is up.
 *
 * @param	pDev	: Interface device data
 * @param	pCfg	: Configuration
 *
 * @return	true - success\n
 * 			false - bad configuration, block too small for the rings, or the
 * 			target port refused the channels
 */
bool IpcIntrfInit(IpcIntrfDev_t * const pDev, const IpcIntrfCfg_t * const pCfg);

/**
 * @brief	Tell whether the link to the other core is up.
 *
 * On the peer this also attaches when the owner has formatted the block, and
 * detaches when the owner formats again.
 *
 * @param	pDev	: Interface device data
 *
 * @return	true when Tx and Rx can move data
 */
bool IpcIntrfIsUp(IpcIntrfDev_t * const pDev);

/**
 * @brief	Free room in the ring this core writes.
 *
 * Byte stream: a write of at most this many bytes is taken whole. Meant for
 * a protocol above the interface that must not write a message in two parts.
 * Packet stream: the number of free packet slots, each takes a packet of up
 * to BlkSize - 2 bytes.
 *
 * @param	pDev	: Interface device data
 *
 * @return	Free bytes or free packet slots, 0 while the link is down
 */
int IpcIntrfTxRoom(IpcIntrfDev_t * const pDev);

/**
 * @brief	Interrupt work of one received IPC signal.
 *
 * Called by the target port from the IPC interrupt after it cleared the
 * receive event of pDev->RxChan. Reports DEVINTRF_EVT_RX_DATA when bytes are
 * waiting and DEVINTRF_EVT_TX_READY when room came back for a producer that
 * had run out.
 *
 * @param	pDev	: Interface device data
 */
void IpcIntrfIrqHandler(IpcIntrfDev_t * const pDev);

// Port seam. One definition per target, in the MCU port.

/**
 * @brief	Claim the channels and the interrupt of pDev.
 *
 * Connects pDev->TxChan as a send channel and pDev->RxChan as a receive
 * channel, binds pDev to the receive interrupt, which calls
 * IpcIntrfIrqHandler, and enables it at pDev->DevIntrf.IntPrio. A port that
 * must open the memory block to the other core does it here.
 *
 * @param	pDev	: Interface device data, filled by IpcIntrfInit
 *
 * @return	true - success\n
 * 			false - channel out of range or already claimed
 */
bool IpcCtrlrInit(IpcIntrfDev_t * const pDev);

/**
 * @brief	Raise a channel toward the other core.
 *
 * Memory writes made before this call are visible to the other core when it
 * takes the signal.
 *
 * @param	Chan	: IPC channel
 */
void IpcCtrlrSend(int Chan);

/**
 * @brief	Enable or disable the receive interrupt of pDev.
 *
 * @param	pDev	: Interface device data
 * @param	bEnable	: true to enable
 */
void IpcCtrlrRxEnable(IpcIntrfDev_t * const pDev, bool bEnable);

#ifdef __cplusplus
}

class IpcIntrf : public DeviceIntrf {
public:
	IpcIntrf() : vIpcDevIntrf() {}
	IpcIntrf(const IpcIntrf &) = delete;
	IpcIntrf &operator = (const IpcIntrf &) = delete;

	bool Init(const IpcIntrfCfg_t &Cfg) {
		return IpcIntrfInit(&vIpcDevIntrf, &Cfg);
	}

	/// true when the link to the other core is up
	bool Up(void) { return IpcIntrfIsUp(&vIpcDevIntrf); }

	operator DevIntrf_t * () override {
		return &vIpcDevIntrf.DevIntrf;
	}

	DevIntrf_t *Data(void) { return &vIpcDevIntrf.DevIntrf; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vIpcDevIntrf.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vIpcDevIntrf.DevIntrf);
	}

	void Disable(void) override {
		DeviceIntrfDisable(&vIpcDevIntrf.DevIntrf);
	}

	void Enable(void) override {
		DeviceIntrfEnable(&vIpcDevIntrf.DevIntrf);
	}

	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vIpcDevIntrf.DevIntrf, DevAddr);
	}

	void StopRx(void) override {
		DeviceIntrfStopRx(&vIpcDevIntrf.DevIntrf);
	}

	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vIpcDevIntrf.DevIntrf, DevAddr);
	}

	void StopTx(void) override {
		DeviceIntrfStopTx(&vIpcDevIntrf.DevIntrf);
	}

	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vIpcDevIntrf.DevIntrf, DevAddr, pData, DataLen);
	}

	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vIpcDevIntrf.DevIntrf, DevAddr, pBuff, BuffLen);
	}

	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vIpcDevIntrf.DevIntrf, pData, DataLen);
	}

	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vIpcDevIntrf.DevIntrf, pBuff, BuffLen);
	}

protected:
	IpcIntrfDev_t vIpcDevIntrf;
};

#endif

/** @} End of group IPC */

#endif // __IPC_INTRF_H__
