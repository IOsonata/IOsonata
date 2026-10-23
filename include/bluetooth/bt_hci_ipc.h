/**-------------------------------------------------------------------------
@file	bt_hci_ipc.h

@brief	Bluetooth HCI transport over IpcIntrf, one packet per slot

This is the IPC binding of BtHciH4 (bt_hci_h4.h), the same way BtHciUart is
the UART binding. The two cores share one memory block holding two packet
rings (ipc/ipc_intrf.h). One ring slot carries one HCI packet: the packet
type byte followed by the packet, the same bytes H4 puts on a wire. The ring
keeps the packet boundary, so nothing is framed from a byte stream: a packet
is never cut in two, and a core that restarts cannot leave half a packet
behind for the other to frame from.

Either core can own the block. The service side owns it: it formats the
rings and the other core attaches to what it finds. On a controller core the
controller is the owner, the host attaches.

Received packets are queued in the caller's context, never in the IPC
interrupt. The interrupt only reports that packets arrived (RxReady). The
owner then calls BtHciIpcProcess from its event queue. The command and
response model of BtHciUart applies unchanged, see bt_hci_uart.h.

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
#ifndef __BT_HCI_IPC_H__
#define __BT_HCI_IPC_H__

#include <stdint.h>
#include <stdbool.h>

#include "ipc/ipc_intrf.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_h4.h"

/** @addtogroup Bluetooth
  * @{
  */

/// Packet types, same values as H4
#define BT_HCI_IPC_PKT_CMD				BT_HCI_H4_PKT_CMD
#define BT_HCI_IPC_PKT_ACL				BT_HCI_H4_PKT_ACL
#define BT_HCI_IPC_PKT_SCO				BT_HCI_H4_PKT_SCO
#define BT_HCI_IPC_PKT_EVT				BT_HCI_H4_PKT_EVT
#define BT_HCI_IPC_PKT_ISO				BT_HCI_H4_PKT_ISO

/// Largest HCI packet one ring slot holds, type byte excluded. The packet
/// interface (BtHciIpcIntrf) carries all of it: it covers an ISO packet with
/// a 251 byte SDU, the largest the SoftDevice Controller hands up with its
/// default buffers. BtHciIpc, which queues through BtHciH4, carries what fits
/// BT_HCI_H4_PKT_MAXLEN and refuses the rest. Both sides of a link must use
/// the same value, it sets the slot size.
#ifndef BT_HCI_IPC_PKT_MAXLEN
#define BT_HCI_IPC_PKT_MAXLEN			272U
#endif
#define BT_HCI_IPC_TIMEOUT_DEFAULT		BT_HCI_H4_TIMEOUT_DEFAULT
#define BT_HCI_IPC_ERR_TIMEOUT			BT_HCI_H4_ERR_TIMEOUT

/// Ring slot size: type byte, packet and the slot length header
#define BT_HCI_IPC_BLKSIZE				(1U + BT_HCI_IPC_PKT_MAXLEN + IPC_INTRF_PKT_HDR_SIZE)

/// Shared memory block for NbPkt slots in each direction
#define BT_HCI_IPC_MEMSIZE(NbPkt) \
	IPC_INTRF_MEMSIZE((NbPkt), BT_HCI_IPC_BLKSIZE, (NbPkt), BT_HCI_IPC_BLKSIZE)

/// Packet queue memory for NbPkt packets
#define BT_HCI_IPC_PKTMEM_SIZE(NbPkt)	BT_HCI_H4_PKTMEM_SIZE(NbPkt)

typedef BtHciH4Pkt_t BtHciIpcPkt_t;

typedef struct __Bt_Hci_Ipc_Dev		BtHciIpcDev_t;

/**
 * @brief	Received packet handler.
 *
 * @param	pDev	Transport.
 * @param	Type	Packet type, BT_HCI_IPC_PKT_*.
 * @param	pPkt	HCI packet, header first, without the type byte. Valid
 * 					only during the call.
 * @param	Len		Packet length in bytes.
 */
typedef void (*BtHciIpcRxHandler_t)(BtHciIpcDev_t * const pDev, uint8_t Type,
									uint8_t *pPkt, uint16_t Len);

typedef struct __Bt_Hci_Ipc_Config {
	uint8_t *pMemBlk;					//!< Shared memory block, word aligned, same address on both cores
	uint32_t MemSize;					//!< Block size. Owner: at least BT_HCI_IPC_MEMSIZE(NbPkt). Peer: the block size
	bool bOwner;						//!< true on the core that formats the block
	uint32_t NbPkt;						//!< Owner: packet slots in each direction. Ignored by the peer
	int TxChan;							//!< IPC channel this core raises toward the other core
	int RxChan;							//!< IPC channel the other core raises toward this core
	int IntPrio;						//!< IPC interrupt priority
	uint8_t *pPktMem;					//!< Packet queue memory, BT_HCI_IPC_PKTMEM_SIZE(n), word aligned
	uint32_t PktMemSize;				//!< Packet queue memory size in bytes
	BtHciIpcRxHandler_t RxHandler;		//!< Each queued packet, in arrival order, from BtHciIpcProcess
	BtHciIpcRxHandler_t CmdRspHandler;	//!< Command Complete and Command Status as soon as they are read, ahead of
										//!< the queue. NULL queues them like any other packet.
	void (*RxReady)(BtHciIpcDev_t * const pDev);	//!< Packets arrived, called from the IPC interrupt. The owner
										//!< schedules BtHciIpcProcess. Keep it short.
	uint32_t TimeoutMs;					//!< Command response and TX room wait, 0 selects BT_HCI_IPC_TIMEOUT_DEFAULT
	void *pCtx;							//!< Owner data, kept in the device
} BtHciIpcCfg_t;

/// Transport state. Ipc must stay the first member: the IPC event handler
/// gets the interface and finds the transport from it.
struct __Bt_Hci_Ipc_Dev {
	IpcIntrfDev_t Ipc;					//!< IPC interface
	BtHciH4Dev_t H4;					//!< Packet queue and command layer, reads and writes through Ipc
	BtHciIpcRxHandler_t RxHandler;
	BtHciIpcRxHandler_t CmdRspHandler;
	void (*RxReady)(BtHciIpcDev_t * const pDev);
	void *pCtx;							//!< Owner data from the configuration
};

/**
 * Packet DevIntrf face of the same slot format.
 *
 * For a controller that already works on a packet interface, as it does over
 * native USB (bt_hci_usb.h): one successful transfer is one complete HCI
 * packet, DevAddr is the packet type and no type byte is in the data. It
 * replaces BtHciIpc, not adds to it: one of the two owns the IpcIntrf.
 *
 * Rx(DevAddr) returns the next packet when it is of that type, else 0. The
 * packet stays until it is asked for by its type, nothing behind it is read.
 * A buffer smaller than the packet drops it and counts it in RxDropCnt.
 * Tx(DevAddr) returns the packet length, or 0 when no slot is free.
 */
typedef struct __Bt_Hci_Ipc_Intrf_Dev	BtHciIpcIntrfDev_t;

typedef struct __Bt_Hci_Ipc_Intrf_Config {
	uint8_t *pMemBlk;					//!< Shared memory block, word aligned, same address on both cores
	uint32_t MemSize;					//!< Block size. Owner: at least BT_HCI_IPC_MEMSIZE(NbPkt). Peer: the block size
	bool bOwner;						//!< true on the core that formats the block
	uint32_t NbPkt;						//!< Owner: packet slots in each direction. Ignored by the peer
	int TxChan;							//!< IPC channel this core raises toward the other core
	int RxChan;							//!< IPC channel the other core raises toward this core
	int IntPrio;						//!< IPC interrupt priority
	DevIntrfEvtHandler_t EvtCB;			//!< DEVINTRF_EVT_RX_DATA when packets arrived and DEVINTRF_EVT_TX_READY when
										//!< a slot came free, called from the IPC interrupt. NULL for none.
} BtHciIpcIntrfCfg_t;

struct __Bt_Hci_Ipc_Intrf_Dev {
	IpcIntrfDev_t Ipc;					//!< IPC interface, first member
	DevIntrf_t DevIntrf;				//!< Packet interface
	uint8_t Held[1 + BT_HCI_IPC_PKT_MAXLEN];	//!< Packet read and not taken yet, type byte first
	uint16_t HeldLen;					//!< Bytes in Held, 0 for none
	uint8_t TxType;						//!< Type selected by StartTx
	uint32_t SyncErrCnt;				//!< Slots dropped for not being a whole packet
	uint32_t RxDropCnt;					//!< Packets dropped for a buffer that was too small
	DevIntrfEvtHandler_t EvtCB;
};

#ifdef __cplusplus
extern "C" {
#endif

/// Initialize the IPC interface and its packet face. Same failures as BtHciIpcInit.
bool BtHciIpcIntrfInit(BtHciIpcIntrfDev_t * const pDev, const BtHciIpcIntrfCfg_t *pCfg);

/// The packet interface, for HciController and the like
static inline DevIntrf_t *BtHciIpcIntrfGet(BtHciIpcIntrfDev_t * const pDev) { return &pDev->DevIntrf; }

/// true when the link to the other core is up
bool BtHciIpcIntrfIsUp(BtHciIpcIntrfDev_t * const pDev);

/**
 * @brief	Initialize the transport and its IPC interface.
 *
 * The owner formats the block. The peer attaches when the owner has.
 *
 * @return	true on success. false on a missing configuration, packet memory
 * 			too small or misaligned, or an IPC interface that refused the
 * 			block or the channels.
 */
bool BtHciIpcInit(BtHciIpcDev_t * const pDev, const BtHciIpcCfg_t *pCfg);

/// true when the link to the other core is up
bool BtHciIpcIsUp(BtHciIpcDev_t * const pDev);

/**
 * @brief	Send one HCI packet.
 *
 * The packet goes out whole or not at all. Waits up to TimeoutMs for a free
 * slot. Send from one context: the one that runs BtHciIpcProcess.
 *
 * @param	Type	Packet type, BT_HCI_IPC_PKT_*.
 * @param	pData	HCI packet, header first, without the type byte.
 * @param	Len		Packet length, 1 to BT_HCI_H4_PKT_MAXLEN.
 *
 * @return	true when the packet is in the ring.
 */
bool BtHciIpcSend(BtHciIpcDev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len);

/// Discard everything received. Ignored when called from a packet handler.
void BtHciIpcFlush(BtHciIpcDev_t * const pDev);

/// Read the packets that arrived into the queue, see BtHciUartPoll
void BtHciIpcPoll(BtHciIpcDev_t * const pDev);

/// Read the packets and dispatch the queued ones, see BtHciUartProcess
bool BtHciIpcProcess(BtHciIpcDev_t * const pDev, int MaxPkt);

/// Send an HCI command and wait for its response, host side. See BtHciUartCommand.
uint8_t BtHciIpcCommand(BtHciIpcDev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
						const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen);

#ifdef __cplusplus
}

/// C++ wrapper of the packet interface, a DeviceIntrf like BtHciUsb
class BtHciIpcIntrf : public DeviceIntrf {
public:
	BtHciIpcIntrf() = default;
	BtHciIpcIntrf(const BtHciIpcIntrf &) = delete;
	BtHciIpcIntrf &operator = (const BtHciIpcIntrf &) = delete;

	bool Init(const BtHciIpcIntrfCfg_t &Cfg) { return BtHciIpcIntrfInit(&vDevData, &Cfg); }

	/// true when the link to the other core is up
	bool Up(void) { return BtHciIpcIntrfIsUp(&vDevData); }

	operator DevIntrf_t * () override { return &vDevData.DevIntrf; }
	DevIntrf_t *Data(void) { return &vDevData.DevIntrf; }
	operator BtHciIpcIntrfDev_t * () { return &vDevData; }

	uint32_t Rate(uint32_t DataRate) override { return DeviceIntrfSetRate(&vDevData.DevIntrf, DataRate); }
	uint32_t Rate(void) override { return DeviceIntrfGetRate(&vDevData.DevIntrf); }
	void Disable(void) override { DeviceIntrfDisable(&vDevData.DevIntrf); }
	void Enable(void) override { DeviceIntrfEnable(&vDevData.DevIntrf); }
	bool StartRx(uint32_t DevAddr) override { return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr); }
	void StopRx(void) override { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	bool StartTx(uint32_t DevAddr) override { return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr); }
	void StopTx(void) override { DeviceIntrfStopTx(&vDevData.DevIntrf); }
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vDevData.DevIntrf, DevAddr, pData, DataLen);
	}
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vDevData.DevIntrf, DevAddr, pBuff, BuffLen);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}

private:
	BtHciIpcIntrfDev_t vDevData = {};
};

/// C++ wrapper of the same transport
class BtHciIpc {
public:
	BtHciIpc() = default;
	BtHciIpc(const BtHciIpc &) = delete;
	BtHciIpc &operator = (const BtHciIpc &) = delete;

	bool Init(const BtHciIpcCfg_t &Cfg) { return BtHciIpcInit(&vDevData, &Cfg); }
	bool IsUp(void) { return BtHciIpcIsUp(&vDevData); }
	bool Send(uint8_t Type, const void *pData, uint32_t Len) { return BtHciIpcSend(&vDevData, Type, pData, Len); }
	void Flush(void) { BtHciIpcFlush(&vDevData); }
	void Poll(void) { BtHciIpcPoll(&vDevData); }
	bool Process(int MaxPkt) { return BtHciIpcProcess(&vDevData, MaxPkt); }
	uint8_t Command(BtHciDevice_t * const pHci, uint16_t OpCode, const void *pParam, uint8_t ParamLen,
					void *pRet, uint8_t RetLen) {
		return BtHciIpcCommand(&vDevData, pHci, OpCode, pParam, ParamLen, pRet, RetLen);
	}

	operator BtHciIpcDev_t * () { return &vDevData; }
	operator IpcIntrfDev_t * () { return &vDevData.Ipc; }

private:
	BtHciIpcDev_t vDevData = {};
};
#endif

/** @} End of group Bluetooth */

#endif
