/**-------------------------------------------------------------------------
@file	bt_hci_h4.h

@brief	Bluetooth HCI H4 framing over any DeviceIntrf

HCI packets over a byte stream, each one preceded by its packet type byte
(Core Vol 4 Part A). The stream is a DeviceIntrf given at initialization, as
Slip does, so the same framing serves a UART, an IpcIntrf between two cores,
or any other interface that moves bytes in order. The framing is the same in
both directions, so one layer serves a host talking to an external controller
and a controller answering a host.

Received packets are framed in the caller's context, never in the interrupt of
the interface. The interrupt only reports that bytes arrived, and the owner
passes that on with BtHciH4RxNotify. The owner then calls BtHciH4Process from
its event queue to frame and dispatch them. The interface buffers the bytes in
between, so its receive buffer has to hold what arrives while the queue is
busy.

A host waits for each command response with BtHciH4Command. It frames the
bytes itself while it waits. Command Complete and Command Status go straight
to CmdRspHandler, every other packet stays queued in arrival order for
BtHciH4Process. A command can therefore be sent from a packet handler: the
packet being handled is not overwritten and no handler is entered twice.
When the queue is full, one more packet is framed into a spare slot. A
response right behind a full queue is still handled; a response further
back is reached only after packets are dispatched, so the command times out.
Size the queue for the packets that can arrive while a handler runs.

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
#ifndef __BT_HCI_H4_H__
#define __BT_HCI_H4_H__

#include <stdint.h>
#include <stdbool.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "bluetooth/bt_hci.h"

/** @addtogroup Bluetooth
  * @{
  */

// H4 packet type, the byte sent ahead of each HCI packet
#define BT_HCI_H4_PKT_CMD				0x01U	//!< HCI command, host to controller
#define BT_HCI_H4_PKT_ACL				0x02U	//!< ACL data, both directions
#define BT_HCI_H4_PKT_SCO				0x03U	//!< Synchronous data, both directions
#define BT_HCI_H4_PKT_EVT				0x04U	//!< HCI event, controller to host
#define BT_HCI_H4_PKT_ISO				0x05U	//!< ISO data, both directions

/// Largest HCI packet carried, packet header included, type byte excluded.
/// A received packet longer than this is dropped and counted in DropCnt.
/// The default holds a 255 byte event and a 251 byte LE ACL payload.
#ifndef BT_HCI_H4_PKT_MAXLEN
#define BT_HCI_H4_PKT_MAXLEN			BT_HCI_BUFFER_MAX_SIZE
#endif

/// Default wait for a command response and for TX room, in msec
#define BT_HCI_H4_TIMEOUT_DEFAULT		1000U

/// Returned by BtHciH4Command when the controller did not answer in time or
/// the command could not be sent. HCI status codes do not reach this value.
#define BT_HCI_H4_ERR_TIMEOUT			0xFFU

/// One received packet in the packet queue
typedef struct __Bt_Hci_H4_Packet {
	uint8_t Type;							//!< H4 packet type
	uint8_t Rsvd;
	uint16_t Len;							//!< Bytes in Data, packet header included
	uint8_t Data[BT_HCI_H4_PKT_MAXLEN];		//!< HCI packet without the type byte
} BtHciH4Pkt_t;

/// Packet queue memory for NbPkt packets
#define BT_HCI_H4_PKTMEM_SIZE(NbPkt)	CFIFO_TOTAL_MEMSIZE((NbPkt), sizeof(BtHciH4Pkt_t))

/// Smallest TX room of the interface: one whole packet with its type byte. A
/// packet is written only when all of it fits, a packet cut short would leave
/// the other side framing from the middle of it.
#define BT_HCI_H4_TXROOM_MIN			(BT_HCI_H4_PKT_MAXLEN + 1)

typedef struct __Bt_Hci_H4_Dev	BtHciH4Dev_t;

/**
 * @brief	Received packet handler.
 *
 * @param	pDev	Framing layer.
 * @param	Type	H4 packet type, BT_HCI_H4_PKT_*.
 * @param	pPkt	HCI packet, header first, without the type byte. Valid
 * 					only during the call.
 * @param	Len		Packet length in bytes.
 */
typedef void (*BtHciH4RxHandler_t)(BtHciH4Dev_t * const pDev, uint8_t Type,
								   uint8_t *pPkt, uint16_t Len);

/**
 * @brief	Free room for transmit in the interface, in bytes.
 *
 * Tells how many bytes the next write to the interface takes whole. The
 * interface is written only when the packet and its type byte fit.
 *
 * @param	pDev	Framing layer.
 *
 * @return	Bytes the interface takes now.
 */
typedef int (*BtHciH4TxRoom_t)(BtHciH4Dev_t * const pDev);

typedef struct __Bt_Hci_H4_Config {
	DevIntrf_t *pPhyIntrf;				//!< Interface the bytes go through. Reads are polled until it has
										//!< nothing, so MaxRetry 1 avoids a delay on every empty read. A write
										//!< must be taken whole or refused, never cut short.
	uint8_t *pPktMem;					//!< Packet queue memory, BT_HCI_H4_PKTMEM_SIZE(n), word aligned
	uint32_t PktMemSize;				//!< Packet queue memory size in bytes
	BtHciH4RxHandler_t RxHandler;		//!< Each queued packet, in arrival order, from BtHciH4Process
	BtHciH4RxHandler_t CmdRspHandler;	//!< Command Complete and Command Status events as soon as they are framed,
										//!< ahead of the queue. NULL queues them like any other packet.
	void (*RxReady)(BtHciH4Dev_t * const pDev);	//!< Bytes arrived, called from BtHciH4RxNotify. The owner
										//!< schedules BtHciH4Process. Keep it short.
	BtHciH4TxRoom_t TxRoom;				//!< Free TX room of the interface. Required.
	bool bPktPhy;						//!< The interface moves whole packets, not bytes: one read returns one
										//!< packet, one write takes one. Each one is the type byte followed by the
										//!< HCI packet, the same bytes H4 sends, so no framing is parsed and a
										//!< packet that is cut or not whole is dropped at once. The interface
										//!< must hold BT_HCI_H4_PKT_MAXLEN + 1 bytes per packet. TxRoom then
										//!< returns BT_HCI_H4_TXROOM_MIN when a packet fits, 0 when not.
	uint32_t TimeoutMs;					//!< Command response and TX room wait, 0 selects BT_HCI_H4_TIMEOUT_DEFAULT
	void *pCtx;							//!< Owner data, kept in the device
} BtHciH4Cfg_t;

/// Framing layer state
struct __Bt_Hci_H4_Dev {
	DevIntrf_t *pPhyIntrf;				//!< Interface the bytes go through
	hCFifo_t hPktFifo;					//!< Received packet queue
	BtHciH4RxHandler_t RxHandler;
	BtHciH4RxHandler_t CmdRspHandler;
	void (*RxReady)(BtHciH4Dev_t * const pDev);
	BtHciH4TxRoom_t TxRoom;
	bool bPktPhy;						//!< Interface moves whole packets
	void *pCtx;							//!< Owner data from the configuration
	uint32_t TimeoutMs;
	BtHciH4Pkt_t *pAsm;					//!< Queue slot the packet in progress is framed into
	uint16_t AsmIdx;					//!< Bytes of the packet in progress received
	uint16_t AsmLen;					//!< Length of the packet in progress, header included
	uint16_t SkipCnt;					//!< Bytes left of a packet being dropped
	uint8_t State;						//!< Framing state
	uint8_t HdrLen;						//!< Header length of the packet in progress
	uint8_t StageIdx;					//!< Next unread byte in Stage
	uint8_t StageCnt;					//!< Bytes in Stage
	volatile bool bPolling;				//!< Framing in progress
	volatile bool bProcessing;			//!< Dispatch in progress
	volatile bool bCmdActive;			//!< A command waits for its response
	uint32_t SyncErrCnt;				//!< Bytes skipped that were not a packet type
	uint32_t DropCnt;					//!< Packets dropped, longer than BT_HCI_H4_PKT_MAXLEN
	uint32_t TxFailCnt;					//!< Packets not sent, no TX room in time
	uint8_t Stage[32];					//!< Bytes read from the interface, not framed yet
	bool bSpareHeld;					//!< Spare holds a packet waiting for a queue slot
	BtHciH4Pkt_t Spare;					//!< Framing slot when the queue is full, so that a command
										//!< response one packet past a full queue is still reached
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Tell whether a buffer is exactly one whole HCI packet of the type.
 *
 * Type known, header present and the length the header declares equal to the
 * length given. Used by transports that receive a packet at a time.
 *
 * @param	Type	H4 packet type.
 * @param	pPkt	HCI packet without the type byte.
 * @param	Len		Bytes in pPkt.
 * @param	MaxLen	Longest packet accepted, type byte excluded.
 */
bool BtHciH4PktCheck(uint8_t Type, const uint8_t *pPkt, uint32_t Len, uint32_t MaxLen);

/**
 * @brief	Initialize the framing layer.
 *
 * The interface is not touched here, the owner initializes it.
 *
 * @param	pDev	Framing layer.
 * @param	pCfg	Configuration.
 *
 * @return	true on success. false on a missing interface, packet memory or
 * 			TxRoom, or packet memory too small or misaligned.
 */
bool BtHciH4Init(BtHciH4Dev_t * const pDev, const BtHciH4Cfg_t *pCfg);

/**
 * @brief	Tell the layer that bytes arrived.
 *
 * Call it from the receive event of the interface. It calls RxReady, nothing
 * is read here.
 *
 * @param	pDev	Framing layer.
 */
void BtHciH4RxNotify(BtHciH4Dev_t * const pDev);

/**
 * @brief	Send one HCI packet.
 *
 * The packet goes out whole or not at all. Waits up to TimeoutMs for TX room
 * in the interface. Send from one context: the one that runs
 * BtHciH4Process.
 *
 * @param	pDev	Framing layer.
 * @param	Type	H4 packet type, BT_HCI_H4_PKT_*.
 * @param	pData	HCI packet, header first, without the type byte.
 * @param	Len		Packet length, 1 to BT_HCI_H4_PKT_MAXLEN.
 *
 * @return	true when the interface took the packet.
 */
bool BtHciH4Send(BtHciH4Dev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len);

/**
 * @brief	Discard everything received and frame from the next byte.
 *
 * Empties the receive side of the interface and the packet queue. H4 has no
 * frame marker, so a byte lost or added on the line, at the controller start
 * for instance, leaves the framing out of step. Call it before an HCI Reset.
 * Ignored when called from a packet handler.
 *
 * @param	pDev	Framing layer.
 */
void BtHciH4Flush(BtHciH4Dev_t * const pDev);

/**
 * @brief	Frame the received bytes into the packet queue.
 *
 * Does not dispatch anything except Command Complete and Command Status to
 * CmdRspHandler. Stops when the interface has nothing more or when the queue
 * has no free slot, leaving the rest in the interface until a slot is freed.
 * Ignored when called again from CmdRspHandler.
 *
 * @param	pDev	Framing layer.
 */
void BtHciH4Poll(BtHciH4Dev_t * const pDev);

/**
 * @brief	Frame the received bytes and dispatch the queued packets.
 *
 * Calls RxHandler for each queued packet in arrival order. Call it from the
 * owner event queue when RxReady fires. Ignored when called again from
 * RxHandler.
 *
 * @param	pDev	Framing layer.
 * @param	MaxPkt	Packets to dispatch at most in this call, 0 for no limit.
 *
 * @return	true when packets are left because MaxPkt was reached. The owner
 * 			queues another call so other events can run in between.
 */
bool BtHciH4Process(BtHciH4Dev_t * const pDev, int MaxPkt);

/**
 * @brief	Send an HCI command and wait for its response. Host side.
 *
 * Waits for a command credit, sends the command and frames received bytes
 * until the Command Complete or Command Status of this opcode has been
 * handed to the host. The host side CmdRspHandler passes those events to
 * BtHciProcessEvent, which sets CmdDone, CmdStatus and the return parameters
 * in pHci. Other packets received meanwhile stay queued.
 *
 * Call it from the context that runs BtHciH4Process, never from an
 * interrupt: the bytes come in through the interface interrupt.
 *
 * @param	pDev		Framing layer.
 * @param	pHci		HCI host device.
 * @param	OpCode		16 bit HCI opcode.
 * @param	pParam		Command parameters, or NULL when ParamLen is 0.
 * @param	ParamLen	Parameter length in bytes.
 * @param	pRet		Buffer for the return parameters, or NULL.
 * @param	RetLen		Size of pRet in bytes.
 *
 * @return	HCI status, 0 on success. BT_HCI_H4_ERR_TIMEOUT when no response
 * 			came in time or the command could not be sent.
 * 			BT_HCI_ERR_COMMAND_DISALLOWED when called while another command
 * 			waits for its response.
 */
uint8_t BtHciH4Command(BtHciH4Dev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
					   const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen);

#ifdef __cplusplus
}

/// C++ wrapper of the same framing layer
class BtHciH4 {
public:
	BtHciH4() = default;
	BtHciH4(const BtHciH4 &) = delete;
	BtHciH4 &operator = (const BtHciH4 &) = delete;

	bool Init(const BtHciH4Cfg_t &Cfg) { return BtHciH4Init(&vDevData, &Cfg); }
	void RxNotify(void) { BtHciH4RxNotify(&vDevData); }
	bool Send(uint8_t Type, const void *pData, uint32_t Len) { return BtHciH4Send(&vDevData, Type, pData, Len); }
	void Flush(void) { BtHciH4Flush(&vDevData); }
	void Poll(void) { BtHciH4Poll(&vDevData); }
	bool Process(int MaxPkt) { return BtHciH4Process(&vDevData, MaxPkt); }
	uint8_t Command(BtHciDevice_t * const pHci, uint16_t OpCode, const void *pParam, uint8_t ParamLen,
					void *pRet, uint8_t RetLen) {
		return BtHciH4Command(&vDevData, pHci, OpCode, pParam, ParamLen, pRet, RetLen);
	}

	operator BtHciH4Dev_t * () { return &vDevData; }

private:
	BtHciH4Dev_t vDevData = {};
};
#endif

/** @} End of group Bluetooth */

#endif
