/**-------------------------------------------------------------------------
@file	bt_hci_uart.h

@brief	Bluetooth HCI UART transport (H4)

HCI packets over a UART, each one preceded by its packet type byte
(Core Vol 4 Part A). The framing is the same in both directions, so one
transport serves a host talking to an external controller and a controller
answering a host.

Received packets are framed in the caller's context, never in the UART
interrupt. The interrupt only reports that bytes arrived (RxReady). The owner
then calls BtHciUartProcess from its event queue to frame and dispatch them.
The UART driver buffers the bytes in between, so its RX FIFO has to hold what
arrives while the queue is busy.

A host waits for each command response with BtHciUartCommand. It frames the
bytes itself while it waits. Command Complete and Command Status go straight
to CmdRspHandler, every other packet stays queued in arrival order for
BtHciUartProcess. A command can therefore be sent from a packet handler: the
packet being handled is not overwritten and no handler is entered twice.
When the queue is full, one more packet is framed into a spare slot. A
response right behind a full queue is still handled; a response further
back is reached only after packets are dispatched, so the command times out.
Size the queue for the packets that can arrive while a handler runs.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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
#ifndef __BT_HCI_UART_H__
#define __BT_HCI_UART_H__

#include <stdint.h>
#include <stdbool.h>

#include "cfifo.h"
#include "coredev/uart.h"
#include "bluetooth/bt_hci.h"

/** @addtogroup Bluetooth
  * @{
  */

// H4 packet type, the byte sent ahead of each HCI packet
#define BT_HCI_UART_PKT_CMD				0x01U	//!< HCI command, host to controller
#define BT_HCI_UART_PKT_ACL				0x02U	//!< ACL data, both directions
#define BT_HCI_UART_PKT_SCO				0x03U	//!< Synchronous data, both directions
#define BT_HCI_UART_PKT_EVT				0x04U	//!< HCI event, controller to host
#define BT_HCI_UART_PKT_ISO				0x05U	//!< ISO data, both directions

/// Largest HCI packet carried, packet header included, type byte excluded.
/// A received packet longer than this is dropped and counted in DropCnt.
/// The default holds a 255 byte event and a 251 byte LE ACL payload.
#ifndef BT_HCI_UART_PKT_MAXLEN
#define BT_HCI_UART_PKT_MAXLEN			BT_HCI_BUFFER_MAX_SIZE
#endif

/// Default wait for a command response and for UART TX room, in msec
#define BT_HCI_UART_TIMEOUT_DEFAULT		1000U

/// Returned by BtHciUartCommand when the controller did not answer in time or
/// the command could not be sent. HCI status codes do not reach this value.
#define BT_HCI_UART_ERR_TIMEOUT			0xFFU

/// One received packet in the packet queue
typedef struct __Bt_Hci_Uart_Packet {
	uint8_t Type;							//!< H4 packet type
	uint8_t Rsvd;
	uint16_t Len;							//!< Bytes in Data, packet header included
	uint8_t Data[BT_HCI_UART_PKT_MAXLEN];	//!< HCI packet without the type byte
} BtHciUartPkt_t;

/// Packet queue memory for NbPkt packets
#define BT_HCI_UART_PKTMEM_SIZE(NbPkt)	CFIFO_TOTAL_MEMSIZE((NbPkt), sizeof(BtHciUartPkt_t))

/// Smallest UART TX FIFO: one whole packet with its type byte. A packet is
/// written to the FIFO only when all of it fits, a packet cut short would
/// leave the other side framing from the middle of it.
#define BT_HCI_UART_TXFIFO_MIN			(BT_HCI_UART_PKT_MAXLEN + 1)

typedef struct __Bt_Hci_Uart_Dev	BtHciUartDev_t;

/**
 * @brief	Received packet handler.
 *
 * @param	pDev	Transport.
 * @param	Type	H4 packet type, BT_HCI_UART_PKT_*.
 * @param	pPkt	HCI packet, header first, without the type byte. Valid
 * 					only during the call.
 * @param	Len		Packet length in bytes.
 */
typedef void (*BtHciUartRxHandler_t)(BtHciUartDev_t * const pDev, uint8_t Type,
									 uint8_t *pPkt, uint16_t Len);

typedef struct __Bt_Hci_Uart_Config {
	const UARTCfg_t *pUartCfg;			//!< UART to the other side. EvtCallback is replaced by the transport,
										//!< interrupt mode and blocking FIFOs are forced. The TX FIFO must hold
										//!< BT_HCI_UART_TXFIFO_MIN bytes. Hardware flow control is recommended.
	uint8_t *pPktMem;					//!< Packet queue memory, BT_HCI_UART_PKTMEM_SIZE(n), word aligned
	uint32_t PktMemSize;				//!< Packet queue memory size in bytes
	BtHciUartRxHandler_t RxHandler;		//!< Each queued packet, in arrival order, from BtHciUartProcess
	BtHciUartRxHandler_t CmdRspHandler;	//!< Command Complete and Command Status events as soon as they are framed,
										//!< ahead of the queue. NULL queues them like any other packet.
	void (*RxReady)(BtHciUartDev_t * const pDev);	//!< Bytes arrived, called from the UART interrupt.
										//!< The owner schedules BtHciUartProcess. Keep it short.
	uint32_t TimeoutMs;					//!< Command response and TX room wait, 0 selects BT_HCI_UART_TIMEOUT_DEFAULT
	void *pCtx;							//!< Owner data, kept in the device
} BtHciUartCfg_t;

/// Transport state. Uart must stay the first member: the UART event handler
/// gets the UART device and finds the transport from it.
struct __Bt_Hci_Uart_Dev {
	UARTDev_t Uart;						//!< UART device
	hCFifo_t hPktFifo;					//!< Received packet queue
	BtHciUartRxHandler_t RxHandler;
	BtHciUartRxHandler_t CmdRspHandler;
	void (*RxReady)(BtHciUartDev_t * const pDev);
	void *pCtx;							//!< Owner data from the configuration
	uint32_t TimeoutMs;
	BtHciUartPkt_t *pAsm;				//!< Queue slot the packet in progress is framed into
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
	uint32_t DropCnt;					//!< Packets dropped, longer than BT_HCI_UART_PKT_MAXLEN
	uint32_t TxFailCnt;					//!< Packets not sent, no TX room in time
	uint8_t Stage[32];					//!< Bytes read from the UART, not framed yet
	bool bSpareHeld;					//!< Spare holds a packet waiting for a queue slot
	BtHciUartPkt_t Spare;				//!< Framing slot when the queue is full, so that a command
										//!< response one packet past a full queue is still reached
};

#ifdef __cplusplus
extern "C" {
#endif

/// UART to the controller. Defined by the application when its Bluetooth port
/// runs the host over this transport, as the nRF91 port does. Define it in a
/// file that includes this header, the declaration gives it its linkage.
extern const UARTCfg_t g_BtHciUartCfg;

/**
 * @brief	Initialize the transport and its UART.
 *
 * @param	pDev	Transport.
 * @param	pCfg	Configuration.
 *
 * @return	true on success. false on a missing UART configuration, packet
 * 			memory too small or misaligned, UART initialization failure or a
 * 			UART TX FIFO smaller than BT_HCI_UART_TXFIFO_MIN.
 */
bool BtHciUartInit(BtHciUartDev_t * const pDev, const BtHciUartCfg_t *pCfg);

/**
 * @brief	Send one HCI packet.
 *
 * The packet goes out whole or not at all. Waits up to TimeoutMs for room
 * in the UART TX FIFO. Send from one context: the one that runs
 * BtHciUartProcess.
 *
 * @param	pDev	Transport.
 * @param	Type	H4 packet type, BT_HCI_UART_PKT_*.
 * @param	pData	HCI packet, header first, without the type byte.
 * @param	Len		Packet length, 1 to BT_HCI_UART_PKT_MAXLEN.
 *
 * @return	true when the packet is in the UART TX FIFO.
 */
bool BtHciUartSend(BtHciUartDev_t * const pDev, uint8_t Type, const void *pData, uint32_t Len);

/**
 * @brief	Discard everything received and frame from the next byte.
 *
 * Empties the UART RX FIFO and the packet queue. H4 has no frame marker, so a
 * byte lost or added on the line, at the controller start for instance,
 * leaves the framing out of step. Call it before an HCI Reset. Ignored when
 * called from a packet handler.
 *
 * @param	pDev	Transport.
 */
void BtHciUartFlush(BtHciUartDev_t * const pDev);

/**
 * @brief	Frame the received bytes into the packet queue.
 *
 * Does not dispatch anything except Command Complete and Command Status to
 * CmdRspHandler. Stops when the UART has nothing more or when the queue
 * has no free slot, leaving the rest in the UART until a slot is freed.
 * Ignored when called again from CmdRspHandler.
 *
 * @param	pDev	Transport.
 */
void BtHciUartPoll(BtHciUartDev_t * const pDev);

/**
 * @brief	Frame the received bytes and dispatch the queued packets.
 *
 * Calls RxHandler for each queued packet in arrival order. Call it from the
 * owner event queue when RxReady fires. Ignored when called again from
 * RxHandler.
 *
 * @param	pDev	Transport.
 * @param	MaxPkt	Packets to dispatch at most in this call, 0 for no limit.
 *
 * @return	true when packets are left because MaxPkt was reached. The owner
 * 			queues another call so other events can run in between.
 */
bool BtHciUartProcess(BtHciUartDev_t * const pDev, int MaxPkt);

/**
 * @brief	Send an HCI command and wait for its response. Host side.
 *
 * Waits for a command credit, sends the command and frames received bytes
 * until the Command Complete or Command Status of this opcode has been
 * handed to the host. The host side CmdRspHandler passes those events to
 * BtHciProcessEvent, which sets CmdDone, CmdStatus and the return parameters
 * in pHci. Other packets received meanwhile stay queued.
 *
 * Call it from the context that runs BtHciUartProcess, never from an
 * interrupt: the bytes come in through the UART interrupt.
 *
 * @param	pDev		Transport.
 * @param	pHci		HCI host device.
 * @param	OpCode		16 bit HCI opcode.
 * @param	pParam		Command parameters, or NULL when ParamLen is 0.
 * @param	ParamLen	Parameter length in bytes.
 * @param	pRet		Buffer for the return parameters, or NULL.
 * @param	RetLen		Size of pRet in bytes.
 *
 * @return	HCI status, 0 on success. BT_HCI_UART_ERR_TIMEOUT when no response
 * 			came in time or the command could not be sent.
 * 			BT_HCI_ERR_COMMAND_DISALLOWED when called while another command
 * 			waits for its response.
 */
uint8_t BtHciUartCommand(BtHciUartDev_t * const pDev, BtHciDevice_t * const pHci, uint16_t OpCode,
						 const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen);

#ifdef __cplusplus
}

/// C++ wrapper of the same transport
class BtHciUart {
public:
	BtHciUart() = default;
	BtHciUart(const BtHciUart &) = delete;
	BtHciUart &operator = (const BtHciUart &) = delete;

	bool Init(const BtHciUartCfg_t &Cfg) { return BtHciUartInit(&vDevData, &Cfg); }
	bool Send(uint8_t Type, const void *pData, uint32_t Len) { return BtHciUartSend(&vDevData, Type, pData, Len); }
	void Flush(void) { BtHciUartFlush(&vDevData); }
	void Poll(void) { BtHciUartPoll(&vDevData); }
	bool Process(int MaxPkt) { return BtHciUartProcess(&vDevData, MaxPkt); }
	uint8_t Command(BtHciDevice_t * const pHci, uint16_t OpCode, const void *pParam, uint8_t ParamLen,
					void *pRet, uint8_t RetLen) {
		return BtHciUartCommand(&vDevData, pHci, OpCode, pParam, ParamLen, pRet, RetLen);
	}

	operator BtHciUartDev_t * () { return &vDevData; }
	operator UARTDev_t * () { return &vDevData.Uart; }

private:
	BtHciUartDev_t vDevData = {};
};
#endif

/** @} End of group Bluetooth */

#endif
