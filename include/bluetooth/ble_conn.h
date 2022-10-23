/**-------------------------------------------------------------------------
@file	ble_conn.h

@brief	Generic Bluetooth LE connection manager

Generic definitions for BLE connection manager implementation

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#ifndef __BLE_CONN_H__
#define __BLE_CONN_H__

#include <inttypes.h>

#pragma pack(push,4)

typedef struct __Ble_Conn {
	uint16_t Handle;				//!< Connection handle
	uint8_t PeerAddrType;			//!< Peer address type
	uint8_t PeerAddr[6];			//!< Address of connected peer
	uint8_t LocalResolPriAddr[6];	//!< Local resolvable private address
	uint8_t PeerResolPriAddr[6];	//!< Peer resolvable private address
	uint16_t ConnInterval;			//!< Connection interval in 1.25ms, time = ConnInterval * 1.25
	uint16_t PeriphLatency;			//!< Peripheral latency in number of connection events
	uint16_t SupervTimeout;			//!< Supervision timeout in 1.25ms
	uint8_t CentralClkAccu;			//!< Central clock accuracy PPM table
	uint16_t Mtu;					//!<
} BleConn_t;

#pragma pack(pop)


#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#endif // __BLE_CONN_H__
