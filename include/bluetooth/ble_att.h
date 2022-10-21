/**-------------------------------------------------------------------------
@file	ble_att.h

@brief	Generic Bluetooth ATT protocol

Generic definitions for Bluetooth Attribute Protocol implementation

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2022

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
#ifndef __BLE_ATT_H__
#define __BLE_ATT_H__

#include <inttypes.h>

#include "bluetooth/ble_uuid.h"

#define BLE_ATT_OPCODE_ATT_ERROR_RSP					1		//!< Error
#define BLE_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ				2		//!< Client Rx MTU
#define BLE_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP				3		//!< Server Rx MTU
#define BLE_ATT_OPCODE_ATT_FIND_INFO_REQ				4		//!<
#define BLE_ATT_OPCODE_ATT_FIND_INFO_RSP				5		//!< Format information data
#define BLE_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ		6		//!< Starting handle
#define BLE_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP		7		//!< Handle information list
#define BLE_ATT_OPCODE_ATT_READ_BY_TYPE_REQ				8		//!< Handle information list
#define BLE_ATT_OPCODE_ATT_READ_BY_TYPE_RSP				9		//!< Length, attribute data list
#define BLE_ATT_OPCODE_ATT_READ_REQ						0xA		//!< Attribute handle
#define BLE_ATT_OPCODE_ATT_READ_RSP						0xB		//!< Attribute value
#define BLE_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ			0xC		//!< Attribute handle
#define BLE_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP			0xD		//!< Part attribute value
#define BLE_ATT_OPCODE_ATT_READ_MULTIPLE_REQ			0xE		//!< Handle set
#define BLE_ATT_OPCODE_ATT_READ_MULTIPLE_RSP			0xF		//!< Value set
#define BLE_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ		0x10	//!< Start handle, End handle, UUID
#define BLE_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP		0x11	//!< Length, Attribute data list
#define BLE_ATT_OPCODE_ATT_WRITE_REQ					0x12	//!< Attribute Handle, Attribute Value
#define BLE_ATT_OPCODE_ATT_WRITE_RSP					0x13	//!<
#define BLE_ATT_OPCODE_ATT_CMD							0x52	//!< Attribute handle, attribute value.
#define BLE_ATT_OPCODE_ATT_PREPARE_WRITE_REQ			0x18	//!<
#define BLE_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP			0x19
#define BLE_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ	0x20	//!< Set of handle
#define BLE_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP	0x21	//!< Length of value tuple list
#define BLE_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF	0x23	//!< Handle Length Value Tuple List
#define BLE_ATT_OPCODE_ATT_HANDLE_VALUE_NTF				0x1B	//!<
#define BLE_ATT_OPCODE_ATT_HANDLE_VALUE_IND				0x1D
#define BLE_ATT_OPCODE_ATT_HANDLE_VALUE_CFM				0x1E
#define BLE_ATT_OPCODE_ATT_SIGNED_WRITE_CMD				0xD2

#define BLE_ATT_ERROR_INVALID_HANDLE					1		//!< Invalid handle
#define BLE_ATT_ERROR_READ_NOT_PERMITTED				2		//!< Read not permitted
#define BLE_ATT_ERROR_WRITE_NOT_PERMITTED				3		//!< Write not permitted
#define BLE_ATT_ERROR_INVALID_PDU						4		//!< Invalid PDU
#define BLE_ATT_ERROR_INSUF_AUTHEN						5		//!< Insufficient authentication
#define BLE_ATT_ERROR_REQUEST_NOT_SUPP					6		//!< Request not supported
#define BLE_ATT_ERROR_INVALID_OFFSET					7		//!< Invalid offset
#define BLE_ATT_ERROR_INSUF_AUTHOR						8		//!< Insufficient authorization
#define BLE_ATT_ERROR_PREPARE_QUE_FULL					9		//!< Pepare writes que full
#define BLE_ATT_ERROR_ATT_NOT_FOUND						10		//!< Attribute not found
#define BLE_ATT_ERROR_ATT_NOT_LONG						11		//!< Attribute cannot be read using ATT_READ_BLOB_REQ
#define BLE_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT				12		//!< Encryption key size too short
#define BLE_ATT_ERROR_INVALID_ATT_VALUE					13		//!< Invalide attribute value
#define BLE_ATT_ERROR_UNLIKELY_ERROR					14		//!< Unknown error
#define BLE_ATT_ERROR_INSUF_ENCRYPT						15		//!< Require encryption
#define BLE_ATT_ERROR_UNSUPP_GROUP_TYPE					16		//!< Unsupported group type
#define BLE_ATT_ERROR_INSUF_RESOURCE					17		//!< Insufficient resources
#define BLE_ATT_ERROR_DB_OUT_SYNC						19		//!< Database out of sync
#define BLE_ATT_ERROR_VALUE_NOT_ALLOWED					20		//!< Value not allowed

#pragma pack(push, 1)

/// Error response : BLE_ATT_OPCODE_ATT_ERROR_RSP
typedef struct __Ble_Att_ErrorRsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t ReqOpCode;		//!< Request opcode in error
	uint16_t Hdl;			//!< Start handle in error
	uint8_t Error;			//!< Error code
} BleAttErrorRsp_t;

/// Exchange MTU request : BLE_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, BLE_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP
typedef struct __Ble_Att_Exchange_Mtu_Req_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t RxMtu;			//!< Receive MTU size
} BleAttExchgMtuReqRsp_t;

/// Find information request : BLE_ATT_OPCODE_ATT_FIND_INFO_REQ
typedef struct __Ble_Att_FInd_Info_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
} BleAttFindInfoReq_t;

/// Find information request : BLE_ATT_OPCODE_ATT_FIND_INFO_RSP
/// NOTE: Variable length
typedef struct __Ble_Att_FInd_Info_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Fmt;			//!< Format
	uint8_t Data[1];		//!< Array of Handle/UUID pair
} BleAttFindInfoRsp_t;

/// Find by type value request : BLE_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ
/// NOTE: Variable length
typedef struct __Ble_Att_Find_Byt_Type_Value_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	uint16_t Type;			//!< Attribute type 2 octet UUID to find
	uint8_t Val[1];			//!< Value to find
} BleAttFindByTypeValueRsq_t;

/// Read by group type request : BLE_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ
typedef struct __Ble_Att_Read_By_Group_Type_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	BleUuid_t Uid;			//!< UUID
} BleAttReadByGroupTypeReq_t;


#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // __BLE_ATT_H__
