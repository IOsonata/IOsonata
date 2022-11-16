/**-------------------------------------------------------------------------
@file	bt_att.h

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
#ifndef __BT_ATT_H__
#define __BT_ATT_H__

#include <inttypes.h>

#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_dev.h"

#define BT_ATT_OPCODE_FLAG_COMMAND						(1<<6)
#define BT_ATT_OPCODE_FLAG_AUTH_SIGNATURE				(1<<7)


#define BT_ATT_OPCODE_ATT_ERROR_RSP						1		//!< Error
#define BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ				2		//!< Client Rx MTU
#define BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP				3		//!< Server Rx MTU
#define BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ			4		//!<
#define BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP			5		//!< Format information data
#define BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ		6		//!< Starting handle
#define BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP		7		//!< Handle information list
#define BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ				8		//!< Handle information list
#define BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP				9		//!< Length, attribute data list
#define BT_ATT_OPCODE_ATT_READ_REQ						0xA		//!< Attribute handle
#define BT_ATT_OPCODE_ATT_READ_RSP						0xB		//!< Attribute value
#define BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ				0xC		//!< Attribute handle
#define BT_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP				0xD		//!< Part attribute value
#define BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ				0xE		//!< Handle set
#define BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP				0xF		//!< Value set
#define BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ		0x10	//!< Start handle, End handle, UUID
#define BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP		0x11	//!< Length, Attribute data list
#define BT_ATT_OPCODE_ATT_WRITE_REQ						0x12	//!< Attribute Handle, Attribute Value
#define BT_ATT_OPCODE_ATT_WRITE_RSP						0x13	//!<
#define BT_ATT_OPCODE_ATT_CMD							0x52	//!< Attribute handle, attribute value.
#define BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ				0x18	//!<
#define BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP				0x19
#define BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ	0x20	//!< Set of handle
#define BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP	0x21	//!< Length of value tuple list
#define BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF		0x23	//!< Handle Length Value Tuple List
#define BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF				0x1B	//!<
#define BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND				0x1D
#define BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM				0x1E
#define BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD				0xD2

#define BT_ATT_ERROR_INVALID_HANDLE						1		//!< Invalid handle
#define BT_ATT_ERROR_READ_NOT_PERMITTED					2		//!< Read not permitted
#define BT_ATT_ERROR_WRITE_NOT_PERMITTED				3		//!< Write not permitted
#define BT_ATT_ERROR_INVALID_PDU						4		//!< Invalid PDU
#define BT_ATT_ERROR_INSUF_AUTHEN						5		//!< Insufficient authentication
#define BT_ATT_ERROR_REQUEST_NOT_SUPP					6		//!< Request not supported
#define BT_ATT_ERROR_INVALID_OFFSET						7		//!< Invalid offset
#define BT_ATT_ERROR_INSUF_AUTHOR						8		//!< Insufficient authorization
#define BT_ATT_ERROR_PREPARE_QUE_FULL					9		//!< Pepare writes que full
#define BT_ATT_ERROR_ATT_NOT_FOUND						10		//!< Attribute not found
#define BT_ATT_ERROR_ATT_NOT_LONG						11		//!< Attribute cannot be read using ATT_READ_BLOB_REQ
#define BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT				12		//!< Encryption key size too short
#define BT_ATT_ERROR_INVALID_ATT_VALUE					13		//!< Invalide attribute value
#define BT_ATT_ERROR_UNLIKELY_ERROR						14		//!< Unknown error
#define BT_ATT_ERROR_INSUF_ENCRYPT						15		//!< Require encryption
#define BT_ATT_ERROR_UNSUPP_GROUP_TYPE					16		//!< Unsupported group type
#define BT_ATT_ERROR_INSUF_RESOURCE						17		//!< Insufficient resources
#define BT_ATT_ERROR_DB_OUT_SYNC						19		//!< Database out of sync
#define BT_ATT_ERROR_VALUE_NOT_ALLOWED					20		//!< Value not allowed

#pragma pack(push, 1)

typedef struct __Bt_Att_Hdl_Uuid_16 {
	uint16_t Hdl;
	uint16_t Uuid;
} BtAttHdlUuid16_t;

typedef struct __Bt_Att_Hdl_Uuid_128 {
	uint16_t Hdl;
	uint8_t Uuid[16];
} BtAttHdlUuid128_t;

typedef struct __Bt_Att_Handle_Range {
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
} BtAttHdlRange_t;

/// Error response : ATT_OPCODE_ATT_ERROR_RSP
typedef struct __Bt_Att_ErrorRsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t ReqOpCode;		//!< Request opcode in error
	uint16_t Hdl;			//!< Start handle in error
	uint8_t Error;			//!< Error code
} BtAttErrorRsp_t;

/// Exchange MTU request/response : ATT_EXCHANGE_MTU_REQ, ATT_EXCHANGE_MTU_RSP
typedef struct __Bt_Att_Exchange_Mtu_Req_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t RxMtu;			//!< Receive MTU size
} BtAttExchgMtuReqRsp_t;

/// Find information request : ATT_FIND_INFO_REQ
typedef struct __Bt_Att_Find_Info_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
} BtAttFindInfoReq_t;

/// Find information response : ATT_FIND_INFO_RSP
#define BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16			1
#define BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128			2

/// NOTE: Variable length
typedef struct __Bt_Att_Find_Info_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Fmt;			//!< Format
	union {
		BtAttHdlUuid16_t HdlUuid16[1];
		BtAttHdlUuid128_t HdlUuid128[1];
	};
} BtAttFindInfoRsp_t;

/// Find by type value request : ATT_FIND_BY_TYPE_VALUE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Find_By_Type_Value_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	uint16_t Type;			//!< Attribute type 2 octet UUID to find
	uint8_t Val[1];			//!< Value to find var length
} BtAttFindByTypeValueReq_t;

/// Find by type value response : ATT_FIND_BY_TYPE_VALUE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Find_By_Type_Value_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	BtAttHdlRange_t Hdl[1];	//!< Array of handles
} BtAttFindByTypeValueRsp_t;

/// Read by type request : ATT_READ_BY_TYPE_REQ
typedef struct __Bt_Att_Read_By_Type_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	BtUuidVal_t Uuid;		//!< UUID
} BtAttReadByTypeReq_t;

/// Read by type response : ATT_READ_BY_TYPE_RSP
/// NOTE: variable length
typedef struct __Bt_Att_Read_By_Type_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Len;			//!< The size of each attribute handle- value pair
	uint8_t Data[1];
} BtAttReadByTypeRsp_t;

/// Read request : ATT_READ_REQ
typedef struct __Bt_Att_Read_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to be read
} BtAttReadReq_t;

/// Read response : ATT_READ_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttReadRsp_t;

/// Read blob request : ATT_READ_BLOB_REQ
typedef struct __Bt_Att_Read_Blob_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to be read
	uint16_t Offset;		//!< Offset location of data to read
} BtAttBlobReq_t;

/// Read blob response : ATT_READ_BLOB_RSP
typedef struct __Bt_Att_Read_Blob_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttBlobRsp_t;

/// Read multiple request : ATT_READ_MULTIPLE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Multiple_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl[1];		//!< Array of attribute handle to be read
} BtAttReadMultipleReq_t;

/// Read multiple response : ATT_READ_MULTIPLE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Multiple_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttReadMultipleRsp_t;

/// Read by group type request : ATT_READ_BY_GROUP_TYPE_REQ
typedef struct __Bt_Att_Read_By_Group_Type_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	BtUuidVal_t Uid;		//!< UUID
} BtAttReadByGroupTypeReq_t;

/// Read by group type response : ATT_READ_BY_GROUP_TYPE_RSP

typedef struct __Bt_Att_Read_By_Group_Type_Rsp_Uuid16 {
	uint16_t HdlStart;
	uint16_t HdlEnd;
	uint16_t Uuid;
} BtAttReadByGroupTypeRspUuid16_t;

typedef struct __Bt_Att_Read_By_Group_Type_Rsp_Uuid128 {
	uint16_t HdlStart;
	uint16_t HdlEnd;
	uint8_t Uuid[16];
} BtAttReadByGroupTypeRspUuid128_t;

/// NOTE: Variable length
typedef struct __Bt_Att_Read_By_Group_Type_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Len;			//!< Size of each attribute data
	uint8_t Data[1];		//!< Array of attribute data returned
} BtAttReadByGroupTypeRsp_t;

/// Read multiple variable request : ATT_READ_MULTIPLE_VARIABLE_REQ
typedef struct __Bt_Att_Read_Multiple_Variable_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl[1];		//!< Array of attribute handle to be read
} BtAttReadMultipleVarReq_t;

/// Read multiple variable response : ATT_READ_MULTIPLE_VARIABLE_RSP
typedef struct __Bt_Att_Read_Multiple_Variable_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Array of variable attribute data returned. Require parsing
} BtAttReadMultipleVarRsp_t;

/// Write request : ATT_WRITE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Write_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttWriteReq_t;

/// Write response : ATT_WRITE_RSP
typedef struct __Bt_Att_Write_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
} BtAttWriteRsp_t;

/// Write command : ATT_WRITE_CMD
/// NOTE: Variable length
typedef struct __Bt_Att_Write_Cmd {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttWriteCmd_t;

/// Signed write command : ATT_SIGNED_WRITE_CMD
/// NOTE: Variable length
typedef struct __Bt_Att_Signed_Write_Cmd {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[12];		//!< Variable length data to write along with 12 bytes authen signature
} BtAttSignedWriteCmd_t;

/// Prepare write request : ATT_PREPARE_WRITE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Prepare_Write_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint16_t Offset;		//!< Offset of attribute data to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttPrepareWriteReq_t;

/// Prepare write response : ATT_PREPARE_WRITE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Prepare_Write_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint16_t Offset;		//!< Offset of attribute data to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttPrepareWriteRsp_t;

/// Execute write request : ATT_EXECUTE_WRITE_REQ
typedef struct __Bt_Att_Execute_Write_Req {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Flag;			//!< 0 - Cancel prepare writes, 1 - Execute write
} BtAttExecuteWriteReq_t;

/// Execute write response : ATT_EXECUTE_WRITE_RSP
typedef struct __Bt_Att_Execute_Write_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
} BtAttExecuteWriteRsp_t;

// Server initiated

/// Handle value NTF : ATT_HANDLE_VALUE_NTF
/// NOTE: Variable length
typedef struct __Bt_Att_Handle_Value_Ntf {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle
	uint8_t Data[1];		//!< Variable length data
} BtAttHandleValueNtf_t;

/// Handle value IND : ATT_HANDLE_VALUE_IND
/// NOTE: Variable length
typedef struct __Bt_Att_Handle_Value_Ind {
	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle
	uint8_t Data[1];		//!< Variable length data
} BtAttHandleValueInd_t;

/// Handle value CFM : ATT_HANDLE_VALUE_CFM
typedef struct __Bt_Att_Handle_Value_Cfm {
	uint8_t OpCode;			//!< Attribute opcode
} BtAttHandleValueCfm_t;

/// Multiple handle value NTF : ATT_MULTIPLE_VALUE_NTF
/// NOTE: Variable length
typedef struct __Bt_Att_Multiple_Handle_Value_Ntf {
	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Variable length data
} BtAttMultipleHandleValueNtf_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BtProcessAttData(BtDev_t * const pDev, uint16_t ConnHdl, BtL2CapPdu_t * const pRcvPdu);

#ifdef __cplusplus
}
#endif

#endif // __BT_ATT_H__
