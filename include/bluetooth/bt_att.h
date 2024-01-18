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
#include <stddef.h>

#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_hci.h"

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
#define BT_ATT_OPCODE_ATT_READ_BLOB_REQ					0xC		//!< Attribute handle
#define BT_ATT_OPCODE_ATT_READ_BLOB_RSP					0xD		//!< Part attribute value
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
#define BT_ATT_ERROR_PREPARE_QUE_FULL					9		//!< Prepare writes que full
#define BT_ATT_ERROR_ATT_NOT_FOUND						10		//!< Attribute not found
#define BT_ATT_ERROR_ATT_NOT_LONG						11		//!< Attribute cannot be read using ATT_READ_BLOB_REQ
#define BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT				12		//!< Encryption key size too short
#define BT_ATT_ERROR_INVALID_ATT_VALUE					13		//!< Invalid attribute value
#define BT_ATT_ERROR_UNLIKELY_ERROR						14		//!< Unknown error
#define BT_ATT_ERROR_INSUF_ENCRYPT						15		//!< Require encryption
#define BT_ATT_ERROR_UNSUPP_GROUP_TYPE					16		//!< Unsupported group type
#define BT_ATT_ERROR_INSUF_RESOURCE						17		//!< Insufficient resources
#define BT_ATT_ERROR_DB_OUT_SYNC						19		//!< Database out of sync
#define BT_ATT_ERROR_VALUE_NOT_ALLOWED					20		//!< Value not allowed

#define BT_ATT_PERMISSION_READ
#define BT_ATT_PERMISSION_WRITE

#define BT_ATT_MTU_MIN				23
#define BT_ATT_MTU_MAX				247

#define BT_ATT_HANDLE_INVALID		0xFFFFU

typedef struct __Bt_Characteristic		BtGattChar_t;
typedef struct __Bt_Service				BtGattSrvc_t;
typedef struct __Bt_Characteristic		BtChar_t;
typedef struct __Bt_Service				BtSrvc_t;

typedef struct __Bt_Att_DB_Entry		BtAttDBEntry_t;

/**
 * @brief	Callback on write
 */
typedef void (*BtCharWrCb_t) (BtChar_t *pChar, uint8_t *pData, int Offset, int Len);

/**
 * @brief	Callback on set notification
 */
typedef void (*BtCharSetNotifCb_t) (BtChar_t *pChar, bool bEnable);

/**
 * @brief	Callback on set indication
 */
typedef void (*BtCharSetIndCb_t) (BtChar_t *pChar, bool bEnable);

/**
 * @brief	Callback when transmission is completed
 *
 * @param	pChar
 * @param	CharIdx
 */
typedef void (*BtCharTxComplete_t) (BtChar_t *pChar, int CharIdx);

/**
 * @brief	Callback on authorization request
 *
 * @param	pBleSvc
 * @param	evt
 */
typedef void (*BtSrvcAuthRqst_t)(BtSrvc_t *pBleSvc, uint32_t evt);

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
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t ReqOpCode;		//!< Request opcode in error
	uint16_t Hdl;			//!< Start handle in error
	uint8_t Error;			//!< Error code
} BtAttErrorRsp_t;

/// Exchange MTU request/response : ATT_EXCHANGE_MTU_REQ, ATT_EXCHANGE_MTU_RSP
typedef struct __Bt_Att_Exchange_Mtu_Req_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t RxMtu;			//!< Receive MTU size
} BtAttExchgMtuReqRsp_t;

/// Find information request : ATT_FIND_INFO_REQ
typedef struct __Bt_Att_Find_Info_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
} BtAttFindInfoReq_t;

/// Find information response : ATT_FIND_INFO_RSP
#define BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16			1
#define BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128			2

/// NOTE: Variable length
typedef struct __Bt_Att_Find_Info_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Fmt;			//!< Format
	union {
		BtAttHdlUuid16_t HdlUuid16[1];
		BtAttHdlUuid128_t HdlUuid128[1];
	};
} BtAttFindInfoRsp_t;

/// Find by type value request : ATT_FIND_BY_TYPE_VALUE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Find_By_Type_Value_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	uint16_t Type;			//!< Attribute type 2 octet UUID to find
	uint8_t Val[1];			//!< Value to find var length
} BtAttFindByTypeValueReq_t;

/// Find by type value response : ATT_FIND_BY_TYPE_VALUE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Find_By_Type_Value_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	BtAttHdlRange_t Hdl[1];	//!< Array of handles
} BtAttFindByTypeValueRsp_t;

/// Read by type request : ATT_READ_BY_TYPE_REQ
typedef struct __Bt_Att_Read_By_Type_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	BtUuidVal_t Uuid;		//!< UUID
} BtAttReadByTypeReq_t;

/// Read by type response : ATT_READ_BY_TYPE_RSP
/// NOTE: variable length
typedef struct __Bt_Att_Read_By_Type_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Len;			//!< The size of each attribute handle- value pair
	uint8_t Data[1];
} BtAttReadByTypeRsp_t;

/// Read request : ATT_READ_REQ
typedef struct __Bt_Att_Read_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to be read
} BtAttReadReq_t;

/// Read response : ATT_READ_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttReadRsp_t;

/// Read blob request : ATT_READ_BLOB_REQ
typedef struct __Bt_Att_Read_Blob_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to be read
	uint16_t Offset;		//!< Offset location of data to read
} BtAttReadBlobReq_t;

/// Read blob response : ATT_READ_BLOB_RSP
typedef struct __Bt_Att_Read_Blob_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttReadBlobRsp_t;

/// Read multiple request : ATT_READ_MULTIPLE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Multiple_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl[1];		//!< Array of attribute handle to be read
} BtAttReadMultipleReq_t;

/// Read multiple response : ATT_READ_MULTIPLE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Read_Multiple_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Attribute values returned
} BtAttReadMultipleRsp_t;

/// Read by group type request : ATT_READ_BY_GROUP_TYPE_REQ
typedef struct __Bt_Att_Read_By_Group_Type_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t StartHdl;		//!< Start handle
	uint16_t EndHdl;		//!< End handle
	BtUuidVal_t Uuid;		//!< UUID
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
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Len;			//!< Size of each attribute data
	uint8_t Data[1];		//!< Array of attribute data returned
} BtAttReadByGroupTypeRsp_t;

/// Read multiple variable request : ATT_READ_MULTIPLE_VARIABLE_REQ
typedef struct __Bt_Att_Read_Multiple_Variable_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl[1];		//!< Array of attribute handle to be read
} BtAttReadMultipleVarReq_t;

/// Read multiple variable response : ATT_READ_MULTIPLE_VARIABLE_RSP
typedef struct __Bt_Att_Read_Multiple_Variable_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Array of variable attribute data returned. Require parsing
} BtAttReadMultipleVarRsp_t;

/// Write request : ATT_WRITE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Write_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttWriteReq_t;

/// Write response : ATT_WRITE_RSP
typedef struct __Bt_Att_Write_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
} BtAttWriteRsp_t;

/// Write command : ATT_WRITE_CMD
/// NOTE: Variable length
typedef struct __Bt_Att_Write_Cmd {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttWriteCmd_t;

/// Signed write command : ATT_SIGNED_WRITE_CMD
/// NOTE: Variable length
typedef struct __Bt_Att_Signed_Write_Cmd {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint8_t Data[12];		//!< Variable length data to write along with 12 bytes authen signature
} BtAttSignedWriteCmd_t;

/// Prepare write request : ATT_PREPARE_WRITE_REQ
/// NOTE: Variable length
typedef struct __Bt_Att_Prepare_Write_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint16_t Offset;		//!< Offset of attribute data to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttPrepareWriteReq_t;

/// Prepare write response : ATT_PREPARE_WRITE_RSP
/// NOTE: Variable length
typedef struct __Bt_Att_Prepare_Write_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle to write
	uint16_t Offset;		//!< Offset of attribute data to write
	uint8_t Data[1];		//!< Variable length data to write
} BtAttPrepareWriteRsp_t;

/// Execute write request : ATT_EXECUTE_WRITE_REQ
typedef struct __Bt_Att_Execute_Write_Req {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Flag;			//!< 0 - Cancel prepare writes, 1 - Execute write
} BtAttExecuteWriteReq_t;

/// Execute write response : ATT_EXECUTE_WRITE_RSP
typedef struct __Bt_Att_Execute_Write_Rsp {
//	uint8_t OpCode;			//!< Attribute opcode
} BtAttExecuteWriteRsp_t;

// Server initiated

/// Handle value NTF : ATT_HANDLE_VALUE_NTF
/// NOTE: Variable length
typedef struct __Bt_Att_Handle_Value_Ntf {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t ValHdl;		//!< Attribute value handle
	uint8_t Data[1];		//!< Variable length data
} BtAttHandleValueNtf_t;

/// Handle value IND : ATT_HANDLE_VALUE_IND
/// NOTE: Variable length
typedef struct __Bt_Att_Handle_Value_Ind {
//	uint8_t OpCode;			//!< Attribute opcode
	uint16_t Hdl;			//!< Attribute handle
	uint8_t Data[1];		//!< Variable length data
} BtAttHandleValueInd_t;

/// Handle value CFM : ATT_HANDLE_VALUE_CFM
typedef struct __Bt_Att_Handle_Value_Cfm {
//	uint8_t OpCode;			//!< Attribute opcode
} BtAttHandleValueCfm_t;

/// Multiple handle value NTF : ATT_MULTIPLE_VALUE_NTF
/// NOTE: Variable length
typedef struct __Bt_Att_Multiple_Handle_Value_Ntf {
//	uint8_t OpCode;			//!< Attribute opcode
	uint8_t Data[1];		//!< Variable length data
} BtAttMultipleHandleValueNtf_t;

typedef struct __Bt_Attribute_Req_Rsp {
	uint8_t OpCode;			//!< Attribute opcode
	union {
		BtAttErrorRsp_t ErrorRsp;
		BtAttExchgMtuReqRsp_t ExchgMtuReqRsp;
		BtAttFindInfoReq_t FindInfoReq;
		BtAttFindInfoRsp_t FindInfoRsp;
		BtAttFindByTypeValueReq_t FindByTypeValueReq;
		BtAttFindByTypeValueRsp_t FindByTypeValueRsp;
		BtAttReadByTypeReq_t ReadByTypeReq;
		BtAttReadByTypeRsp_t ReadByTypeRsp;
		BtAttReadReq_t ReadReq;
		BtAttReadRsp_t ReadRsp;
		BtAttReadBlobReq_t ReadBlobReq;
		BtAttReadBlobRsp_t ReadBlobRsp;
		BtAttReadMultipleReq_t ReadMultipleReq;
		BtAttReadMultipleRsp_t ReadMultipleRsp;
		BtAttReadByGroupTypeReq_t ReadByGroupTypeReq;
		BtAttReadByGroupTypeRspUuid16_t ReadByGroupTypeRspUuid16;
		BtAttReadByGroupTypeRspUuid128_t ReadByGroupTypeRspUuid128;
		BtAttReadByGroupTypeRsp_t ReadByGroupTypeRsp;
		BtAttReadMultipleVarReq_t ReadMultipleVarReq;
		BtAttReadMultipleVarRsp_t ReadMultipleVarRsp;
		BtAttWriteReq_t WriteReq;
		BtAttWriteRsp_t WriteRsp;
		BtAttWriteCmd_t WriteCmd;
		BtAttSignedWriteCmd_t SignedWriteCmd;
		BtAttPrepareWriteReq_t PrepareWriteReq;
		BtAttPrepareWriteRsp_t PrepareWriteRsp;
		BtAttExecuteWriteReq_t ExecuteWriteReq;
		BtAttExecuteWriteRsp_t ExecuteWriteRsp;
		BtAttHandleValueNtf_t HandleValueNtf;
		BtAttHandleValueInd_t HandleValueInd;
		BtAttHandleValueCfm_t HandleValueCfm;
		BtAttMultipleHandleValueNtf_t MultipleHandleValueNtf;
	};
} BtAttReqRsp_t;

// Service attribute : type UUID 0x2800 Primary, 0x2801 Secondary
typedef struct __Bt_Att_Srvc_Declar {
	BtUuid_t Uuid;						//!< Service UUID
	BtSrvc_t *pSrvc;
	//uint8_t SecType;					//!< Secure or Open service/char
	//int NbChar;
	//BtAttDBEntry_t *pCharEntry[1];		//!< Variable length char entry table
} BtAttSrvcDeclar_t;

// Service include attribute : 0x2802
typedef struct __Bt_Att_Srvc_Include {
	uint16_t SrvcHdl;					//!< Service attribute handle
	uint16_t EndGrpHdl;					//!< End group handle
	BtUuid_t SrvcUuid;					//!< Service UUID
} BtAttSrvcInclude_t;

#define BT_CHAR_PROP_BROADCAST			1	//!< If set, permits broadcasts of the
												//!< Characteristic Value using Server
												//! Characteristic Configuration Descriptor.
												//! If set, the Server Characteristic Configuration
												//! Descriptor shall exist.
#define BT_CHAR_PROP_READ				2	//!< If set, permits reads of the Characteristic Value
#define BT_CHAR_PROP_WRITE_WORESP		4	//!<
#define BT_CHAR_PROP_WRITE				8
#define BT_CHAR_PROP_NOTIFY				0x10
#define BT_CHAR_PROP_INDICATE			0x20
#define BT_CHAR_PROP_AUTH_SIGNED		0x40
#define BT_CHAR_PROP_EXT_PROP			0x80
#define BT_CHAR_PROP_VALEN				0x8000

// Characteristic declaration attribute : type UUID 0x2803
typedef struct __Bt_Att_Char_Declar {
	//uint8_t Prop;						//!< Orable properties
	//uint16_t ValHdl;					//!< Value handle
	BtUuid_t Uuid;						//!< Characteristic UUID
	BtChar_t *pChar;					//!< Point to owner service entry
} BtAttCharDeclar_t;

typedef struct __Bt_Att_Char_Value {
	BtChar_t *pChar;					//!< Pointer to owner characteristic
	uint8_t Data[1];					//!< Variable length data buffer
} BtAttCharValue_t;

#define BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION		(1<<0)
#define BT_DESC_CLIENT_CHAR_CONFIG_INDICATION		(1<<1)

typedef struct __Bt_Desc_Client_Char_Config {
	BtGattChar_t *pChar;				//!< Owner characteristic
	uint16_t CccVal;					//!< Characteristic value handle
} BtDescClientCharConfig_t;

//BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION
typedef struct __Bt_Desc_Char_User_Desc {
	BtChar_t *pChar;					//!< Owner characteristic
	//const char *pDescStr;				//!< Description string
} BtDescCharUserDesc_t;

#pragma pack(pop)

#pragma pack(push,4)

struct __Bt_Att_DB_Entry {
	uint16_t Hdl;						//!< Attribute handle
	BtUuid16_t TypeUuid;				//!< Attribute type UUID
	uint32_t Permission;				//!< Attribute Permission
	uint16_t DataLen;					//!< Data length
	BtAttDBEntry_t *pPrev;
	BtAttDBEntry_t *pNext;
	uint8_t Data[1];					//!< Variable length attribute data
};

struct __Bt_Characteristic {
	uint16_t Uuid;						//!< Characteristic UUID
	uint16_t MaxDataLen;				//!< Characteristic max data length in bytes
	uint32_t Property;              	//!< char properties defined by orable BT_GATT_CHAR_PROP_...
	const char *pDesc;                  //!< char UTF-8 description string
	BtCharWrCb_t WrCB;              	//!< Callback for write char, set to NULL for read char
	BtCharSetNotifCb_t SetNotifCB;		//!< Callback on set notification
	BtCharSetIndCb_t SetIndCB;			//!< Callback on set indication
	BtCharTxComplete_t TxCompleteCB;	//!< Callback when TX is completed
	void *pValue;						//!< Characteristic data value
	uint16_t ValueLen;					//!< Current length in bytes of data value
	// Bellow are private data. Do not modify
	bool bNotify;                       //!< Notify enable flag for read characteristic
	bool bIndic;						//!< Indication enable flag
	uint8_t BaseUuidIdx;				//!< Index of Base UUID used for this characteristic.
	uint16_t Hdl;       				//!< char handle
	uint16_t ValHdl;					//!< char value handle
	uint16_t DescHdl;					//!< descriptor handle
	uint16_t CccdHdl;					//!< client char configuration descriptor handle
	uint16_t SccdHdl;					//!< Server char configuration value
	BtSrvc_t *pSrvc;					//!< Pointer to the service instance which this char belongs to.
	//BtGattCharValue_t *pData;
};

/*
 * Bluetooth service private data to be passed when calling service related functions.
 * The data is filled by BleSrvcInit function.
 * Pointer to this structure is often referred as Service Handle
 *
 */
struct __Bt_Service {
    int NbChar;							//!< Number of characteristic defined for this service
    BtGattChar_t *pCharArray;			//!< Pointer to array of characteristics
    uint16_t Hdl;            			//!< Service handle
//    uint16_t ConnHdl;					//!< Connection handle
    BtUuid_t Uuid;						//!< Service UUID
    uint8_t	*pLongWrBuff;				//!< pointer to user long write buffer
    int	LongWrBuffSize;					//!< long write buffer size
    void *pContext;
    BtSrvcAuthRqst_t AuthReqCB;			//!< Authorization request callback
    BtSrvc_t *pPrev;
    BtSrvc_t *pNext;
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void BtAttDBInit(size_t MemSize);
BtAttDBEntry_t * const BtAttDBAddEntry(BtUuid16_t *pUuid, int MaxDataLen);//, void *pData, int DataLen);
BtAttDBEntry_t * const BtAttDBFindHandle(uint16_t Hdl);
BtAttDBEntry_t * const BtAttDBFindUuid(BtAttDBEntry_t *pStart, BtUuid16_t *pUuid);
BtAttDBEntry_t * const BtAttDBFindUuidRange(BtUuid16_t *pUuid, uint16_t HdlStart, uint16_t HdlEnd);
BtAttDBEntry_t * const BtAttDBFindHdlRange(BtUuid16_t *pUuid, uint16_t *pHdlStart, uint16_t *pHdlEnd);

uint32_t BtAttError(BtAttReqRsp_t * const pRspAtt, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode);
/**
 * @brief	Set max attribute MTU
 *
 * @param	MaxMtu : Max MTU
 *
 * @return	Current max MTU
 */
uint16_t BtAttSetMtu(uint16_t MaxMtu);
uint16_t BtAttGetMtu();
uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pInAtt, int ReqLen, BtAttReqRsp_t * const pOutAtt);

bool BtAttExchangeMtuRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Mtu);

#ifdef __cplusplus
}
#endif

#endif // __BT_ATT_H__
