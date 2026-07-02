/**-------------------------------------------------------------------------
@file	bt_gatt.h

@brief	Bluetooth GATT  

Generic implementation & definitions of Bluetooth Generic Attribute Profile

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#ifndef __BT_GATT_H__
#define __BT_GATT_H__

#include <inttypes.h>
#include <stddef.h>

#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_att.h"

#define BT_GATT_CHAR_EXT_PROP_RELIABLE_WRITE	1	//!< Reliable write using procedure Section 4.9.5
#define BT_GATT_CHAR_EXT_PROP_WRITABLE_AUX		2	//!< Write to descriptor defined in Section 3.3.3.2

// Characteristic extended property attribute : type UUID 0x2900
//typedef uint8_t		BtGattCharExtProp_t;

// Characteristic user description attribute : Type UUID 0x2901
// Attribute value : UTF-8
//typedef char *		BtGattCharUserDesc_t;


// Client characteristic configuration declaration attribute UUID 0x2902
//typedef uint8_t BtGatClientCharConfig_t;

//#define BT_GATT_SERVER_CHAR_CONFIG_BROADCAST		1

// Server characteristic configuration declaration attribute UUID 0x2903
//typedef uint8_t		BtGattServerCharConfig_t;

#define BT_GATT_CHAR_PROP_BROADCAST			BT_CHAR_PROP_BROADCAST	//!< If set, permits broadcasts of the
												//!< Characteristic Value using Server
												//! Characteristic Configuration Descriptor.
												//! If set, the Server Characteristic Configuration
												//! Descriptor shall exist.
#define BT_GATT_CHAR_PROP_READ				BT_CHAR_PROP_READ	//!< If set, permits reads of the Characteristic Value
#define BT_GATT_CHAR_PROP_WRITE_WORESP		BT_CHAR_PROP_WRITE_WORESP	//!<
#define BT_GATT_CHAR_PROP_WRITE				BT_CHAR_PROP_WRITE
#define BT_GATT_CHAR_PROP_NOTIFY			BT_CHAR_PROP_NOTIFY
#define BT_GATT_CHAR_PROP_INDICATE			BT_CHAR_PROP_INDICATE
#define BT_GATT_CHAR_PROP_AUTH_SIGNED		BT_CHAR_PROP_AUTH_SIGNED
#define BT_GATT_CHAR_PROP_EXT_PROP			BT_CHAR_PROP_EXT_PROP


#ifndef BT_GATT_DB_MAX_CHARS
#define BT_GATT_DB_MAX_CHARS	6 	/**< The maximum number of characteristics present in a service record. */
#endif


#pragma pack(push,1)

typedef struct __Bt_Gatt_Char_Srvc_Changed {
	uint16_t StartHdl;					//!< Start service handle
	uint16_t EndHdl;					//!< End service handle
} BtGattCharSrvcChanged_t;

typedef struct __Bt_Gatt_Char_Prefered_Conn_Params {
	uint16_t IntervalMin;				//!< Min connection interval in 1.25ms counts
	uint16_t IntervalMax;				//!< Max connection interval in 1.25ms counts
	uint16_t Latency;					//!< Peripheral latency
	uint16_t Timeout;					//!< Supervision timeout in 10ms count
} BtGattPreferedConnParams_t;

#pragma pack(pop)


// BtGattSrvcCfg_t was retired: its fields are now part of BtGattSrvc_t
// itself (see bt_att.h). Service declaration is a single struct now.


/* Struct analogous to ble_gatt_db.h of SoftDevice lib for central device*/
#pragma pack(push,4)

typedef struct __Bt_Gattc_Hdl_Range {
	uint16_t StartHdl;					//!< Start service handle
	uint16_t EndHdl;					//!< End service handle
} BtGattcHdlRange_t;

/**@brief GATT Characteristic Properties.
 * Equivalent to ble_gatt_char_props_t
 * */
typedef struct
{
  /* Standard properties */
  uint8_t broadcast       :1; /**< Broadcasting of the value permitted. */
  uint8_t read            :1; /**< Reading the value permitted. */
  uint8_t write_wo_resp   :1; /**< Writing the value with Write Command permitted. */
  uint8_t write           :1; /**< Writing the value with Write Request permitted. */
  uint8_t notify          :1; /**< Notification of the value permitted. */
  uint8_t indicate        :1; /**< Indications of the value permitted. */
  uint8_t auth_signed_wr  :1; /**< Writing the value with Signed Write Command permitted. */
} BtGattCharProps_t;//ble_gatt_char_props_t;

/**@brief GATT Characteristic Extended Properties.
 * Equivalent to ble_gatt_char_ext_props_t
 * */
typedef struct
{
  /* Extended properties */
  uint8_t reliable_wr     :1; /**< Writing the value with Queued Write operations permitted. */
  uint8_t wr_aux          :1; /**< Writing the Characteristic User Description descriptor permitted. */
} BtGattCharExtProps_t;//ble_gatt_char_ext_props_t;


/**@brief GATT characteristic.
 * Equivalent to ble_gattc_char_t
 * */
typedef struct
{
	BtUuid16_t					uuid;
	BtGattCharProps_t   		char_props;           /**< Characteristic Properties. */
	uint8_t                 	char_ext_props : 1;   /**< Extended properties present. */
	uint16_t                	handle_decl;          /**< Handle of the Characteristic Declaration. */
	uint16_t                	handle_value;         /**< Handle of the Characteristic Value. */
} BtGattcChar_t;

/**@brief Structure for holding the characteristic and the handle of its CCCD present on a server.
 * 	Equivalent to ble_gatt_db_char_t
 *
 */
typedef struct
{
    BtGattcChar_t 	characteristic;    /**< Structure containing information about the characteristic. */
    uint16_t        cccd_handle;       /**< CCCD Handle value for this characteristic. This will be set to BLE_GATT_HANDLE_INVALID if a CCCD is not present at the server. */
    uint16_t        ext_prop_handle;   /**< Extended Properties Handle value for this characteristic. This will be set to BLE_GATT_HANDLE_INVALID if an Extended Properties descriptor is not present at the server. */
    uint16_t        user_desc_handle;  /**< User Description Handle value for this characteristic. This will be set to BLE_GATT_HANDLE_INVALID if a User Description descriptor is not present at the server. */
    uint16_t        report_ref_handle; /**< Report Reference Handle value for this characteristic. This will be set to BLE_GATT_HANDLE_INVALID if a Report Reference descriptor is not present at the server. */
} BtGattDBChar_t;

/**@brief Structure for holding information about the service and the characteristics present on a
 *        server.
 *        Equivalent to ble_gatt_db_srv_t
 */
typedef struct
{
	BtUuid16_t			srv_uuid;
    uint8_t             char_count;                                /**< Number of characteristics present in the service. */
    BtGattcHdlRange_t	handle_range;                              /**< Service Handle Range. */
    BtGattDBChar_t      characteristics[BT_GATT_DB_MAX_CHARS];     /**< Array of information related to the characteristics present in the service. This list can extend further than one. */
} BtGattDBSrvc_t;//ble_gatt_db_srv_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

BtGattSrvc_t * const BtGattGetSrvcList(void);
void BtGattInsertSrvcList(BtGattSrvc_t * const pSrvc);
bool BtGattUpdate(uint16_t Hdl, void * const pAttVal, size_t Len);
bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar);
bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len);
bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len);
// Record a notify/indicate packet accepted by the transport so
// BtGattSendCompleted() can fire TxCompleteCB in send order.
void BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar);
void BtGattHandleValueConfirm(uint16_t ConnHdl);

uint16_t BtGattCccdGet(uint16_t ConnHdl, uint16_t CccdHdl);
bool BtGattCccdSet(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value);
void BtGattCccdClear(uint16_t ConnHdl);
void BtGattCccdRestoreBonded(uint16_t ConnHdl);
// Mirror the aggregate CCCD value into the native ATT DB descriptor so a local
// CCCD read reflects it. Weak no-op default in bt_gatt.cpp; the native host
// (bt_att.cpp) provides the strong version. SoftDevice/ST ports have no native
// DB and use the no-op, which keeps bt_gatt.cpp free of BtAttDBFindHandle.
void BtGattCccdDbSync(uint16_t CccdHdl, uint16_t CccVal);
bool BtGattCharNotifyEnabled(uint16_t ConnHdl, BtGattChar_t *pChar);
bool BtGattCharIndicateEnabled(uint16_t ConnHdl, BtGattChar_t *pChar);

/**
 * @brief	Client-role receive hook. Called when a Handle Value Notification or
 *			Indication arrives from a peer. ValHdl is the peer characteristic
 *			value handle; pData/Len are the received value. For an indication the
 *			Handle Value Confirmation is sent before this runs. Weak default is
 *			empty; the application overrides it (e.g. to forward to a UART) and
 *			matches ValHdl against the handle it discovered.
 */
void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len);

/**
 * @brief	Millisecond tick source for the indication transaction timeout.
 *			Weak default returns 0 (no clock), which disables the timeout. A port
 *			or application that has a running millisecond counter overrides this
 *			to return it.
 */
uint32_t BtGattMsTick(void);

/**
 * @brief	Read-only query for the ATT indication transaction timeout (Core
 *			spec Vol 3 Part F, 3.3.3). Returns true when an indication has been
 *			outstanding on ConnHdl for at least TimeoutMs without the peer's
 *			confirmation. The spec action on timeout is to terminate the link, so
 *			the caller disconnects ConnHdl when this returns true. Always false
 *			unless BtGattMsTick() is overridden with a real clock.
 */
bool BtGattIndicationTimedOut(uint16_t ConnHdl, uint32_t TimeoutMs);
bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc);
void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc);
//void BtGattServiceInit(BtGattSrvc_t * const pSrvc);
void BtGattEvtHandler(uint32_t Evt, void * const pCtx);
void BtGattSrvcEvtHandler(BtGattSrvc_t * const pSrvc, uint32_t Evt, void * const pCtx);
void BtGattSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent);

#ifdef __cplusplus
}
#endif

//
// --- Declaration macros ---
//
// Pure textual substitution. Each macro expands to a brace-enclosed
// aggregate initializer for the corresponding stack struct; no statements,
// no helper functions, no code generation. Callers fill in user-supplied
// fields by name; the variadic tail accepts any additional designated
// initializers (e.g. .WrCB, .SetNotifCB).
//
// Spec terms used:
//   - Property bits are BT_GATT_CHAR_PROP_* from this header, matching the
//     Characteristic Properties field in Core Vol 3 Part G 3.3.1.1.
//   - bCustom distinguishes a vendor-defined 128-bit UUID (true) from a
//     Bluetooth SIG adopted 16-bit UUID (false).
//
// Example usage:
//
//   static BtGattChar_t s_UartChars[] = {
//       BT_CHAR(BLE_UART_UUID_RX_CHAR, PACKET_SIZE,
//               BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY, "UART Rx"),
//       BT_CHAR(BLE_UART_UUID_TX_CHAR, PACKET_SIZE,
//               BT_GATT_CHAR_PROP_WRITE_WORESP, "UART Tx", .WrCB = OnTxWrite),
//   };
//
//   BtGattSrvc_t g_UartSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
//                                            BLE_UART_UUID_SERVICE,
//                                            s_UartChars);
//
//   // For a read-only static-backed characteristic:
//   //   BtGattChar_t s_FwVer = BT_CHAR(FW_VER_UUID, sizeof(s_FwVersion),
//   //                                  BT_GATT_CHAR_PROP_READ, "FW Version",
//   //                                  .WrCB = MyWriteCb);

#define BT_CHAR(uuid, maxlen, props, desc, ...) \
	{ .Uuid = (uuid), .MaxDataLen = (maxlen), .Property = (props), \
	  .pDesc = (desc), __VA_ARGS__ }

#define BT_SRVC_STD(uuid_srvc, chars_array, ...) \
	{ .bCustom = false, .UuidSrvc = (uuid_srvc), \
	  .NbChar = sizeof(chars_array) / sizeof((chars_array)[0]), \
	  .pCharArray = (chars_array), __VA_ARGS__ }

#define BT_SRVC_CUSTOM(uuid_base, uuid_srvc, chars_array, ...) \
	{ .bCustom = true, .UuidBase = uuid_base, .UuidSrvc = (uuid_srvc), \
	  .NbChar = sizeof(chars_array) / sizeof((chars_array)[0]), \
	  .pCharArray = (chars_array), __VA_ARGS__ }

#endif // __BT_GATT_H__

