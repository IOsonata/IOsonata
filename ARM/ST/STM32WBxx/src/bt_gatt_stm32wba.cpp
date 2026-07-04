/**-------------------------------------------------------------------------
@file	bt_gatt_stm32wba.cpp

@brief	Bluetooth GATT server registration for the STM32WBA ST WPAN stack.

Registers IOsonata services and characteristics with the ST GATT server using
aci_gatt_add_service / aci_gatt_add_char, captures the ST assigned handles into
the BtGattChar_t fields that BtAppNotify, BtAppIndicate and the attribute
modified handler rely on, and links the service into the shared list so the per
connection CCCD machinery can resolve a characteristic from its CCCD handle.

This is the ST equivalent of bt_gatt_nrf52.cpp / bt_gatt_bm.cpp. The generic
BtGattSrvcAdd in bt_gatt.cpp is weak; this strong override replaces it on WBA.

@author	Nguyen Hoan Hoang
----------------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"
#include "ble_types.h"
#include "ble_std.h"
#include "ble_defs.h"
#include "ble_gatt_aci.h"
#include "ble_events.h"

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_app.h"

// ST lays a characteristic out as a run of consecutive handles: the declaration
// at the handle returned by aci_gatt_add_char, the value at declaration + 1,
// and, when the Notify or Indicate property is set, the CCCD that the stack
// adds automatically at declaration + 2. No other descriptor is added here, so
// this layout is fixed. Verify on hardware if a user description descriptor is
// later added, as that shifts the CCCD handle.
#define WBA_CHAR_VALUE_OFFSET		1
#define WBA_CHAR_CCCD_OFFSET		2

// Map the app security type to ST attribute permission bits. The model has
// no per-characteristic security, so the app level SecType applies to every
// user characteristic. MITM types require authenticated access; no-MITM
// types require encryption only.
static uint8_t WbaSecPerm(uint32_t SecType)
{
	switch (SecType)
	{
		case BTGAP_SECTYPE_STATICKEY_MITM:
		case BTGAP_SECTYPE_LESC_MITM:
		case BTGAP_SECTYPE_SIGNED_MITM:
			return ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_AUTHEN_WRITE;
		case BTGAP_SECTYPE_STATICKEY_NO_MITM:
		case BTGAP_SECTYPE_SIGNED_NO_MITM:
			return ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE;
		case BTGAP_SECTYPE_NONE:
		default:
			return ATTR_PERMISSION_NONE;
	}
}

static bool WbaGattCharAdd(BtGattSrvc_t * const pSrvc, BtGattChar_t * const pChar)
{
	Char_UUID_t uuid;
	uint8_t uuidType;

	if (pSrvc->bCustom)
	{
		// 128-bit characteristic UUID: the service base UUID with the 16-bit
		// value placed at byte offset 12 (array is little endian, LSB first).
		uuidType = UUID_TYPE_128;
		memcpy(uuid.Char_UUID_128, pSrvc->UuidBase, 16);
		uuid.Char_UUID_128[12] = (uint8_t)(pChar->Uuid & 0xFF);
		uuid.Char_UUID_128[13] = (uint8_t)(pChar->Uuid >> 8);
	}
	else
	{
		uuidType = UUID_TYPE_16;
		uuid.Char_UUID_16 = pChar->Uuid;
	}

	// BT_GATT_CHAR_PROP_* are the standard GATT property bits, identical in
	// value to ST CHAR_PROP_*, so the low byte passes through unchanged.
	uint16_t charHdl = BT_ATT_HANDLE_INVALID;
	tBleStatus st = aci_gatt_add_char(pSrvc->Hdl, uuidType, &uuid,
									  pChar->MaxDataLen, (uint8_t)pChar->Property,
									  WbaSecPerm(g_BtAppData.SecType), GATT_NOTIFY_ATTRIBUTE_WRITE,
									  MIN_ENCRY_KEY_SIZE, CHAR_VALUE_LEN_VARIABLE,
									  &charHdl);
	if (st != BLE_STATUS_SUCCESS)
	{
		return false;
	}

	pChar->Hdl = charHdl;
	pChar->ValHdl = charHdl + WBA_CHAR_VALUE_OFFSET;
	if (pChar->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
	{
		pChar->CccdHdl = charHdl + WBA_CHAR_CCCD_OFFSET;
	}
	else
	{
		pChar->CccdHdl = BT_ATT_HANDLE_INVALID;
	}
	pChar->bNotify = false;
	pChar->bIndic = false;
	pChar->pSrvc = (BtSrvc_t *)pSrvc;

	return true;
}

bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	Service_UUID_t uuid;
	uint8_t uuidType;

	if (pSrvc->bCustom)
	{
		uuidType = UUID_TYPE_128;
		memcpy(uuid.Service_UUID_128, pSrvc->UuidBase, 16);
		uuid.Service_UUID_128[12] = (uint8_t)(pSrvc->UuidSrvc & 0xFF);
		uuid.Service_UUID_128[13] = (uint8_t)(pSrvc->UuidSrvc >> 8);
	}
	else
	{
		uuidType = UUID_TYPE_16;
		uuid.Service_UUID_16 = pSrvc->UuidSrvc;
	}

	// Reserve one handle for the service declaration plus, per characteristic,
	// the declaration and value handles and a CCCD when Notify or Indicate is
	// set.
	uint8_t records = 1;
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		records += 2;
		if (pSrvc->pCharArray[i].Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
		{
			records += 1;
		}
	}

	uint16_t srvcHdl = BT_ATT_HANDLE_INVALID;
	tBleStatus st = aci_gatt_add_service(uuidType, &uuid, PRIMARY_SERVICE,
										 records, &srvcHdl);
	if (st != BLE_STATUS_SUCCESS)
	{
		return false;
	}
	pSrvc->Hdl = srvcHdl;

	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		if (WbaGattCharAdd(pSrvc, &pSrvc->pCharArray[i]) == false)
		{
			return false;
		}
	}

	BtGattInsertSrvcList(pSrvc);

	return true;
}

// Per-connection notify. aci_gatt_update_char_value_ext targets Conn_Handle and
// the ST stack sends only if that connection enabled notification, so the stack
// is the per-connection authority and no local CCCD gate is applied (same
// policy as the SoftDevice ports). Update_Type 0x01 = Notification.
bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		return false;
	}

	if (pChar == nullptr || pChar->pSrvc == nullptr ||
		pChar->Hdl == BT_ATT_HANDLE_INVALID ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (Len > pChar->MaxDataLen || Len > UINT8_MAX)
	{
		return false;
	}

	uint8_t dummy = 0;
	const uint8_t *p = (Len > 0) ? (const uint8_t *)pVal : &dummy;

	tBleStatus st = aci_gatt_update_char_value_ext(ConnHdl, pChar->pSrvc->Hdl,
												   pChar->Hdl, 0x01,
												   (uint16_t)Len, 0,
												   (uint8_t)Len, p);

	if (st == BLE_STATUS_SUCCESS)
	{
		// The ST stack copies the value into its TX pool synchronously and has
		// no per-notification on-air completion event, so acceptance is the
		// completion point on this port. Queue then drain so TxCompleteCB
		// fires with the same semantics as the other ports.
		BtGattTxPendingAdd(ConnHdl, pChar);
		BtGattSendCompleted(ConnHdl, 1);
		return true;
	}

	return false;
}

// Per-connection indicate. Update_Type 0x02 = Indication. The ST stack owns
// the on-air exchange; a successful send is recorded on the TX-pending ring
// and starts the per-link indication transaction timeout. The client
// confirmation arrives as ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE and routes
// to BtGattHandleValueConfirm plus a ring drain.
bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		return false;
	}

	if (pChar == nullptr || pChar->pSrvc == nullptr ||
		pChar->Hdl == BT_ATT_HANDLE_INVALID ||
		pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (Len > pChar->MaxDataLen || Len > UINT8_MAX)
	{
		return false;
	}

	uint8_t dummy = 0;
	const uint8_t *p = (Len > 0) ? (const uint8_t *)pVal : &dummy;

	tBleStatus st = aci_gatt_update_char_value_ext(ConnHdl, pChar->pSrvc->Hdl,
												   pChar->Hdl, 0x02,
												   (uint16_t)Len, 0,
												   (uint8_t)Len, p);

	if (st == BLE_STATUS_SUCCESS)
	{
		BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
		if (pConn != nullptr)
		{
			pConn->Conn.bIndCfmPending = true;
			pConn->Conn.IndCfmTime = BtGattMsTick();
		}
		BtGattTxPendingAdd(ConnHdl, pChar);
		return true;
	}

	return false;
}
