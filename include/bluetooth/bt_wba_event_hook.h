/**-------------------------------------------------------------------------
@file	bt_wba_event_hook.h

@brief	STM32WBA BLE event validation and native GAP/GATT routing.

Included only after the STM32WBA BLE middleware headers are visible. It wraps
SVCCTL_RegisterHandler so malformed variable-length discovery events are
dropped before the port parser reads them, notification completion is matched
by connection/attribute handle, indication confirmation is kept separate from
the notification completion ring, and connection-parameter events are routed
to the per-link GAP state machine.

The same source-level hook captures the one aci_gap_init result and retains the
native GAP handles. It also corrects the old BtAppGapDeviceNameSet path, which
used the GAP value-handle offset instead of the characteristic handle returned
by aci_gap_init.
----------------------------------------------------------------------------*/
#ifndef __BT_WBA_EVENT_HOOK_H__
#define __BT_WBA_EVENT_HOOK_H__

#if defined(__cplusplus) && defined(STM32WBAxx_H) && \
	defined(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) && \
	defined(ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE)

#include <stddef.h>
#include <stdint.h>

#include "bluetooth/bt_gatt.h"

void BtGattWbaNotificationComplete(uint16_t ConnHdl, uint16_t AttrHdl);
void BtGapWbaConnParamsConnected(uint16_t ConnHdl, uint8_t Role,
		uint16_t Interval, uint16_t Latency, uint16_t Timeout);
void BtGapWbaConnParamsDisconnected(uint16_t ConnHdl);
void BtGapWbaConnParamsUpdateComplete(uint16_t ConnHdl, uint8_t Status,
		uint16_t Interval, uint16_t Latency, uint16_t Timeout);
void BtGapWbaConnParamsL2capResponse(uint16_t ConnHdl, uint16_t Result);

void BtGapWbaNativeHandlesSet(uint8_t Role, uint8_t DeviceNameMaxLen,
		uint16_t GapSrvcHdl, uint16_t DevNameCharHdl,
		uint16_t AppearanceCharHdl);
uint16_t BtGapWbaNativeCharHandleResolve(uint16_t SrvcHdl,
		uint16_t CharHdl);
void BtGapWbaNativeCharUpdateComplete(uint16_t SrvcHdl,
		uint16_t RequestedCharHdl, uint16_t ActualCharHdl, uint8_t Status);

static inline uint16_t BtWbaEvtLe16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline bool BtWbaEvtRangeValid(size_t FixedLen, size_t DataLen,
									 size_t Available)
{
	return FixedLen <= Available && DataLen <= Available - FixedLen;
}

static bool BtWbaDiscServiceEventValid(
		const aci_att_read_by_group_type_resp_event_rp0 *p, size_t Available)
{
	const size_t fixed = offsetof(
			aci_att_read_by_group_type_resp_event_rp0, Attribute_Data_List);
	if (p == nullptr || !BtWbaEvtRangeValid(fixed, p->Data_Length, Available))
	{
		return false;
	}

	uint8_t elemLen = p->Attribute_Data_Length;
	uint8_t total = p->Data_Length;
	if ((elemLen != 6 && elemLen != 20) || total < elemLen ||
		(total % elemLen) != 0)
	{
		return false;
	}

	uint16_t previousEnd = 0;
	for (uint16_t off = 0; off < total; off += elemLen)
	{
		const uint8_t *e = p->Attribute_Data_List + off;
		uint16_t start = BtWbaEvtLe16(e);
		uint16_t end = BtWbaEvtLe16(e + 2);
		if (start == 0 || end < start ||
			(previousEnd != 0 && start <= previousEnd))
		{
			return false;
		}
		previousEnd = end;
	}
	return true;
}

static bool BtWbaDiscCharEventValid(
		const aci_att_read_by_type_resp_event_rp0 *p, size_t Available)
{
	const size_t fixed = offsetof(
			aci_att_read_by_type_resp_event_rp0, Handle_Value_Pair_Data);
	if (p == nullptr || !BtWbaEvtRangeValid(fixed, p->Data_Length, Available))
	{
		return false;
	}

	uint8_t elemLen = p->Handle_Value_Pair_Length;
	uint8_t total = p->Data_Length;
	if ((elemLen != 7 && elemLen != 21) || total < elemLen ||
		(total % elemLen) != 0)
	{
		return false;
	}

	uint16_t previousDecl = 0;
	for (uint16_t off = 0; off < total; off += elemLen)
	{
		const uint8_t *e = p->Handle_Value_Pair_Data + off;
		uint16_t decl = BtWbaEvtLe16(e);
		uint16_t value = BtWbaEvtLe16(e + 3);
		if (decl == 0 || value == 0 || value <= decl ||
			(previousDecl != 0 && decl <= previousDecl))
		{
			return false;
		}
		previousDecl = decl;
	}
	return true;
}

static bool BtWbaDiscInfoEventValid(
		const aci_att_find_info_resp_event_rp0 *p, size_t Available)
{
	const size_t fixed = offsetof(
			aci_att_find_info_resp_event_rp0, Handle_UUID_Pair);
	if (p == nullptr || !BtWbaEvtRangeValid(
			fixed, p->Event_Data_Length, Available))
	{
		return false;
	}

	uint8_t elemLen;
	if (p->Format == 0x01)
	{
		elemLen = 4;
	}
	else if (p->Format == 0x02)
	{
		elemLen = 18;
	}
	else
	{
		return false;
	}

	return p->Event_Data_Length >= elemLen &&
		   (p->Event_Data_Length % elemLen) == 0;
}

typedef SVCCTL_UserEvtFlowStatus_t (*BtWbaUserEvtHandler_t)(void *pPayload);

static BtWbaUserEvtHandler_t s_BtWbaOriginalHandler;

static SVCCTL_UserEvtFlowStatus_t BtWbaEventHookDispatch(void *pPayload)
{
	if (pPayload == nullptr)
	{
		return SVCCTL_UserEvtFlowEnable;
	}

	hci_event_pckt *pEvt =
		(hci_event_pckt *)((hci_uart_pckt *)pPayload)->data;

	if (pEvt->evt == HCI_DISCONNECTION_COMPLETE_EVT_CODE &&
		pEvt->plen >= sizeof(hci_disconnection_complete_event_rp0))
	{
		auto *p = (hci_disconnection_complete_event_rp0 *)pEvt->data;
		if (p->Status == 0)
		{
			BtGapWbaConnParamsDisconnected(p->Connection_Handle);
		}
	}
	else if (pEvt->evt == HCI_LE_META_EVT_CODE)
	{
		const size_t fixed = offsetof(evt_le_meta_event, data);
		if (pEvt->plen >= fixed)
		{
			evt_le_meta_event *pMeta = (evt_le_meta_event *)pEvt->data;
			size_t available = (size_t)pEvt->plen - fixed;

			switch (pMeta->subevent)
			{
				case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
					if (available >= sizeof(
							hci_le_connection_complete_event_rp0))
					{
						auto *p = (hci_le_connection_complete_event_rp0 *)pMeta->data;
						if (p->Status == 0)
						{
							BtGapWbaConnParamsConnected(
								p->Connection_Handle, p->Role, p->Conn_Interval,
								p->Conn_Latency, p->Supervision_Timeout);
						}
					}
					break;

				case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
					if (available >= sizeof(
							hci_le_connection_update_complete_event_rp0))
					{
						auto *p =
							(hci_le_connection_update_complete_event_rp0 *)pMeta->data;
						BtGapWbaConnParamsUpdateComplete(
							p->Connection_Handle, p->Status, p->Conn_Interval,
							p->Conn_Latency, p->Supervision_Timeout);
					}
					break;

				default:
					break;
			}
		}
	}

	if (pEvt->evt == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
	{
		if (pEvt->plen < offsetof(evt_blecore_aci, data))
		{
			return SVCCTL_UserEvtFlowEnable;
		}

		evt_blecore_aci *pAci = (evt_blecore_aci *)pEvt->data;
		size_t available =
			(size_t)pEvt->plen - offsetof(evt_blecore_aci, data);

		switch (pAci->ecode)
		{
			case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
				if (!BtWbaDiscServiceEventValid(
						(aci_att_read_by_group_type_resp_event_rp0 *)pAci->data,
						available))
				{
					return SVCCTL_UserEvtFlowEnable;
				}
				break;

			case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
				if (!BtWbaDiscCharEventValid(
						(aci_att_read_by_type_resp_event_rp0 *)pAci->data,
						available))
				{
					return SVCCTL_UserEvtFlowEnable;
				}
				break;

			case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
				if (!BtWbaDiscInfoEventValid(
						(aci_att_find_info_resp_event_rp0 *)pAci->data,
						available))
				{
					return SVCCTL_UserEvtFlowEnable;
				}
				break;

#ifdef ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE
			case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
				if (available >= sizeof(
						aci_l2cap_connection_update_resp_event_rp0))
				{
					auto *p =
						(aci_l2cap_connection_update_resp_event_rp0 *)pAci->data;
					BtGapWbaConnParamsL2capResponse(
						p->Connection_Handle, p->Result);
				}
				break;
#endif

#ifdef ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE
			case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
			{
				if (available >= sizeof(
						aci_gatt_notification_complete_event_rp0))
				{
					auto *p =
						(aci_gatt_notification_complete_event_rp0 *)pAci->data;
					BtGattWbaNotificationComplete(
						p->Connection_Handle, p->Attribute_Handle);
				}
				return SVCCTL_UserEvtFlowEnable;
			}
#endif

			case ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE:
				if (available >= 2)
				{
					uint16_t connHdl = BtWbaEvtLe16(pAci->data);
					BtGattHandleValueConfirm(connHdl);
					BtGattSendCompleted(connHdl, 1);
				}
				return SVCCTL_UserEvtFlowEnable;

			default:
				break;
		}
	}

	return s_BtWbaOriginalHandler != nullptr ?
		s_BtWbaOriginalHandler(pPayload) : SVCCTL_UserEvtFlowEnable;
}

static inline void BtWbaEventHookRegister(BtWbaUserEvtHandler_t Handler)
{
	s_BtWbaOriginalHandler = Handler;
	SVCCTL_RegisterHandler(BtWbaEventHookDispatch);
}

static inline tBleStatus BtWbaGapInitCapture(uint8_t Role,
		uint8_t PrivacyType, uint8_t DeviceNameCharLen,
		uint16_t *pGapSrvcHdl, uint16_t *pDevNameCharHdl,
		uint16_t *pAppearanceCharHdl)
{
	tBleStatus status = aci_gap_init(Role, PrivacyType, DeviceNameCharLen,
		pGapSrvcHdl, pDevNameCharHdl, pAppearanceCharHdl);
	if (status == BLE_STATUS_SUCCESS && pGapSrvcHdl != nullptr &&
		pDevNameCharHdl != nullptr && pAppearanceCharHdl != nullptr)
	{
		BtGapWbaNativeHandlesSet(Role, DeviceNameCharLen,
			*pGapSrvcHdl, *pDevNameCharHdl, *pAppearanceCharHdl);
	}
	return status;
}

static inline tBleStatus BtWbaGattUpdateCharValue(uint16_t SrvcHdl,
		uint16_t CharHdl, uint8_t Offset, uint8_t Len, uint8_t *pValue)
{
	uint16_t actualHdl = BtGapWbaNativeCharHandleResolve(SrvcHdl, CharHdl);
	tBleStatus status = aci_gatt_update_char_value(
		SrvcHdl, actualHdl, Offset, Len, pValue);

	// Every successful source-level Device Name write is followed by the
	// generic BtGapSetDevName forwarding call. When no handle correction was
	// needed, use INVALID as the source marker so the GAP port still suppresses
	// that duplicate native write. The actual handle remains unchanged.
	uint16_t sourceHdl = CharHdl == actualHdl ? BT_ATT_HANDLE_INVALID : CharHdl;
	BtGapWbaNativeCharUpdateComplete(
		SrvcHdl, sourceHdl, actualHdl, (uint8_t)status);
	return status;
}

// Preserve the vendor calls inside the wrappers above, then replace subsequent
// source-level calls in bt_app_stm32wba.cpp. Some CubeWBA releases expose these
// spellings as macros rather than plain function declarations.
#ifdef SVCCTL_RegisterHandler
#undef SVCCTL_RegisterHandler
#endif
#define SVCCTL_RegisterHandler(handler) BtWbaEventHookRegister(handler)

#ifdef aci_gap_init
#undef aci_gap_init
#endif
#define aci_gap_init(...) BtWbaGapInitCapture(__VA_ARGS__)

#ifdef aci_gatt_update_char_value
#undef aci_gatt_update_char_value
#endif
#define aci_gatt_update_char_value(...) BtWbaGattUpdateCharValue(__VA_ARGS__)

#endif // STM32WBA BLE headers visible

#endif // __BT_WBA_EVENT_HOOK_H__
