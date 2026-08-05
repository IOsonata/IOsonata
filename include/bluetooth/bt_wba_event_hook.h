/**-------------------------------------------------------------------------
@file	bt_wba_event_hook.h

@brief	STM32WBA BLE event validation and GATT completion routing.

Included only after the STM32WBA BLE middleware headers are visible. It wraps
SVCCTL_RegisterHandler so malformed variable-length discovery events are
dropped before the port parser reads them, notification completion is matched
by connection/attribute handle, and indication confirmation is kept separate
from the notification completion ring.
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
					BtGattHandleValueConfirm(BtWbaEvtLe16(pAci->data));
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

// Preserve the vendor registration call inside BtWbaEventHookRegister above,
// then replace subsequent source-level registrations with the validating
// wrapper. Some CubeWBA releases expose SVCCTL_RegisterHandler as a macro
// rather than a plain function declaration, so remove that spelling before
// installing ours.
#ifdef SVCCTL_RegisterHandler
#undef SVCCTL_RegisterHandler
#endif
#define SVCCTL_RegisterHandler(handler) BtWbaEventHookRegister(handler)

#endif // STM32WBA BLE headers visible

#endif // __BT_WBA_EVENT_HOOK_H__
