#include "bt_virtual_peer.h"

#include <cstring>

#include "bluetooth/bt_l2cap.h"

namespace btcompliance {

VirtualPeer::VirtualPeer(VirtualController &Controller, uint16_t ConnHdl) :
	vController(Controller),
	vConnHdl(ConnHdl),
	vHostAclReadIndex(0),
	vHostReassemblyActive(false),
	vHostExpected(0),
	vHostReceived(0)
{
	std::memset(vHostBuffer, 0, sizeof(vHostBuffer));
}

bool VirtualPeer::SendL2cap(uint16_t Cid, const void *pPayload, uint16_t PayloadLen,
	uint16_t FragmentPayloadLen)
{
	if ((PayloadLen != 0 && pPayload == nullptr) ||
		static_cast<uint32_t>(PayloadLen) + sizeof(BtL2CapHdr_t) > sizeof(vHostBuffer))
	{
		return false;
	}

	uint8_t pdu[BT_HCI_BUFFER_MAX_SIZE] = {};
	pdu[0] = static_cast<uint8_t>(PayloadLen & 0xFFU);
	pdu[1] = static_cast<uint8_t>((PayloadLen >> 8) & 0xFFU);
	pdu[2] = static_cast<uint8_t>(Cid & 0xFFU);
	pdu[3] = static_cast<uint8_t>((Cid >> 8) & 0xFFU);
	if (PayloadLen != 0)
	{
		std::memcpy(pdu + sizeof(BtL2CapHdr_t), pPayload, PayloadLen);
	}

	uint16_t total = static_cast<uint16_t>(PayloadLen + sizeof(BtL2CapHdr_t));
	if (FragmentPayloadLen == 0 || FragmentPayloadLen >= total)
	{
		return vController.InjectAcl(vConnHdl, BT_HCI_PBFLAG_START_NONFLUSHABLE,
			pdu, total);
	}

	if (FragmentPayloadLen < sizeof(BtL2CapHdr_t))
	{
		return false;
	}

	uint16_t offset = 0;
	while (offset < total)
	{
		uint16_t chunk = static_cast<uint16_t>(total - offset);
		if (chunk > FragmentPayloadLen)
		{
			chunk = FragmentPayloadLen;
		}

		uint8_t pbFlag = offset == 0 ? BT_HCI_PBFLAG_START_NONFLUSHABLE
			: BT_HCI_PBFLAG_CONTINUING_FRAGMENT;
		if (!vController.InjectAcl(vConnHdl, pbFlag, pdu + offset, chunk))
		{
			return false;
		}
		offset = static_cast<uint16_t>(offset + chunk);
	}

	return true;
}

bool VirtualPeer::SendAtt(const void *pPdu, uint16_t Length, uint16_t FragmentPayloadLen)
{
	return SendL2cap(BT_L2CAP_CID_ATT, pPdu, Length, FragmentPayloadLen);
}

bool VirtualPeer::SendSmp(const void *pPdu, uint16_t Length, uint16_t FragmentPayloadLen)
{
	return SendL2cap(BT_L2CAP_CID_SEC_MNGR, pPdu, Length, FragmentPayloadLen);
}

bool VirtualPeer::SendOrphanContinuation(const void *pData, uint16_t Length)
{
	return vController.InjectAcl(vConnHdl, BT_HCI_PBFLAG_CONTINUING_FRAGMENT,
		pData, Length);
}

void VirtualPeer::ResetHostReader()
{
	vHostAclReadIndex = 0;
	vHostReassemblyActive = false;
	vHostExpected = 0;
	vHostReceived = 0;
	std::memset(vHostBuffer, 0, sizeof(vHostBuffer));
}

bool VirtualPeer::ReadHostL2cap(uint16_t *pCid, void *pPayload, size_t PayloadCapacity,
	size_t *pPayloadLen)
{
	if (pCid == nullptr || pPayloadLen == nullptr ||
		(PayloadCapacity != 0 && pPayload == nullptr))
	{
		return false;
	}

	while (vHostAclReadIndex < vController.AclCount())
	{
		const VirtualController::PacketRecord *pRecord =
			vController.AclAt(vHostAclReadIndex++);
		if (pRecord == nullptr || pRecord->Length < sizeof(BtHciACLDataPacketHdr_t))
		{
			continue;
		}

		const BtHciACLDataPacket_t *pAcl =
			reinterpret_cast<const BtHciACLDataPacket_t *>(pRecord->Data);
		if (pAcl->Hdr.ConnHdl != vConnHdl ||
			pAcl->Hdr.Len > pRecord->Length - sizeof(BtHciACLDataPacketHdr_t))
		{
			continue;
		}

		if (pAcl->Hdr.PBFlag != BT_HCI_PBFLAG_CONTINUING_FRAGMENT)
		{
			vHostReassemblyActive = false;
			if (pAcl->Hdr.Len < sizeof(BtL2CapHdr_t))
			{
				continue;
			}

			uint16_t l2capPayloadLen = static_cast<uint16_t>(pAcl->Data[0]) |
				(static_cast<uint16_t>(pAcl->Data[1]) << 8);
			vHostExpected = static_cast<uint16_t>(sizeof(BtL2CapHdr_t) + l2capPayloadLen);
			if (vHostExpected > sizeof(vHostBuffer) || pAcl->Hdr.Len > vHostExpected)
			{
				continue;
			}

			std::memcpy(vHostBuffer, pAcl->Data, pAcl->Hdr.Len);
			vHostReceived = pAcl->Hdr.Len;
			vHostReassemblyActive = true;
		}
		else
		{
			if (!vHostReassemblyActive ||
				static_cast<uint32_t>(vHostReceived) + pAcl->Hdr.Len > vHostExpected)
			{
				vHostReassemblyActive = false;
				continue;
			}
			std::memcpy(vHostBuffer + vHostReceived, pAcl->Data, pAcl->Hdr.Len);
			vHostReceived = static_cast<uint16_t>(vHostReceived + pAcl->Hdr.Len);
		}

		if (vHostReassemblyActive && vHostReceived == vHostExpected)
		{
			size_t payloadLen = vHostExpected - sizeof(BtL2CapHdr_t);
			if (payloadLen > PayloadCapacity)
			{
				vHostReassemblyActive = false;
				return false;
			}

			*pCid = static_cast<uint16_t>(vHostBuffer[2]) |
				(static_cast<uint16_t>(vHostBuffer[3]) << 8);
			*pPayloadLen = payloadLen;
			if (payloadLen != 0)
			{
				std::memcpy(pPayload, vHostBuffer + sizeof(BtL2CapHdr_t), payloadLen);
			}
			vHostReassemblyActive = false;
			return true;
		}
	}

	return false;
}

} // namespace btcompliance
