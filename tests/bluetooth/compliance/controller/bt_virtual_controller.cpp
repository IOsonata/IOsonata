#include "bt_virtual_controller.h"

#include <cstring>

namespace btcompliance {

VirtualController *VirtualController::spActive = nullptr;

VirtualController::VirtualController(VirtualClock &Clock) :
	vClock(Clock),
	vpHost(nullptr),
	vCommandCount(0),
	vAclCount(0),
	vAclSendCalls(0),
	vFailAclCall(0)
{
	Reset();
}

VirtualController::~VirtualController()
{
	// BtHciDevice_t is caller-owned and may have gone out of scope before this
	// controller. Never dereference it from the destructor. A caller that needs
	// the function pointers cleared while the host is still alive calls Detach().
	if (spActive == this)
	{
		spActive = nullptr;
	}
	vpHost = nullptr;
}

bool VirtualController::Attach(BtHciDevice_t *pHost)
{
	if (pHost == nullptr || (spActive != nullptr && spActive != this))
	{
		return false;
	}

	if (vpHost != nullptr && vpHost != pHost)
	{
		Detach();
	}

	vpHost = pHost;
	spActive = this;
	vpHost->pCtx = this;
	vpHost->SendData = HostSendData;
	vpHost->Command = HostCommand;
	return true;
}

void VirtualController::Detach()
{
	if (vpHost != nullptr)
	{
		if (vpHost->pCtx == this)
		{
			vpHost->pCtx = nullptr;
		}
		if (vpHost->SendData == HostSendData)
		{
			vpHost->SendData = nullptr;
		}
		if (vpHost->Command == HostCommand)
		{
			vpHost->Command = nullptr;
		}
	}

	if (spActive == this)
	{
		spActive = nullptr;
	}
	vpHost = nullptr;
}

void VirtualController::Reset()
{
	std::memset(vCommands, 0, sizeof(vCommands));
	std::memset(vAclPackets, 0, sizeof(vAclPackets));
	std::memset(vScheduledEvents, 0, sizeof(vScheduledEvents));
	std::memset(&vCommandResponse, 0, sizeof(vCommandResponse));
	vCommandCount = 0;
	vAclCount = 0;
	vAclSendCalls = 0;
	vFailAclCall = 0;
	vClock.Reset();
}

void VirtualController::SetCommandResponse(uint16_t Opcode, uint8_t Status,
	const void *pReturnData, uint8_t ReturnLen)
{
	std::memset(&vCommandResponse, 0, sizeof(vCommandResponse));
	vCommandResponse.Enabled = true;
	vCommandResponse.Opcode = Opcode;
	vCommandResponse.Status = Status;
	vCommandResponse.ReturnLen = ReturnLen;
	if (ReturnLen != 0 && pReturnData != nullptr)
	{
		std::memcpy(vCommandResponse.ReturnData, pReturnData, ReturnLen);
	}
	else if (ReturnLen != 0)
	{
		vCommandResponse.ReturnLen = 0;
	}
}

void VirtualController::ClearCommandResponse()
{
	std::memset(&vCommandResponse, 0, sizeof(vCommandResponse));
}

void VirtualController::FailAclOnCall(size_t CallNumber)
{
	vFailAclCall = CallNumber;
}

void VirtualController::ClearAclFailure()
{
	vFailAclCall = 0;
}

const VirtualController::CommandRecord *VirtualController::CommandAt(size_t Index) const
{
	return Index < vCommandCount ? &vCommands[Index] : nullptr;
}

const VirtualController::PacketRecord *VirtualController::AclAt(size_t Index) const
{
	return Index < vAclCount ? &vAclPackets[Index] : nullptr;
}

uint32_t VirtualController::HostSendData(void *pData, uint32_t Length)
{
	return spActive != nullptr ? spActive->CaptureAcl(pData, Length) : 0;
}

uint8_t VirtualController::HostCommand(BtHciDevice_t * const pDev, uint16_t Opcode,
	const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (pDev == nullptr || pDev->pCtx == nullptr)
	{
		return 0xFF;
	}

	VirtualController *pController = static_cast<VirtualController *>(pDev->pCtx);
	return pController->ExecuteCommand(Opcode, pParam, ParamLen, pRet, RetLen);
}

uint32_t VirtualController::CaptureAcl(const void *pData, uint32_t Length)
{
	vAclSendCalls++;
	if (vFailAclCall != 0 && vAclSendCalls == vFailAclCall)
	{
		return 0;
	}

	if (pData == nullptr || Length > ACL_PACKET_CAPACITY || vAclCount >= ACL_CAPACITY)
	{
		return 0;
	}

	PacketRecord &record = vAclPackets[vAclCount++];
	record.Length = Length;
	std::memcpy(record.Data, pData, Length);
	return Length;
}

uint8_t VirtualController::ExecuteCommand(uint16_t Opcode, const void *pParam,
	uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (vCommandCount >= COMMAND_CAPACITY || (ParamLen != 0 && pParam == nullptr))
	{
		return 0xFF;
	}

	CommandRecord &record = vCommands[vCommandCount++];
	record.Opcode = Opcode;
	record.ParamLen = ParamLen;
	if (ParamLen != 0)
	{
		std::memcpy(record.Param, pParam, ParamLen);
	}

	if (!vCommandResponse.Enabled || vCommandResponse.Opcode != Opcode)
	{
		return 0;
	}

	if (pRet != nullptr && RetLen != 0 && vCommandResponse.ReturnLen != 0)
	{
		uint8_t copyLen = RetLen < vCommandResponse.ReturnLen ? RetLen : vCommandResponse.ReturnLen;
		std::memcpy(pRet, vCommandResponse.ReturnData, copyLen);
	}
	return vCommandResponse.Status;
}

bool VirtualController::InjectEvent(uint8_t EventCode, const void *pParameters,
	uint8_t ParameterLen)
{
	if (vpHost == nullptr || (ParameterLen != 0 && pParameters == nullptr))
	{
		return false;
	}

	alignas(4) uint8_t raw[EVENT_PACKET_CAPACITY] = {};
	BtHciEvtPacket_t *pEvent = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvent->Hdr.Evt = EventCode;
	pEvent->Hdr.Len = ParameterLen;
	if (ParameterLen != 0)
	{
		std::memcpy(pEvent->Data, pParameters, ParameterLen);
	}
	BtHciProcessEvent(vpHost, pEvent);
	return true;
}

bool VirtualController::InjectLeEvent(uint8_t SubeventCode, const void *pParameters,
	uint8_t ParameterLen)
{
	if (ParameterLen == 255)
	{
		return false;
	}

	uint8_t parameters[255] = {};
	parameters[0] = SubeventCode;
	if (ParameterLen != 0)
	{
		if (pParameters == nullptr)
		{
			return false;
		}
		std::memcpy(parameters + 1, pParameters, ParameterLen);
	}
	return InjectEvent(BT_HCI_EVT_LE, parameters, static_cast<uint8_t>(ParameterLen + 1));
}

bool VirtualController::InjectEventHeaderOnly(uint8_t EventCode)
{
	if (vpHost == nullptr)
	{
		return false;
	}

	// Allocate a complete packet object even though Hdr.Len declares no event
	// parameters. Casting storage smaller than BtHciEvtPacket_t is undefined in
	// the test itself and hides the parser result behind an object-size failure.
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t)] = {};
	BtHciEvtPacket_t *pEvent = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvent->Hdr.Evt = EventCode;
	pEvent->Hdr.Len = 0;
	pEvent->Data[0] = 0xFF;
	BtHciProcessEvent(vpHost, pEvent);
	return true;
}

bool VirtualController::InjectLeSubeventOnly(uint8_t SubeventCode)
{
	if (vpHost == nullptr)
	{
		return false;
	}

	// Keep one sentinel byte after the declared LE subevent. The parser must not
	// treat it as the list count merely because it exists in the allocation.
	alignas(4) uint8_t raw[sizeof(BtHciEvtPacket_t) + 1] = {};
	BtHciEvtPacket_t *pEvent = reinterpret_cast<BtHciEvtPacket_t *>(raw);
	pEvent->Hdr.Evt = BT_HCI_EVT_LE;
	pEvent->Hdr.Len = 1;
	pEvent->Data[0] = SubeventCode;
	pEvent->Data[1] = 0xFF;
	BtHciProcessEvent(vpHost, pEvent);
	return true;
}

VirtualController::ScheduledEvent *VirtualController::AllocateScheduledEvent()
{
	for (size_t i = 0; i < SCHEDULED_EVENT_CAPACITY; i++)
	{
		if (!vScheduledEvents[i].Used)
		{
			vScheduledEvents[i].Used = true;
			vScheduledEvents[i].pOwner = this;
			vScheduledEvents[i].Length = 0;
			return &vScheduledEvents[i];
		}
	}
	return nullptr;
}

bool VirtualController::ScheduleEventUs(uint64_t DelayUs, uint8_t EventCode,
	const void *pParameters, uint8_t ParameterLen)
{
	if (vpHost == nullptr || (ParameterLen != 0 && pParameters == nullptr))
	{
		return false;
	}

	ScheduledEvent *pScheduled = AllocateScheduledEvent();
	if (pScheduled == nullptr)
	{
		return false;
	}

	BtHciEvtPacket_t *pEvent = reinterpret_cast<BtHciEvtPacket_t *>(pScheduled->Data);
	pEvent->Hdr.Evt = EventCode;
	pEvent->Hdr.Len = ParameterLen;
	if (ParameterLen != 0)
	{
		std::memcpy(pEvent->Data, pParameters, ParameterLen);
	}
	pScheduled->Length = sizeof(BtHciEvtPacketHdr_t) + ParameterLen;

	if (!vClock.ScheduleUs(DelayUs, DeliverScheduledEvent, pScheduled))
	{
		pScheduled->Used = false;
		return false;
	}
	return true;
}

bool VirtualController::ScheduleLeEventUs(uint64_t DelayUs, uint8_t SubeventCode,
	const void *pParameters, uint8_t ParameterLen)
{
	if (ParameterLen == 255)
	{
		return false;
	}

	uint8_t parameters[255] = {};
	parameters[0] = SubeventCode;
	if (ParameterLen != 0)
	{
		if (pParameters == nullptr)
		{
			return false;
		}
		std::memcpy(parameters + 1, pParameters, ParameterLen);
	}
	return ScheduleEventUs(DelayUs, BT_HCI_EVT_LE, parameters,
		static_cast<uint8_t>(ParameterLen + 1));
}

void VirtualController::DeliverScheduledEvent(void *pContext)
{
	ScheduledEvent *pScheduled = static_cast<ScheduledEvent *>(pContext);
	if (pScheduled == nullptr || !pScheduled->Used || pScheduled->pOwner == nullptr)
	{
		return;
	}

	VirtualController *pOwner = pScheduled->pOwner;
	pScheduled->Used = false;
	if (pOwner->vpHost != nullptr && pScheduled->Length >= sizeof(BtHciEvtPacketHdr_t))
	{
		BtHciProcessEvent(pOwner->vpHost,
			reinterpret_cast<BtHciEvtPacket_t *>(pScheduled->Data));
	}
}

bool VirtualController::CompletePackets(uint16_t ConnHdl, uint16_t Count)
{
	uint8_t parameters[5];
	parameters[0] = 1;
	parameters[1] = static_cast<uint8_t>(ConnHdl & 0xFFU);
	parameters[2] = static_cast<uint8_t>((ConnHdl >> 8) & 0xFFU);
	parameters[3] = static_cast<uint8_t>(Count & 0xFFU);
	parameters[4] = static_cast<uint8_t>((Count >> 8) & 0xFFU);
	return InjectEvent(BT_HCI_EVT_NB_COMPLETED_PACKET, parameters, sizeof(parameters));
}

bool VirtualController::InjectAcl(uint16_t ConnHdl, uint8_t PbFlag,
	const void *pData, uint16_t Length)
{
	if (vpHost == nullptr || Length > BT_HCI_BUFFER_MAX_SIZE ||
		(Length != 0 && pData == nullptr))
	{
		return false;
	}

	alignas(4) uint8_t raw[ACL_PACKET_CAPACITY] = {};
	BtHciACLDataPacket_t *pAcl = reinterpret_cast<BtHciACLDataPacket_t *>(raw);
	pAcl->Hdr.ConnHdl = ConnHdl;
	pAcl->Hdr.PBFlag = PbFlag;
	pAcl->Hdr.BCFlag = BT_HCI_BCFLAG_POINT_TO_POINT;
	pAcl->Hdr.Len = Length;
	if (Length != 0)
	{
		std::memcpy(pAcl->Data, pData, Length);
	}
	BtHciProcessData(vpHost, pAcl);
	return true;
}

} // namespace btcompliance
