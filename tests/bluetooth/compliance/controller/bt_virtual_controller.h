#ifndef __BT_VIRTUAL_CONTROLLER_H__
#define __BT_VIRTUAL_CONTROLLER_H__

#include <cstddef>
#include <cstdint>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"
#include "bt_virtual_clock.h"

namespace btcompliance {

class VirtualController {
public:
	static const size_t COMMAND_CAPACITY = 32;
	static const size_t ACL_CAPACITY = 64;
	static const size_t SCHEDULED_EVENT_CAPACITY = 32;
	static const size_t EVENT_PACKET_CAPACITY = 257;
	static const size_t ACL_PACKET_CAPACITY = BT_HCI_BUFFER_MAX_SIZE + 8;

	struct CommandRecord {
		uint16_t Opcode;
		uint8_t ParamLen;
		uint8_t Param[255];
	};

	struct PacketRecord {
		size_t Length;
		uint8_t Data[ACL_PACKET_CAPACITY];
	};

	explicit VirtualController(VirtualClock &Clock);
	~VirtualController();

	bool Attach(BtHciDevice_t *pHost);
	void Detach();
	void Reset();

	BtHciDevice_t *Host() const { return vpHost; }
	VirtualClock &Clock() { return vClock; }

	void SetCommandResponse(uint16_t Opcode, uint8_t Status,
		const void *pReturnData = nullptr, uint8_t ReturnLen = 0);
	void ClearCommandResponse();
	void FailAclOnCall(size_t CallNumber);
	void ClearAclFailure();

	size_t CommandCount() const { return vCommandCount; }
	const CommandRecord *CommandAt(size_t Index) const;
	size_t AclCount() const { return vAclCount; }
	const PacketRecord *AclAt(size_t Index) const;
	size_t AclSendCallCount() const { return vAclSendCalls; }

	bool InjectEvent(uint8_t EventCode, const void *pParameters, uint8_t ParameterLen);
	bool InjectLeEvent(uint8_t SubeventCode, const void *pParameters, uint8_t ParameterLen);
	bool InjectEventHeaderOnly(uint8_t EventCode);
	bool InjectLeSubeventOnly(uint8_t SubeventCode);
	bool ScheduleEventUs(uint64_t DelayUs, uint8_t EventCode,
		const void *pParameters, uint8_t ParameterLen);
	bool ScheduleLeEventUs(uint64_t DelayUs, uint8_t SubeventCode,
		const void *pParameters, uint8_t ParameterLen);
	bool CompletePackets(uint16_t ConnHdl, uint16_t Count);

	bool InjectAcl(uint16_t ConnHdl, uint8_t PbFlag,
		const void *pData, uint16_t Length);

private:
	struct CommandResponse {
		bool Enabled;
		uint16_t Opcode;
		uint8_t Status;
		uint8_t ReturnLen;
		uint8_t ReturnData[255];
	};

	struct ScheduledEvent {
		bool Used;
		VirtualController *pOwner;
		size_t Length;
		alignas(4) uint8_t Data[EVENT_PACKET_CAPACITY];
	};

	static uint32_t HostSendData(void *pData, uint32_t Length);
	static uint8_t HostCommand(BtHciDevice_t * const pDev, uint16_t Opcode,
		const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen);
	static void DeliverScheduledEvent(void *pContext);

	uint32_t CaptureAcl(const void *pData, uint32_t Length);
	uint8_t ExecuteCommand(uint16_t Opcode, const void *pParam,
		uint8_t ParamLen, void *pRet, uint8_t RetLen);
	ScheduledEvent *AllocateScheduledEvent();

	static VirtualController *spActive;
	VirtualClock &vClock;
	BtHciDevice_t *vpHost;
	CommandRecord vCommands[COMMAND_CAPACITY];
	PacketRecord vAclPackets[ACL_CAPACITY];
	ScheduledEvent vScheduledEvents[SCHEDULED_EVENT_CAPACITY];
	CommandResponse vCommandResponse;
	size_t vCommandCount;
	size_t vAclCount;
	size_t vAclSendCalls;
	size_t vFailAclCall;
};

} // namespace btcompliance

#endif // __BT_VIRTUAL_CONTROLLER_H__
