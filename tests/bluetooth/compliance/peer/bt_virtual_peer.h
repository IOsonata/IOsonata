#ifndef __BT_VIRTUAL_PEER_H__
#define __BT_VIRTUAL_PEER_H__

#include <cstddef>
#include <cstdint>

#include "bt_virtual_controller.h"

namespace btcompliance {

class VirtualPeer {
public:
	explicit VirtualPeer(VirtualController &Controller, uint16_t ConnHdl = 1);

	void SetConnectionHandle(uint16_t ConnHdl) { vConnHdl = ConnHdl; }
	uint16_t ConnectionHandle() const { return vConnHdl; }

	bool SendL2cap(uint16_t Cid, const void *pPayload, uint16_t PayloadLen,
		uint16_t FragmentPayloadLen = 0);
	bool SendAtt(const void *pPdu, uint16_t Length, uint16_t FragmentPayloadLen = 0);
	bool SendSmp(const void *pPdu, uint16_t Length, uint16_t FragmentPayloadLen = 0);
	bool SendOrphanContinuation(const void *pData, uint16_t Length);

	void ResetHostReader();
	bool ReadHostL2cap(uint16_t *pCid, void *pPayload, size_t PayloadCapacity,
		size_t *pPayloadLen);

private:
	VirtualController &vController;
	uint16_t vConnHdl;
	size_t vHostAclReadIndex;
	bool vHostReassemblyActive;
	uint16_t vHostExpected;
	uint16_t vHostReceived;
	uint8_t vHostBuffer[BT_HCI_BUFFER_MAX_SIZE];
};

} // namespace btcompliance

#endif // __BT_VIRTUAL_PEER_H__
