#ifndef BT_VIRTUAL_OOB_H
#define BT_VIRTUAL_OOB_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace btcompliance {

struct ScOobData {
	uint8_t Random[16];
	uint8_t Confirm[16];
};

enum class OobFault : uint8_t {
	None = 0,
	Drop,
	CorruptRandom,
	CorruptConfirm,
	Replay,
};

class VirtualOobChannel {
public:
	VirtualOobChannel()
	{
		Reset();
	}

	void Reset()
	{
		memset(vData, 0, sizeof(vData));
		memset(vValid, 0, sizeof(vValid));
		memset(vConsumed, 0, sizeof(vConsumed));
	}

	bool Publish(unsigned Side, const ScOobData &Data)
	{
		if (Side > 1)
		{
			return false;
		}
		vData[Side] = Data;
		vValid[Side] = true;
		vConsumed[Side] = false;
		return true;
	}

	bool Deliver(unsigned FromSide, ScOobData *pData, OobFault Fault)
	{
		if (FromSide > 1 || pData == nullptr || !vValid[FromSide])
		{
			return false;
		}
		if (Fault == OobFault::Drop)
		{
			return false;
		}
		if (vConsumed[FromSide] && Fault != OobFault::Replay)
		{
			return false;
		}

		*pData = vData[FromSide];
		if (Fault == OobFault::CorruptRandom)
		{
			pData->Random[0] ^= 0x80;
		}
		else if (Fault == OobFault::CorruptConfirm)
		{
			pData->Confirm[15] ^= 0x01;
		}
		vConsumed[FromSide] = true;
		return true;
	}

	bool Consumed(unsigned Side) const
	{
		return Side <= 1 && vConsumed[Side];
	}

private:
	ScOobData vData[2];
	bool vValid[2];
	bool vConsumed[2];
};

} // namespace btcompliance

#endif // BT_VIRTUAL_OOB_H
