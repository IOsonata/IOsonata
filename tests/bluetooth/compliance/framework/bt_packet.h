#ifndef __BT_COMPLIANCE_PACKET_H__
#define __BT_COMPLIANCE_PACKET_H__

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace btcompliance {

class PacketBuilder {
public:
	PacketBuilder(uint8_t *pBuffer, size_t Capacity) :
		vpBuffer(pBuffer), vCapacity(Capacity), vLength(0), vValid(pBuffer != nullptr)
	{
	}

	void Reset()
	{
		vLength = 0;
		vValid = vpBuffer != nullptr;
	}

	bool U8(uint8_t Value)
	{
		return Bytes(&Value, 1);
	}

	bool Le16(uint16_t Value)
	{
		uint8_t data[2] = {
			static_cast<uint8_t>(Value & 0xFFU),
			static_cast<uint8_t>((Value >> 8) & 0xFFU)
		};
		return Bytes(data, sizeof(data));
	}

	bool Le32(uint32_t Value)
	{
		uint8_t data[4] = {
			static_cast<uint8_t>(Value & 0xFFU),
			static_cast<uint8_t>((Value >> 8) & 0xFFU),
			static_cast<uint8_t>((Value >> 16) & 0xFFU),
			static_cast<uint8_t>((Value >> 24) & 0xFFU)
		};
		return Bytes(data, sizeof(data));
	}

	bool Bytes(const void *pData, size_t Length)
	{
		if (!vValid || Length > vCapacity - vLength || (Length != 0 && pData == nullptr))
		{
			vValid = false;
			return false;
		}

		if (Length != 0)
		{
			std::memcpy(vpBuffer + vLength, pData, Length);
		}
		vLength += Length;
		return true;
	}

	bool PatchU8(size_t Offset, uint8_t Value)
	{
		if (!vValid || Offset >= vLength)
		{
			vValid = false;
			return false;
		}
		vpBuffer[Offset] = Value;
		return true;
	}

	bool PatchLe16(size_t Offset, uint16_t Value)
	{
		if (!vValid || Offset + 2 > vLength)
		{
			vValid = false;
			return false;
		}
		vpBuffer[Offset] = static_cast<uint8_t>(Value & 0xFFU);
		vpBuffer[Offset + 1] = static_cast<uint8_t>((Value >> 8) & 0xFFU);
		return true;
	}

	uint8_t *Data() { return vpBuffer; }
	const uint8_t *Data() const { return vpBuffer; }
	size_t Length() const { return vLength; }
	bool Valid() const { return vValid; }

private:
	uint8_t *vpBuffer;
	size_t vCapacity;
	size_t vLength;
	bool vValid;
};

} // namespace btcompliance

#endif // __BT_COMPLIANCE_PACKET_H__
