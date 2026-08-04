#ifndef BT_VIRTUAL_LE_LINK_H
#define BT_VIRTUAL_LE_LINK_H

#include <stddef.h>
#include <stdint.h>

namespace btcompliance {

enum class LeAddressType : uint8_t {
	Public = 0,
	RandomStatic,
	ResolvablePrivate,
	NonResolvablePrivate,
};

enum class LeConnectionType : uint8_t {
	LegacyUndirected = 0,
	LegacyDirected,
	Extended,
};

enum class SecurityInitiator : uint8_t {
	Central = 0,
	PeripheralRequest,
};

struct LeConnectionCase {
	LeAddressType CentralAddress;
	LeAddressType PeripheralAddress;
	LeConnectionType Type;
	SecurityInitiator SecurityStart;
};

class VirtualLeLink {
public:
	static constexpr size_t ADDRESS_TYPE_COUNT = 4;
	static constexpr size_t CONNECTION_TYPE_COUNT = 3;
	static constexpr size_t SECURITY_START_COUNT = 2;
	static constexpr size_t CASE_COUNT = ADDRESS_TYPE_COUNT *
		ADDRESS_TYPE_COUNT * CONNECTION_TYPE_COUNT * SECURITY_START_COUNT;

	static LeConnectionCase CaseAt(size_t Index)
	{
		LeConnectionCase c = {};
		c.SecurityStart = static_cast<SecurityInitiator>(Index % SECURITY_START_COUNT);
		Index /= SECURITY_START_COUNT;
		c.Type = static_cast<LeConnectionType>(Index % CONNECTION_TYPE_COUNT);
		Index /= CONNECTION_TYPE_COUNT;
		c.PeripheralAddress = static_cast<LeAddressType>(Index % ADDRESS_TYPE_COUNT);
		Index /= ADDRESS_TYPE_COUNT;
		c.CentralAddress = static_cast<LeAddressType>(Index % ADDRESS_TYPE_COUNT);
		return c;
	}

	static bool Valid(const LeConnectionCase &Case)
	{
		return static_cast<uint8_t>(Case.CentralAddress) < ADDRESS_TYPE_COUNT &&
			static_cast<uint8_t>(Case.PeripheralAddress) < ADDRESS_TYPE_COUNT &&
			static_cast<uint8_t>(Case.Type) < CONNECTION_TYPE_COUNT &&
			static_cast<uint8_t>(Case.SecurityStart) < SECURITY_START_COUNT;
	}

	VirtualLeLink() : vConnected(false), vCentralHandle(0), vPeripheralHandle(0),
		vForwardedCentralToPeripheral(0), vForwardedPeripheralToCentral(0)
	{
	}

	bool Connect(const LeConnectionCase &Case, uint16_t CentralHandle,
		uint16_t PeripheralHandle)
	{
		if (vConnected || !Valid(Case) || CentralHandle == 0 || PeripheralHandle == 0)
		{
			return false;
		}
		vCase = Case;
		vCentralHandle = CentralHandle;
		vPeripheralHandle = PeripheralHandle;
		vConnected = true;
		return true;
	}

	void Disconnect()
	{
		vConnected = false;
		vCentralHandle = 0;
		vPeripheralHandle = 0;
	}

	bool ForwardFromCentral(const uint8_t *pData, size_t Len)
	{
		if (!vConnected || (Len != 0 && pData == nullptr))
		{
			return false;
		}
		vForwardedCentralToPeripheral += Len;
		return true;
	}

	bool ForwardFromPeripheral(const uint8_t *pData, size_t Len)
	{
		if (!vConnected || (Len != 0 && pData == nullptr))
		{
			return false;
		}
		vForwardedPeripheralToCentral += Len;
		return true;
	}

	bool Connected() const { return vConnected; }
	uint16_t CentralHandle() const { return vCentralHandle; }
	uint16_t PeripheralHandle() const { return vPeripheralHandle; }
	size_t CentralToPeripheralBytes() const { return vForwardedCentralToPeripheral; }
	size_t PeripheralToCentralBytes() const { return vForwardedPeripheralToCentral; }
	const LeConnectionCase &ConnectionCase() const { return vCase; }

private:
	LeConnectionCase vCase = {};
	bool vConnected;
	uint16_t vCentralHandle;
	uint16_t vPeripheralHandle;
	size_t vForwardedCentralToPeripheral;
	size_t vForwardedPeripheralToCentral;
};

} // namespace btcompliance

#endif // BT_VIRTUAL_LE_LINK_H
