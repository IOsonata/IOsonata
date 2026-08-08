#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_hci_ctlr.h"

static_assert(BT_HCI_CAP_CMD_LE_ADD_DEVICE_TO_RESOLVING_LIST == 275U);
static_assert(BT_HCI_CAP_CMD_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST == 276U);
static_assert(BT_HCI_CAP_CMD_LE_CLEAR_RESOLVING_LIST == 277U);
static_assert(BT_HCI_CAP_CMD_LE_READ_RESOLVING_LIST_SIZE == 278U);
static_assert(BT_HCI_CAP_CMD_LE_READ_PEER_RESOLVABLE_ADDRESS == 279U);
static_assert(BT_HCI_CAP_CMD_LE_READ_LOCAL_RESOLVABLE_ADDRESS == 280U);
static_assert(BT_HCI_CAP_CMD_LE_SET_ADDRESS_RESOLUTION_ENABLE == 281U);
static_assert(BT_HCI_CAP_CMD_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT == 282U);
static_assert(BT_HCI_CAP_CMD_LE_SET_PRIVACY_MODE == 314U);
static_assert(BT_HCI_CAP_LE_FEATURE_LL_PRIVACY == 6U);

namespace {

int s_Checks;
int s_Failures;
bool s_LlPrivacySupported;
bool s_PrivacyCommandsSupported;
bool s_PrivacyModeSupported;
uint8_t s_ResolvingListSize;
uint16_t s_FailOpcode;
int s_FailOccurrence;
int s_FailSeen;

struct CapturedCommand {
	uint16_t Opcode;
	uint8_t ParamLen;
	uint8_t Param[40];
};

CapturedCommand s_PrivacyCommand[24];
size_t s_PrivacyCommandCount;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

void CopyReturn(void *pRet, uint8_t RetLen, const void *pData, size_t DataLen)
{
	if (pRet != nullptr && RetLen != 0)
	{
		size_t len = RetLen < DataLen ? RetLen : DataLen;
		std::memcpy(pRet, pData, len);
	}
}

bool IsPrivacyWrite(uint16_t Opcode)
{
	return Opcode == BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE ||
		Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR ||
		Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV ||
		Opcode == BT_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT ||
		Opcode == BT_HCI_CMD_CTLR_SET_PRIVACY_MODE;
}

uint8_t TestCommand(BtHciDevice_t * const, uint16_t Opcode,
	const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
{
	if (IsPrivacyWrite(Opcode))
	{
		if (s_PrivacyCommandCount <
			sizeof(s_PrivacyCommand) / sizeof(s_PrivacyCommand[0]))
		{
			CapturedCommand &cmd = s_PrivacyCommand[s_PrivacyCommandCount++];
			cmd.Opcode = Opcode;
			cmd.ParamLen = ParamLen;
			std::memset(cmd.Param, 0, sizeof(cmd.Param));
			if (pParam != nullptr && ParamLen <= sizeof(cmd.Param))
			{
				std::memcpy(cmd.Param, pParam, ParamLen);
			}
		}

		if (Opcode == s_FailOpcode)
		{
			++s_FailSeen;
			if (s_FailOccurrence > 0 && s_FailSeen == s_FailOccurrence)
			{
				return 0x0C;
			}
		}
	}

	uint8_t data[64] = {};
	switch (Opcode)
	{
		case BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO:
		{
			const uint8_t value[8] = { 0x0D, 0, 0, 0x0D, 0x59, 0, 0, 0 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			return 0;
		}

		case BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS:
			if (s_PrivacyCommandsSupported)
			{
				// Literal Core Supported Commands bitmap positions.
				data[34] = 0xF8;	// resolving-list add/remove/clear/read/read peer RPA
				data[35] = 0x07;	// read local RPA, resolution enable, RPA timeout
				if (s_PrivacyModeSupported)
				{
					data[39] = 0x04;	// LE Set Privacy Mode
				}
			}
			CopyReturn(pRet, RetLen, data, sizeof(data));
			return 0;

		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			if (s_LlPrivacySupported)
			{
				data[0] = 0x40;	// LL Privacy, feature bit 6
			}
			CopyReturn(pRet, RetLen, data, 8);
			return 0;

		case BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES:
			CopyReturn(pRet, RetLen, data, 8);
			return 0;

		case BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE:
			data[0] = s_ResolvingListSize;
			CopyReturn(pRet, RetLen, data, 1);
			return 0;

		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT:
		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE:
		case BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN:
		case BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN:
		case BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS:
		case BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE:
			return 1;

		default:
			return IsPrivacyWrite(Opcode) ? 0 : 1;
	}
}

void Reset()
{
	s_LlPrivacySupported = true;
	s_PrivacyCommandsSupported = true;
	s_PrivacyModeSupported = true;
	s_ResolvingListSize = 8;
	s_FailOpcode = 0;
	s_FailOccurrence = 0;
	s_FailSeen = 0;
	s_PrivacyCommandCount = 0;
	std::memset(s_PrivacyCommand, 0, sizeof(s_PrivacyCommand));
}

BtHciDevice_t MakeDevice()
{
	BtHciDevice_t dev = {};
	dev.Command = TestCommand;
	return dev;
}

void FillIrk(uint8_t Irk[16], uint8_t Base)
{
	for (int i = 0; i < 16; i++)
	{
		Irk[i] = static_cast<uint8_t>(Base + i);
	}
}

BtHciPrivacyEntry_t MakeEntry(uint8_t AddrType, uint8_t AddressBase,
	uint8_t IrkBase)
{
	BtHciPrivacyEntry_t entry = {};
	entry.PeerIdentityAddrType = AddrType;
	for (int i = 0; i < 6; i++)
	{
		entry.PeerIdentityAddr[i] = static_cast<uint8_t>(AddressBase + i);
	}
	FillIrk(entry.PeerIrk, IrkBase);
	return entry;
}

void CheckAddCommand(const CapturedCommand &cmd,
	const BtHciPrivacyEntry_t &entry, const uint8_t LocalIrk[16])
{
	CHECK(cmd.Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV);
	CHECK(cmd.ParamLen == 39);
	CHECK(cmd.Param[0] == entry.PeerIdentityAddrType);
	CHECK(std::memcmp(&cmd.Param[1], entry.PeerIdentityAddr, 6) == 0);
	CHECK(std::memcmp(&cmd.Param[7], entry.PeerIrk, 16) == 0);
	CHECK(std::memcmp(&cmd.Param[23], LocalIrk, 16) == 0);
}

void TestNetworkPrivacySequence()
{
	Reset();
	BtHciDevice_t dev = MakeDevice();
	BtHciPrivacyEntry_t entry[2] = {
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_PUBLIC, 0x10, 0x20),
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_RANDOM, 0x30, 0x40),
	};
	uint8_t localIrk[16];
	FillIrk(localIrk, 0x80);

	CHECK(BtHciPrivacyConfigure(&dev, entry, 2, localIrk,
		BT_HCI_PRIVACY_RPA_TIMEOUT_DEFAULT,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 6);
	if (s_PrivacyCommandCount == 6)
	{
		CHECK(s_PrivacyCommand[0].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[0].ParamLen == 1);
		CHECK(s_PrivacyCommand[0].Param[0] == 0);
		CHECK(s_PrivacyCommand[1].Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR);
		CHECK(s_PrivacyCommand[1].ParamLen == 0);
		CHECK(s_PrivacyCommand[2].Opcode ==
			BT_HCI_CMD_CTLR_SET_RESOLVABLE_PRIVATE_ADDR_TIMEOUT);
		CHECK(s_PrivacyCommand[2].ParamLen == 2);
		CHECK(s_PrivacyCommand[2].Param[0] == 0x84);
		CHECK(s_PrivacyCommand[2].Param[1] == 0x03);
		CheckAddCommand(s_PrivacyCommand[3], entry[0], localIrk);
		CheckAddCommand(s_PrivacyCommand[4], entry[1], localIrk);
		CHECK(s_PrivacyCommand[5].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[5].Param[0] == 1);
	}
}

void TestDevicePrivacySequence()
{
	Reset();
	BtHciDevice_t dev = MakeDevice();
	BtHciPrivacyEntry_t entry =
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_RANDOM, 0x50, 0x60);
	uint8_t localIrk[16];
	FillIrk(localIrk, 0x90);

	CHECK(BtHciPrivacyConfigure(&dev, &entry, 1, localIrk, 60,
		BT_HCI_PRIVACY_MODE_DEVICE));
	CHECK(s_PrivacyCommandCount == 6);
	if (s_PrivacyCommandCount == 6)
	{
		CheckAddCommand(s_PrivacyCommand[3], entry, localIrk);
		const CapturedCommand &mode = s_PrivacyCommand[4];
		CHECK(mode.Opcode == BT_HCI_CMD_CTLR_SET_PRIVACY_MODE);
		CHECK(mode.ParamLen == 8);
		CHECK(mode.Param[0] == entry.PeerIdentityAddrType);
		CHECK(std::memcmp(&mode.Param[1], entry.PeerIdentityAddr, 6) == 0);
		CHECK(mode.Param[7] == BT_HCI_PRIVACY_MODE_DEVICE);
		CHECK(s_PrivacyCommand[5].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[5].Param[0] == 1);
	}
}

void TestCapacityAndCapabilityRefusal()
{
	BtHciPrivacyEntry_t entry[2] = {
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_PUBLIC, 1, 0x20),
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_RANDOM, 7, 0x40),
	};
	uint8_t localIrk[16];
	FillIrk(localIrk, 0x80);

	Reset();
	s_ResolvingListSize = 1;
	BtHciDevice_t dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, entry, 2, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	s_LlPrivacySupported = false;
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	s_PrivacyCommandsSupported = false;
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	s_PrivacyModeSupported = false;
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_DEVICE));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	s_PrivacyModeSupported = false;
	dev = MakeDevice();
	CHECK(BtHciPrivacyConfigure(&dev, entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
}

void TestPartialAddFailureClearsList()
{
	Reset();
	BtHciDevice_t dev = MakeDevice();
	BtHciPrivacyEntry_t entry[2] = {
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_PUBLIC, 1, 0x20),
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_RANDOM, 7, 0x40),
	};
	uint8_t localIrk[16];
	FillIrk(localIrk, 0x80);

	s_FailOpcode = BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV;
	s_FailOccurrence = 2;
	CHECK(!BtHciPrivacyConfigure(&dev, entry, 2, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 7);
	if (s_PrivacyCommandCount == 7)
	{
		CHECK(s_PrivacyCommand[0].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[0].Param[0] == 0);
		CHECK(s_PrivacyCommand[3].Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV);
		CHECK(s_PrivacyCommand[4].Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_ADD_DEV);
		CHECK(s_PrivacyCommand[5].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[5].Param[0] == 0);
		CHECK(s_PrivacyCommand[6].Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR);
	}
}

void TestEmptyListDisablesResolution()
{
	Reset();
	s_ResolvingListSize = 0;
	BtHciDevice_t dev = MakeDevice();
	CHECK(BtHciPrivacyConfigure(&dev, nullptr, 0, nullptr, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 2);
	if (s_PrivacyCommandCount == 2)
	{
		CHECK(s_PrivacyCommand[0].Opcode ==
			BT_HCI_CMD_CTLR_SET_ADDR_RESOLUTION_ENABLE);
		CHECK(s_PrivacyCommand[0].Param[0] == 0);
		CHECK(s_PrivacyCommand[1].Opcode == BT_HCI_CMD_CTLR_RESOLVING_LIST_CLEAR);
	}
}

void TestInvalidArguments()
{
	BtHciPrivacyEntry_t entry =
		MakeEntry(BT_HCI_PRIVACY_ID_ADDR_PUBLIC, 1, 0x20);
	uint8_t localIrk[16];
	FillIrk(localIrk, 0x80);
	BtHciDevice_t dev;

	Reset();
	CHECK(!BtHciPrivacyConfigure(nullptr, &entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));

	Reset();
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, localIrk, 0,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, localIrk,
		static_cast<uint16_t>(BT_HCI_PRIVACY_RPA_TIMEOUT_MAX + 1U),
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	dev = MakeDevice();
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, localIrk, 900, 2));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	dev = MakeDevice();
	entry.PeerIdentityAddrType = 2;
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	dev = MakeDevice();
	entry = MakeEntry(BT_HCI_PRIVACY_ID_ADDR_PUBLIC, 1, 0x20);
	uint8_t zeroIrk[16] = {};
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, zeroIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);

	Reset();
	dev = MakeDevice();
	std::memset(entry.PeerIrk, 0, sizeof(entry.PeerIrk));
	CHECK(!BtHciPrivacyConfigure(&dev, &entry, 1, localIrk, 900,
		BT_HCI_PRIVACY_MODE_NETWORK));
	CHECK(s_PrivacyCommandCount == 0);
}

} // namespace

int main()
{
	TestNetworkPrivacySequence();
	TestDevicePrivacySequence();
	TestCapacityAndCapabilityRefusal();
	TestPartialAddFailureClearsList();
	TestEmptyListDisablesResolution();
	TestInvalidArguments();

	if (s_Failures != 0)
	{
		std::printf("Bluetooth HCI privacy tests: %d failure(s), %d checks\n",
			s_Failures, s_Checks);
		return 1;
	}

	std::printf("Bluetooth HCI privacy tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
