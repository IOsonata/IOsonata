#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_hci_ctlr.h"

namespace {

int s_Checks;
int s_Failures;
int s_Mode;
uint16_t s_Opcode[16];
size_t s_OpcodeCount;

#define CHECK(expr) do { \
	s_Checks++; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		s_Failures++; \
	} \
} while (0)

void CopyReturn(void *pRet, uint8_t RetLen, const uint8_t *pData, size_t DataLen)
{
	if (pRet != nullptr && RetLen != 0)
	{
		size_t len = RetLen < DataLen ? RetLen : DataLen;
		std::memcpy(pRet, pData, len);
	}
}

void SetCommand(BtHciCapabilities_t *pCapabilities, uint16_t CommandBit)
{
	pCapabilities->SupportedCommands[CommandBit >> 3] |=
		static_cast<uint8_t>(1U << (CommandBit & 7U));
}

uint8_t CapabilityCommand(BtHciDevice_t * const, uint16_t OpCode,
	const void *, uint8_t, void *pRet, uint8_t RetLen)
{
	if (s_OpcodeCount < sizeof(s_Opcode) / sizeof(s_Opcode[0]))
	{
		s_Opcode[s_OpcodeCount++] = OpCode;
	}

	if (s_Mode == 2 && OpCode == BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS)
	{
		return 1;
	}

	if (s_Mode == 1)
	{
		if (OpCode == BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT)
		{
			return 1;
		}
		if (OpCode != BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO &&
			OpCode != BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS &&
			OpCode != BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES &&
			OpCode != BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES &&
			OpCode != BT_HCI_CMD_CTLR_READ_BUFF_SIZE)
		{
			return 1;
		}
	}

	uint8_t data[64] = {};
	switch (OpCode)
	{
		case BT_HCI_CMD_INFO_READ_LOCAL_VERS_INFO:
		{
			const uint8_t value[8] = { 0x0D, 0x34, 0x12, 0x0D, 0x59, 0x00, 0x78, 0x56 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			break;
		}
		case BT_HCI_CMD_INFO_READ_LOCAL_SUPPORTED_COMMANDS:
			for (size_t i = 0; i < sizeof(data); i++)
			{
				data[i] = static_cast<uint8_t>(i);
			}
			data[25] |= 0xB0;
			data[26] |= 0x03;
			data[35] |= 0xC0;
			data[36] |= 0x1F;
			CopyReturn(pRet, RetLen, data, sizeof(data));
			break;
		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			std::memset(data, 0xA5, 8);
			data[1] |= 0x10;
			CopyReturn(pRet, RetLen, data, 8);
			break;
		case BT_HCI_CMD_CTLR_READ_SUPPORTED_STATES:
			std::memset(data, 0x5A, 8);
			CopyReturn(pRet, RetLen, data, 8);
			break;
		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT:
		{
			const uint8_t value[6] = { 0xFB, 0x00, 7, 0x78, 0x00, 2 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			break;
		}
		case BT_HCI_CMD_CTLR_READ_BUFF_SIZE:
		{
			const uint8_t value[3] = { 0x1B, 0x00, 4 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			break;
		}
		case BT_HCI_CMD_CTLR_READ_MAX_DATA_LEN:
		{
			const uint8_t value[8] = { 0xFB, 0x00, 0x48, 0x08, 0xFB, 0x00, 0x48, 0x08 };
			CopyReturn(pRet, RetLen, value, sizeof(value));
			break;
		}
		case BT_HCI_CMD_CTLR_RESOLVING_LIST_READ_SIZE:
			data[0] = 8;
			CopyReturn(pRet, RetLen, data, 1);
			break;
		case BT_HCI_CMD_CTLR_READ_MAX_ADV_DATA_LEN:
			data[0] = 0x72;
			data[1] = 0x06;
			CopyReturn(pRet, RetLen, data, 2);
			break;
		case BT_HCI_CMD_CTLR_READ_NB_SUPPORTED_ADV_SETS:
			data[0] = 4;
			CopyReturn(pRet, RetLen, data, 1);
			break;
		case BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE:
			data[0] = 3;
			CopyReturn(pRet, RetLen, data, 1);
			break;
		default:
			return 1;
	}
	return 0;
}

void Reset(int Mode)
{
	s_Mode = Mode;
	s_OpcodeCount = 0;
	std::memset(s_Opcode, 0, sizeof(s_Opcode));
}

void TestFullDiscovery()
{
	Reset(0);
	BtHciDevice_t dev = {};
	dev.Command = CapabilityCommand;
	BtHciCapabilities_t caps = {};

	CHECK(BtHciCapabilitiesRead(&dev, &caps));
	CHECK(caps.Valid == ((1UL << 10) - 1));
	CHECK(caps.HciVersion == 0x0D);
	CHECK(caps.HciRevision == 0x1234);
	CHECK(caps.LmpVersion == 0x0D);
	CHECK(caps.Manufacturer == 0x0059);
	CHECK(caps.LmpSubversion == 0x5678);
	CHECK(caps.SupportedCommands[0] == 0);
	CHECK(caps.SupportedCommands[63] == 63);
	CHECK(caps.LeFeatures[0] == 0xA5);
	CHECK(caps.LeStates[7] == 0x5A);
	CHECK(caps.LeAclDataLen == 251);
	CHECK(caps.LeAclPacketCount == 7);
	CHECK(caps.IsoDataLen == 120);
	CHECK(caps.IsoPacketCount == 2);
	CHECK(caps.MaxTxOctets == 251);
	CHECK(caps.MaxTxTime == 2120);
	CHECK(caps.MaxRxOctets == 251);
	CHECK(caps.MaxRxTime == 2120);
	CHECK(caps.ResolvingListSize == 8);
	CHECK(caps.MaxAdvDataLen == 1650);
	CHECK(caps.AdvSetCount == 4);
	CHECK(caps.PeriodicAdvListSize == 3);
	CHECK(s_OpcodeCount == 10);
	CHECK(BtHciCapabilitiesCommandsKnown(&caps));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_EXT_ADV));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_PHY_2M));
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, true, true));
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, true, true));
}

void TestOptionalFallback()
{
	Reset(1);
	BtHciDevice_t dev = {};
	dev.Command = CapabilityCommand;
	BtHciCapabilities_t caps = {};

	CHECK(BtHciCapabilitiesRead(&dev, &caps));
	CHECK(caps.Valid == (BT_HCI_CAP_VALID_VERSION |
		BT_HCI_CAP_VALID_COMMANDS |
		BT_HCI_CAP_VALID_LE_FEATURES |
		BT_HCI_CAP_VALID_LE_STATES |
		BT_HCI_CAP_VALID_BUFFER_SIZE));
	CHECK(caps.LeAclDataLen == 27);
	CHECK(caps.LeAclPacketCount == 4);
	CHECK(caps.IsoDataLen == 0);
	CHECK(caps.IsoPacketCount == 0);
	CHECK(s_Opcode[4] == BT_HCI_CMD_CTLR_READ_BUFF_SIZE_EXT);
	CHECK(s_Opcode[5] == BT_HCI_CMD_CTLR_READ_BUFF_SIZE);
}

void TestMandatoryFailure()
{
	Reset(2);
	BtHciDevice_t dev = {};
	dev.Command = CapabilityCommand;
	BtHciCapabilities_t caps;
	std::memset(&caps, 0xA5, sizeof(caps));

	CHECK(BtHciCapabilitiesRead(&dev, &caps) == false);
	CHECK(caps.Valid == 0);
	CHECK(caps.HciVersion == 0);
	CHECK(s_OpcodeCount == 2);
}

void TestArguments()
{
	BtHciDevice_t dev = {};
	BtHciCapabilities_t caps;
	std::memset(&caps, 0xA5, sizeof(caps));
	CHECK(BtHciCapabilitiesRead(nullptr, &caps) == false);
	CHECK(caps.Valid == 0);
	std::memset(&caps, 0xA5, sizeof(caps));
	CHECK(BtHciCapabilitiesRead(&dev, &caps) == false);
	CHECK(caps.Valid == 0);
	dev.Command = CapabilityCommand;
	CHECK(BtHciCapabilitiesRead(&dev, nullptr) == false);
}

void TestControllerCapabilityStorage()
{
	alignas(4) static uint8_t fifo[1024] = {};
	static BtHciCtlrDev_t ctlr = {};
	static BtHciDevice_t hci = {};
	BtHciCtlrCfg_t cfg = {};
	cfg.PacketSize = 64;
	cfg.pRxFifoMem = fifo;
	cfg.RxFifoMemSize = sizeof(fifo);

	CHECK(BtHciCtlrEnable(&ctlr, &cfg));
	CHECK(ctlr.Capabilities.Valid == 0);
	CHECK(ctlr.pHciDev == nullptr);
	CHECK(ctlr.CapabilitiesApplied == false);
	CHECK(BtHciCtlrCapabilitiesGet(&ctlr) == &ctlr.Capabilities);
	CHECK(BtHciCtlrCapabilitiesGet(nullptr) == nullptr);
	CHECK(BtHciCapabilitiesForDeviceGet(&hci) == nullptr);
	ctlr.pHciDev = &hci;
	CHECK(BtHciCapabilitiesForDeviceGet(&hci) == &ctlr.Capabilities);
	CHECK(BtHciCapabilitiesForDeviceGet(nullptr) == nullptr);
}

void TestCapabilityPredicates()
{
	BtHciCapabilities_t caps = {};

	CHECK(BtHciCapabilitiesCommandsKnown(nullptr) == false);
	CHECK(BtHciCapabilitiesCommandsKnown(&caps) == false);
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS) == false);

	caps.Valid = BT_HCI_CAP_VALID_COMMANDS;
	caps.SupportedCommands[35] = 0x80;
	CHECK(BtHciCapabilitiesCommandsKnown(&caps));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS) == false);
	CHECK(BtHciCapabilitiesCommandSupported(&caps, 512) == false);

	CHECK(BtHciCapabilitiesLeFeaturesKnown(&caps) == false);
	caps.Valid |= BT_HCI_CAP_VALID_LE_FEATURES;
	caps.LeFeatures[1] = 0x10;
	CHECK(BtHciCapabilitiesLeFeaturesKnown(&caps));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_EXT_ADV));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps, 64) == false);
}

void TestAdvertisingCommandChecks()
{
	BtHciCapabilities_t caps = {};

	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, false, false) == false);
	caps.Valid = BT_HCI_CAP_VALID_COMMANDS;
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_ADV_PARAMETERS);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_ADV_DATA);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_ADV_ENABLE);
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, false, false));
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, true, false) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_RANDOM_ADDRESS);
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, true, false));
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, false, true) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_SCAN_RESPONSE_DATA);
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, false, true));

	std::memset(&caps, 0, sizeof(caps));
	caps.Valid = BT_HCI_CAP_VALID_COMMANDS | BT_HCI_CAP_VALID_LE_FEATURES;
	caps.LeFeatures[1] = 0x10;
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_PARAMETERS);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_ADV_ENABLE);
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, false, false));
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, true, false) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_ADV_SET_RANDOM_ADDRESS);
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, true, false));
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, false, true) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_RESPONSE_DATA);
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, false, true));

	caps.LeFeatures[1] &= static_cast<uint8_t>(~0x10U);
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, false, false) == false);
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_PHY_2M) == false);
	caps.LeFeatures[1] |= 0x01;
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_PHY_2M));

	caps.LeFeatures[1] |= 0x10;
	caps.SupportedCommands[BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA >> 3] &=
		static_cast<uint8_t>(~(1U <<
		(BT_HCI_CAP_CMD_LE_SET_EXT_ADV_DATA & 7U)));
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, false, false) == false);
}

} // namespace

int main()
{
	TestFullDiscovery();
	TestOptionalFallback();
	TestMandatoryFailure();
	TestArguments();
	TestControllerCapabilityStorage();
	TestCapabilityPredicates();
	TestAdvertisingCommandChecks();

	if (s_Failures == 0)
	{
		std::printf("Bluetooth HCI capability tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Bluetooth HCI capability tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
