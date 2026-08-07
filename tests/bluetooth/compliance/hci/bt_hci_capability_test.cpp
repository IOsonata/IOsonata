#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_hci_cap.h"
#include "bluetooth/bt_hci_ctlr.h"

static_assert(BT_HCI_CAP_CMD_LE_SET_DATA_LENGTH == 270U);

namespace {

int s_Checks;
int s_Failures;
int s_Mode;
uint16_t s_Opcode[16];
size_t s_OpcodeCount;
uint8_t s_DataLengthStatus;
uint8_t s_DataLengthParam[6];
uint8_t s_DataLengthParamLen;
int s_DataLengthCount;
uint16_t s_DataLengthReturnHdl;
bool s_DleCommandSupported;
bool s_DleFeatureSupported;
bool s_MaxDataLengthSupported;
uint16_t s_MaxTxOctets;
uint16_t s_MaxTxTime;

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
	const void *pParam, uint8_t ParamLen, void *pRet, uint8_t RetLen)
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
			data[26] |= 0x1F;
			if (s_DleCommandSupported)
			{
				data[33] |= 0x40;
			}
			else
			{
				data[33] &= static_cast<uint8_t>(~0x40U);
			}
			data[35] |= 0xC0;
			data[36] |= 0x1F;
			data[37] |= 0x1C;
			CopyReturn(pRet, RetLen, data, sizeof(data));
			break;
		case BT_HCI_CMD_CTLR_READ_LOCAL_SUPP_FEATURES:
			std::memset(data, 0xA5, 8);
			if (s_DleFeatureSupported)
			{
				data[0] |= 0x20;
			}
			else
			{
				data[0] &= static_cast<uint8_t>(~0x20U);
			}
			data[1] |= 0x18;
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
			if (!s_MaxDataLengthSupported)
			{
				return 1;
			}
			const uint8_t value[8] = {
				static_cast<uint8_t>(s_MaxTxOctets),
				static_cast<uint8_t>(s_MaxTxOctets >> 8),
				static_cast<uint8_t>(s_MaxTxTime),
				static_cast<uint8_t>(s_MaxTxTime >> 8),
				0xFB, 0x00, 0x48, 0x08
			};
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
		case BT_HCI_CMD_CTLR_SET_DATA_LEN:
			s_DataLengthCount++;
			s_DataLengthParamLen = ParamLen;
			std::memset(s_DataLengthParam, 0, sizeof(s_DataLengthParam));
			if (pParam != nullptr && ParamLen <= sizeof(s_DataLengthParam))
			{
				std::memcpy(s_DataLengthParam, pParam, ParamLen);
			}
			if (s_DataLengthStatus == 0)
			{
				data[0] = static_cast<uint8_t>(s_DataLengthReturnHdl);
				data[1] = static_cast<uint8_t>(s_DataLengthReturnHdl >> 8);
				CopyReturn(pRet, RetLen, data, 2);
			}
			return s_DataLengthStatus;
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
	s_DataLengthStatus = 0;
	std::memset(s_DataLengthParam, 0, sizeof(s_DataLengthParam));
	s_DataLengthParamLen = 0;
	s_DataLengthCount = 0;
	s_DataLengthReturnHdl = 0x0042;
	s_DleCommandSupported = true;
	s_DleFeatureSupported = true;
	s_MaxDataLengthSupported = true;
	s_MaxTxOctets = 251;
	s_MaxTxTime = 2120;
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
		BT_HCI_CAP_CMD_LE_SET_SCAN_PARAMETERS));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_CREATE_CONNECTION));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_DATA_LENGTH));
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
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE));
	CHECK(BtHciCapabilitiesCommandSupported(&caps,
		BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_DATA_LENGTH_EXT));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_EXT_ADV));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_PHY_2M));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_CODED_PHY));
	CHECK(BtHciCapabilitiesLegacyAdvertisingSupported(&caps, true, true));
	CHECK(BtHciCapabilitiesExtendedAdvertisingSupported(&caps, true, true));
	CHECK(BtHciCapabilitiesLegacyScanningSupported(&caps));
	CHECK(BtHciCapabilitiesExtendedScanningSupported(&caps));
	CHECK(BtHciCapabilitiesLegacyInitiatingSupported(&caps));
	CHECK(BtHciCapabilitiesExtendedInitiatingSupported(&caps));
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
	caps.LeFeatures[1] = 0x18;
	CHECK(BtHciCapabilitiesLeFeaturesKnown(&caps));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_EXT_ADV));
	CHECK(BtHciCapabilitiesLeFeatureSupported(&caps,
		BT_HCI_CAP_LE_FEATURE_CODED_PHY));
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

void TestGapCommandChecks()
{
	BtHciCapabilities_t caps = {};
	caps.Valid = BT_HCI_CAP_VALID_COMMANDS;

	CHECK(BtHciCapabilitiesLegacyScanningSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_SCAN_PARAMETERS);
	CHECK(BtHciCapabilitiesLegacyScanningSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_SCAN_ENABLE);
	CHECK(BtHciCapabilitiesLegacyScanningSupported(&caps));

	CHECK(BtHciCapabilitiesExtendedScanningSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_PARAMETERS);
	CHECK(BtHciCapabilitiesExtendedScanningSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_SET_EXT_SCAN_ENABLE);
	CHECK(BtHciCapabilitiesExtendedScanningSupported(&caps));

	CHECK(BtHciCapabilitiesLegacyInitiatingSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_CREATE_CONNECTION);
	CHECK(BtHciCapabilitiesLegacyInitiatingSupported(&caps));
	CHECK(BtHciCapabilitiesExtendedInitiatingSupported(&caps) == false);
	SetCommand(&caps, BT_HCI_CAP_CMD_LE_EXT_CREATE_CONNECTION);
	CHECK(BtHciCapabilitiesExtendedInitiatingSupported(&caps));
}

void TestDataLengthControl()
{
	BtHciDevice_t dev = {};
	dev.Command = CapabilityCommand;

	Reset(0);
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120));
	CHECK(s_DataLengthCount == 1);
	CHECK(s_DataLengthParamLen == 6);
	CHECK(s_DataLengthParam[0] == 0x42);
	CHECK(s_DataLengthParam[1] == 0x00);
	CHECK(s_DataLengthParam[2] == 0xFB);
	CHECK(s_DataLengthParam[3] == 0x00);
	CHECK(s_DataLengthParam[4] == 0x48);
	CHECK(s_DataLengthParam[5] == 0x08);

	Reset(0);
	s_DataLengthReturnHdl = 1;
	CHECK(BtHciSetDataLength(&dev, 1, 27, 328));
	CHECK(s_DataLengthCount == 1);
	CHECK(s_DataLengthParam[2] == 0x1B);
	CHECK(s_DataLengthParam[3] == 0x00);
	CHECK(s_DataLengthParam[4] == 0x48);
	CHECK(s_DataLengthParam[5] == 0x01);

	Reset(0);
	CHECK(BtHciSetDataLength(nullptr, 1, 27, 328) == false);
	CHECK(BtHciSetDataLength(&dev, BT_HCI_LE_CONN_HANDLE_MAX + 1U,
		27, 328) == false);
	CHECK(BtHciSetDataLength(&dev, 1, 26, 328) == false);
	CHECK(BtHciSetDataLength(&dev, 1, 252, 328) == false);
	CHECK(BtHciSetDataLength(&dev, 1, 27, 327) == false);
	CHECK(BtHciSetDataLength(&dev, 1, 27, 17041) == false);
	CHECK(s_OpcodeCount == 0);
	CHECK(s_DataLengthCount == 0);

	Reset(0);
	s_DleCommandSupported = false;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120) == false);
	CHECK(s_DataLengthCount == 0);

	Reset(0);
	s_DleFeatureSupported = false;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120) == false);
	CHECK(s_DataLengthCount == 0);

	Reset(0);
	s_MaxTxOctets = 100;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 101, 1000) == false);
	CHECK(s_DataLengthCount == 0);

	Reset(0);
	s_MaxTxTime = 1000;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 100, 1001) == false);
	CHECK(s_DataLengthCount == 0);

	Reset(0);
	s_MaxDataLengthSupported = false;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120));
	CHECK(s_DataLengthCount == 1);

	Reset(0);
	s_DataLengthStatus = 0x0C;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120) == false);
	CHECK(s_DataLengthCount == 1);

	Reset(0);
	s_DataLengthReturnHdl = 0x0043;
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120) == false);
	CHECK(s_DataLengthCount == 1);

	Reset(2);
	CHECK(BtHciSetDataLength(&dev, 0x0042, 251, 2120) == false);
	CHECK(s_DataLengthCount == 0);
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
	TestGapCommandChecks();
	TestDataLengthControl();

	if (s_Failures == 0)
	{
		std::printf("Bluetooth HCI capability tests: PASS (%d checks)\n", s_Checks);
		return 0;
	}

	std::printf("Bluetooth HCI capability tests: FAIL (%d failures, %d checks)\n",
		s_Failures, s_Checks);
	return 1;
}
