// Reuse the core test controller and request helpers in a separate executable,
// so this strong descriptor override does not replace the default-builder tests.
#define main UsbDefaultDescriptorTests
#include "usb_core_test.cpp"
#undef main

static uint8_t s_ConfigBuffer[25];
static unsigned s_DeviceReads;
static unsigned s_ConfigReads[2];
static bool s_Malformed;

const uint8_t *UsbGetDescriptor(int, uint8_t Type, uint8_t Index,
	uint16_t, UsbSpeed_t, uint16_t *pLength)
{
	static const uint8_t device[] = {
		18, USB_DESCTYPE_DEVICE, 0, 2, 0, 0, 0, 64,
		9, 18, 1, 0, 0, 1, 0, 0, 0, 2
	};
	*pLength = 0;
	if (Type == USB_DESCTYPE_DEVICE && Index == 0)
	{
		s_DeviceReads++;
		*pLength = sizeof(device);
		return device;
	}
	if (Type != USB_DESCTYPE_CONFIGURATION || Index >= 2)
		return nullptr;
	s_ConfigReads[Index]++;
	// Shared provider buffer deliberately overwritten on every fetch.
	const uint8_t config[] = {
		9, USB_DESCTYPE_CONFIGURATION, 25, 0, 1,
		(uint8_t)(Index == 0 ? 7 : 42), 0,
		(uint8_t)(USB_CONFATT_RESERVED | (Index == 0 ? USB_CONFATT_SELF_POWERED : 0)), 50,
		9, USB_DESCTYPE_INTERFACE, 0, 0, 1, USB_INTRFCLASS_VENDOR, 0, 0, 0,
		7, USB_DESCTYPE_ENDPOINT, (uint8_t)(Index == 0 ? EP1_IN : EP2_IN), 2, 64, 0, 0
	};
	memcpy(s_ConfigBuffer, config, sizeof(config));
	if (s_Malformed && Index == 1)
		s_ConfigBuffer[0] = 0;
	*pLength = sizeof(config);
	return s_ConfigBuffer;
}

static bool CheckActive(uint8_t value, uint8_t endpoint, bool selfpowered)
{
	CHECK(UsbGetConfiguration(TEST_DEVNO) == value);
	s_DeviceReads = s_ConfigReads[0] = s_ConfigReads[1] = 0;
	ClearCtrlrLog();
	Setup(STD_DEV_IN, USB_REQ_GET_STATUS, 0, 0, 2);
	CHECK(LastXfer() && LastXfer()->Data[0] == (selfpowered ? 1 : 0));
	Complete(EP0_IN, 2);
	Complete(EP0_OUT, 0);
	if (value != 0)
	{
		CHECK(UsbEpSetHalt(TEST_DEVNO, endpoint, true, true));
		CHECK(!UsbEpSetHalt(TEST_DEVNO, endpoint == 1 ? 2 : 1, true, true));
	}
	CHECK(s_DeviceReads == 0);
	CHECK(s_ConfigReads[selfpowered ? 1 : 0] == 0);
	CHECK(s_ConfigReads[selfpowered ? 0 : 1] != 0);
	return true;
}

static bool TestMultipleConfigurations(void)
{
	TestUsbDeviceClass device;
	CHECK(Fixture(true, &device));
	CHECK(SetAddress(3));
	CHECK(SetConfig(42));
	CHECK(CheckActive(42, 2, false));
	// Fetching another descriptor must not change the active configuration.
	uint16_t len;
	CHECK(UsbGetDescriptor(0, USB_DESCTYPE_CONFIGURATION, 0, 0, USB_SPEED_FULL, &len));
	CHECK(CheckActive(42, 2, false));
	const int stalls = s_Ctrlr.StallCnt;
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, 9, 0, 0);
	CHECK(s_Ctrlr.StallCnt == stalls + 1);
	CHECK(CheckActive(42, 2, false));
	CHECK(SetConfig(7));
	CHECK(CheckActive(7, 1, true));
	s_Malformed = true;
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, 42, 0, 0);
	CHECK(UsbGetConfiguration(0) == 7);
	s_Malformed = false;
	CHECK(CheckActive(7, 1, true));
	CHECK(SetConfig(42));
	device.RejectConfig = true;
	Setup(STD_DEV_OUT, USB_REQ_SET_CONFIGURATION, 7, 0, 0);
	CHECK(CheckActive(0, 1, true));
	device.RejectConfig = false;
	CHECK(SetConfig(42));
	CHECK(SetConfig(0));
	CHECK(CheckActive(0, 1, true));
	CHECK(SetConfig(42));
	Event(USB_CTRLR_EVT_RESET);
	CHECK(CheckActive(0, 1, true));
	return true;
}

int main(void)
{
	if (!TestMultipleConfigurations())
		return 1;
	puts("PASS: multiple configurations, shared descriptor buffer, rollback, unconfigure and reset");
	return 0;
}
