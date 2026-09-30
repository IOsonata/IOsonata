// Compile with the USB hostport include path, -ffunction-sections,
// -fdata-sections and -Wl,--gc-sections. Include production code so this
// test can exercise the private validator without exposing a public API.
#include "../../src/usb/usb.cpp"
#include <assert.h>

int main(void)
{
	static uint8_t desc[65535];
	desc[0] = 9U;
	desc[1] = USB_DESCTYPE_CONFIGURATION;
	desc[2] = 255U;
	desc[3] = 255U;
	unsigned offset = 9U;
	while (sizeof(desc) - offset > 255U)
	{
	 desc[offset] = 255U;
	 desc[offset + 1U] = 0xFFU;
	 offset += 255U;
	}
	desc[offset] = (uint8_t)(sizeof(desc) - offset);
	desc[offset + 1U] = 0xFFU;
	uint16_t length = 0U;
	assert(UsbCoreValidateConfigDescriptor(desc, sizeof(desc),
	 USB_DESCTYPE_CONFIGURATION, &length));
	assert(length == sizeof(desc));
	// Cross 65535: the old 16-bit addition wrapped and revisited earlier bytes.
	desc[offset] = (uint8_t)(sizeof(desc) - offset + 1U);
	assert(!UsbCoreValidateConfigDescriptor(desc, sizeof(desc),
	 USB_DESCTYPE_CONFIGURATION, &length));
	// Leave one byte, which cannot contain a descriptor header.
	desc[offset] = (uint8_t)(sizeof(desc) - offset - 1U);
	assert(!UsbCoreValidateConfigDescriptor(desc, sizeof(desc),
	 USB_DESCTYPE_CONFIGURATION, &length));
}
