/*---------------------------------------------------------------------------
File : header_test.c
Desc : LPC11Uxx USB capability and generic-header compatibility checks.
Copyright (c) 2026 I-SYST inc. MIT License.
---------------------------------------------------------------------------*/
#include <assert.h>
#include "usb/usb.h"

int main(void)
{
	assert(USB_CTRLR_CNT == 1);
	assert(USB_EPIN_CNT(0) == 5);
	assert(USB_EPOUT_CNT(0) == 5);
	assert(!USB_HIGHSPEED_CAPABLE(0));
	assert(!USB_ISO_SUPPORTED(0));
	assert(USB_ISO_EPIN_MASK(0) == 0);
	assert(USB_ISO_EPOUT_MASK(0) == 0);
	assert(USB_CTRLR_PKT_LEN_MAX(0, CONTROL) == 64);
	assert(USB_CTRLR_PKT_LEN_MAX(0, BULK) == 64);
	assert(USB_CTRLR_PKT_LEN_MAX(0, INT) == 64);
	assert(USB_CTRLR_PKT_LEN_MAX(0, ISO) == 1023);
	assert(USB_CTRLR_PKT_LEN_MAX(0, 99) == 0);
	for (int dev = -1; dev <= 1; dev += 2)
	{
		assert(USB_EPIN_CNT(dev) == 0);
		assert(USB_EPOUT_CNT(dev) == 0);
		assert(!USB_HIGHSPEED_CAPABLE(dev));
		assert(!USB_ISO_SUPPORTED(dev));
		assert(USB_ISO_EPIN_MASK(dev) == 0);
		assert(USB_ISO_EPOUT_MASK(dev) == 0);
		assert(USB_CTRLR_PKT_LEN_MAX(dev, CONTROL) == 0);
		assert(USB_CTRLR_PKT_LEN_MAX(dev, BULK) == 0);
		assert(USB_CTRLR_PKT_LEN_MAX(dev, INT) == 0);
		assert(USB_CTRLR_PKT_LEN_MAX(dev, ISO) == 0);
	}
	return 0;
}
