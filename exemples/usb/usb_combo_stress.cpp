/**-------------------------------------------------------------------------
@example	usb_combo_stress.cpp

@brief	USB composite stress test: dual CDC + HID + interrupt + ISO loopback.

Runs all USB data paths on one controller at the same time. Two CDC functions
retain the UsbDualCdcStress loopback/PRBS behavior. HID, raw interrupt and
isochronous functions each provide a bidirectional loopback. Interface and
endpoint numbers are assigned by the common allocator; ISO reserves the
controller-supported ISO endpoint.

Host runner: Python/usb_combo_stress.py

@author	Hoang Nguyen Hoan
@date	Sep. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/

#include "usb_combo_stress_device.h"

int main()
{
	// Same loopback and PRBS logic as exemples/usb/tinyusb_combo_stress, so
	// both builds do the same work per byte.
	uint8_t loopbackBuffer[CDC_BUFFER_SIZE];
	uint8_t loopbackExpected = Prbs8(0xff);
	uint8_t prbs = 0xff;
	uint32_t loopbackRxErrorNotify = 0;
	int loopbackPending = 0;
	int loopbackOffset = 0;
	bool loopbackConnected = false;

	if (!UsbInit(&s_UsbCfg) ||
		!g_LoopbackCdc.Init(s_LoopbackCfg) ||
		!g_PrbsCdc.Init(s_PrbsCfg) ||
		!g_Hid.Init(s_HidCfg) ||
		!IntInit() ||
		!IsoInit())
	{
		return -1;
	}

	(void)UsbEnable(USB_DEVNO);
	while (1)
	{
		UsbProcess(USB_DEVNO);

		// A port open restarts the host generator: restart the checker and
		// drop any echo still pending from the previous session.
		const bool connected = g_LoopbackCdc.IsPortOpen();
		if (connected != loopbackConnected)
		{
			loopbackConnected = connected;
			loopbackPending = 0;
			loopbackOffset = 0;

			if (connected)
			{
				static const char msg[] = "\r\nIOsonata USB Combo Stress\r\n";
				loopbackExpected = Prbs8(0xff);
				g_LoopbackCdc.Tx(0, reinterpret_cast<const uint8_t *>(msg),
					(int)sizeof(msg) - 1);
			}
		}

		if (loopbackConnected)
		{
			if (loopbackPending > 0)
			{
				const int length = g_LoopbackCdc.Tx(
					0, &loopbackBuffer[loopbackOffset], loopbackPending);
				if (length > 0)
				{
					loopbackOffset += length;
					loopbackPending -= length;
				}
			}
			else
			{
				const int length = g_LoopbackCdc.Rx(
					0, loopbackBuffer, sizeof(loopbackBuffer));
				if (length > 0)
				{
					for (int i = 0; i < length; i++)
					{
						if (loopbackBuffer[i] != loopbackExpected)
						{
							loopbackRxErrorNotify++;
						}
						loopbackExpected = Prbs8(loopbackBuffer[i]);
					}
					loopbackPending = length;
					loopbackOffset = 0;
				}
			}
		}

		const uint8_t prbsByte = loopbackRxErrorNotify > 0U ? 0U : prbs;
		if (g_PrbsCdc.IsPortOpen() && g_PrbsCdc.Tx(0, &prbsByte, 1) > 0)
		{
			if (loopbackRxErrorNotify > 0U)
			{
				loopbackRxErrorNotify--;
			}
			else
			{
				prbs = Prbs8(prbs);
			}
		}
	}

	return 0;
}


