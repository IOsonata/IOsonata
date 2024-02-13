/**-------------------------------------------------------------------------
@file	bt_scan.cpp

@brief	Generic Bluetooth scan

Generic implementation of bt device scanning

@author	Hoang Nguyen Hoan
@date	Feb. 06, 2024

@license

MIT License

Copyright (c) 2024, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "bluetooth/bt_scan.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_hcievt.h"

#include "coredev/uart.h"
extern UART g_Uart;

void BtParseAdvReport(BtAdvReport_t *pReport)
{

}

void BtParseExtAdvReport(BtExtAdvReport_t *pReport)
{

}

void BtScanReport(uint8_t Type, uint8_t NbReport, void *pReport)
{
	switch (Type)
	{
		case BT_HCI_EVT_LE_ADV_REPORT:
			{
				BtAdvReport_t *p = (BtAdvReport_t*)pReport;
				g_Uart.printf("BT_HCI_EVT_LE_ADV_REPORT\r\n");
				g_Uart.printf("BT_HCI_EVT_LE_ADV_REPORT: %d\r\n", NbReport);

				for (int i = 0; i < NbReport; i++)
				{
					g_Uart.printf("%02x %02x %02x %02x %02x %02x, Datalen = %d\r\n",
								p[i].Addr[0], p[i].Addr[1], p[i].Addr[2],
								p[i].Addr[3], p[i].Addr[4], p[i].Addr[5], p[i].DataLen);
					//int8_t *rssi = p[i].DataLen + p[i].Data;

				}

			}
			break;
		case BT_HCI_EVT_LE_EXT_ADV_REPORT:
			{
				BtExtAdvReport_t *p = (BtExtAdvReport_t*)pReport;
				g_Uart.printf("BT_HCI_EVT_LE_EXT_ADV_REPORT\r\n");
				g_Uart.printf("Nb reports : %d\r\n", NbReport);
				for (int i = 0; i < NbReport; i++)
				{
					g_Uart.printf("%02x %02x %02x %02x %02x %02x, Datlen = %d\r\n",
								p[i].Addr[0], p[i].Addr[1], p[i].Addr[2],
								p[i].Addr[3], p[i].Addr[4], p[i].Addr[5],
								p[i].DataLen);

				}
			}
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1:
			g_Uart.printf("BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1\r\n");
			break;
		case BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V2:
			g_Uart.printf("BT_HCI_EVT_LE_PERIODIC_ADV_REPORT_V1\r\n");
			break;
	}
}
