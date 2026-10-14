/* LPC1769 CPU transfer-state regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usb_data_lpc17xx.h"

static Lpc17UsbDataEndpoint_t s_Ep;
static uint32_t s_Mask;
static bool s_Ready = true;
static bool s_WriteFail;
static bool s_AutoReceive;
static bool s_StopOnReady;
static bool s_Rearm;
static unsigned s_Faults;
static unsigned s_Writes;
static unsigned s_Kicks;
static unsigned s_Completions;
static unsigned s_Cancels;
static unsigned s_Requests;
static uint16_t s_CompleteLength;
static int s_ReadLength;
static uint8_t s_Buffer[200];
static uint8_t s_Output[64];
static uint8_t s_Sent[200];
static unsigned s_SentLength;
static unsigned s_LastWrite;

uint32_t TestGetPrimask(void)
{
	return s_Mask;
}
void TestDisableIRQ(void)
{
	s_Mask = 1;
}
void TestSetPrimask(uint32_t State)
{
	s_Mask = State;
}
static bool Ready(void)
{
	return s_Ready;
}
static void Fault(void)
{
	assert(s_Mask == 1);
	++s_Faults;
	s_Ready = false;
	// Model the owner stopping hardware before returning the buffer.
	Lpc17UsbDataForget(&s_Ep);
}
static void Kick(uint8_t EpNo)
{
	assert(s_Mask == 1 && EpNo == s_Ep.EpNo);
	++s_Kicks;
}
bool Lpc17UsbDataWrite(uint8_t EpNo, const uint8_t *pData, int Length)
{
	assert(s_Mask == 1 && EpNo == s_Ep.EpNo && Length <= 64);
	++s_Writes;
	s_LastWrite = Length;
	if (s_WriteFail)
		return false;
	if (Length != 0)
		memcpy(s_Sent + s_SentLength, pData, Length);
	s_SentLength += Length;
	return true;
}
int Lpc17UsbDataRead(uint8_t EpNo, uint8_t *pData, int Capacity)
{
	assert(s_Mask == 1 && EpNo == s_Ep.EpNo && Capacity == s_Ep.Mps);
	if (s_ReadLength > Capacity)
		return -1;
	if (s_ReadLength > 0)
		memset(pData, 0xA5, s_ReadLength);
	return s_ReadLength;
}
static void Event(UsbCtrlrEvtType_t Type, uint16_t Length, void *pContext)
{
	assert(pContext == &s_Ep && s_Mask == 1);
	if (Type == USB_CTRLR_EVT_DRDY)
	{
		++s_Requests;
		if (s_StopOnReady)
		{
			s_Ready = false;
			Lpc17UsbDataForget(&s_Ep);
		}
		else if (s_AutoReceive)
		{
			unsigned completed = s_Completions;
			assert(Lpc17UsbDataReceive(&s_Ep, s_Output, sizeof(s_Output)));
			assert(s_Completions == completed); // submission never completes inline
		}
	}
	else if (Type == USB_CTRLR_EVT_XFER_CMPL)
	{
		assert(!s_Ep.Busy && s_Ep.pBuffer == NULL);
		++s_Completions;
		s_CompleteLength = Length;
		if (s_Rearm)
		{
			s_Rearm = false;
			assert(Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
		}
	}
	else
	{
		assert(Type == USB_CTRLR_EVT_CANCEL && !s_Ep.Enabled && !s_Ep.Busy);
		++s_Cancels;
		assert(!Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
	}
}
static void OpenEndpoint(uint8_t EpNo, bool In, uint16_t Mps)
{
	memset(&s_Ep, 0, sizeof(s_Ep));
	s_Ep.Handler = Event;
	s_Ep.pContext = &s_Ep;
	s_Ep.Ready = Ready;
	s_Ep.Fault = Fault;
	s_Ep.Kick = Kick;
	s_Ready = true;
	s_WriteFail = false;
	s_AutoReceive = false;
	s_StopOnReady = false;
	s_Rearm = false;
	s_Writes = 0;
	s_SentLength = 0;
	s_Completions = 0;
	s_Cancels = 0;
	s_Kicks = 0;
	s_Requests = 0;
	assert(!Lpc17UsbDataOpen(&s_Ep, 0, In, 64));
	assert(!Lpc17UsbDataOpen(&s_Ep, 3, In, 64));
	assert(!Lpc17UsbDataOpen(&s_Ep, 2, In, 63));
	assert(Lpc17UsbDataOpen(&s_Ep, EpNo, In, Mps));
	assert(!Lpc17UsbDataOpen(&s_Ep, EpNo, In, Mps));
}
static void Open(bool In)
{
	OpenEndpoint(2, In, 64);
}

static void ReceiveBoundsTests(void)
{
	uint8_t output[66];
	for (unsigned ep = 1; ep < 16; ++ep)
	{
		if (ep % 3 == 0 && ep != 15)
			continue;
		for (unsigned mps = 1; mps <= 64; ++mps)
		{
			if (ep % 3 != 1 && mps != 8 && mps != 16 && mps != 32 && mps != 64)
				continue;
			for (unsigned mask = 0; mask < 2; ++mask)
			{
				OpenEndpoint(ep, false, mps);
				s_Mask = mask;
				assert(!Lpc17UsbDataReceive(&s_Ep, output + 1, mps - 1));
				assert(!s_Ep.Busy && s_Kicks == 0 && s_Mask == mask);
				for (unsigned length = 0; length <= mps; ++length)
				{
					memset(output, 0xCC, sizeof(output));
					s_ReadLength = length;
					assert(Lpc17UsbDataReceive(&s_Ep, output + 1, mps));
					Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
					assert(s_Completions == length + 1 && s_CompleteLength == length);
					assert(!s_Ep.Busy && s_Mask == mask);
					for (unsigned i = 0; i < sizeof(output); ++i)
						assert(output[i] == (i > 0 && i <= length ? 0xA5 : 0xCC));
				}
				// Even a larger caller buffer must not relax the logical MPS.
				memset(output, 0xCC, sizeof(output));
				s_ReadLength = mps + 1;
				unsigned faults = s_Faults;
				assert(Lpc17UsbDataReceive(&s_Ep, output + 1, 64));
				Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
				assert(s_Faults == faults + 1 && s_Cancels == 1);
				assert(s_Completions == mps + 1 && !s_Ep.Busy && !s_Ep.Enabled);
				assert(s_Mask == mask);
				for (unsigned i = 0; i < sizeof(output); ++i)
					assert(output[i] == 0xCC);
			}
		}
	}
	puts("LPC1769 all bulk/interrupt OUT endpoints, logical MPS and buffer bounds: PASS");
}

int main(void)
{
	for (unsigned i = 0; i < sizeof(s_Buffer); ++i)
		s_Buffer[i] = i;
	const unsigned lengths[] = {0, 1, 63, 64, 65, 128, 129};
	for (unsigned mask = 0; mask < 2; ++mask)
	{
		for (unsigned n = 0; n < sizeof(lengths) / sizeof(lengths[0]); ++n)
		{
			Open(true);
			s_Mask = mask;
			unsigned length = lengths[n];
			assert(Lpc17UsbDataSend(&s_Ep, length ? s_Buffer : NULL, length));
			assert(!Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
			uint32_t generation = s_Ep.Generation;
			Lpc17UsbDataDispatch(&s_Ep, generation - 1, 0);
			Lpc17UsbDataDispatch(&s_Ep, generation, 0x10); // NAK
			assert(s_Completions == 0 && s_Writes == 1);
			unsigned packets = length ? (length + 63) / 64 : 1;
			for (unsigned packet = 0; packet < packets; ++packet)
			{
				Lpc17UsbDataDispatch(&s_Ep, generation, 0);
				assert(s_Completions == (packet + 1 == packets));
			}
			assert(s_CompleteLength == length && s_Writes == packets);
			assert(s_SentLength == length && memcmp(s_Sent, s_Buffer, length) == 0);
			assert(s_Mask == mask);
		}
	}
	Open(true);
	assert(!Lpc17UsbDataSend(&s_Ep, NULL, 1));
	assert(!Lpc17UsbDataReceive(&s_Ep, s_Output, 64));
	assert(Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
	uint32_t old = s_Ep.Generation;
	s_Rearm = true;
	Lpc17UsbDataDispatch(&s_Ep, old, 0);
	assert(s_Ep.Busy && s_Ep.Generation != old);
	Lpc17UsbDataDispatch(&s_Ep, old, 0);
	assert(s_Completions == 1);
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 0);
	assert(s_Completions == 2);
	Open(true);
	assert(Lpc17UsbDataSend(&s_Ep, s_Buffer, 129));
	s_WriteFail = true;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 0);
	assert(s_Cancels == 1 && s_Completions == 0 && !s_Ep.Enabled);
	Open(true);
	s_WriteFail = true;
	assert(!Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
	assert(s_Cancels == 0 && !s_Ep.Enabled);
	Open(false);
	assert(!Lpc17UsbDataSend(&s_Ep, s_Buffer, 1));
	assert(!Lpc17UsbDataReceive(&s_Ep, s_Output, 63));
	assert(!Lpc17UsbDataReceive(&s_Ep, NULL, 64));
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(s_Requests == 1 && s_Completions == 0 && !s_Ep.Busy);
	s_AutoReceive = true;
	s_ReadLength = 7;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(s_Completions == 1 && s_CompleteLength == 7 && s_Output[6] == 0xA5);
	assert(s_Kicks == 2); // submission and possible second buffered packet
	s_ReadLength = 0;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(s_Completions == 2 && s_CompleteLength == 0);
	assert(Lpc17UsbDataReceive(&s_Ep, s_Output, 64));
	assert(!Lpc17UsbDataReceive(&s_Ep, s_Output, 64));
	s_ReadLength = -2;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(s_Ep.Busy && s_Completions == 2);
	s_ReadLength = -1;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(s_Cancels == 1 && !s_Ep.Enabled);
	Open(false);
	s_StopOnReady = true;
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 1);
	assert(!s_Ep.Enabled && s_Completions == 0);
	Open(false);
	assert(Lpc17UsbDataReceive(&s_Ep, s_Output, 64));
	Lpc17UsbDataDispatch(&s_Ep, s_Ep.Generation, 2);
	assert(s_Cancels == 1 && !s_Ep.Enabled);
	Lpc17UsbDataForget(&s_Ep);
	assert(s_Cancels == 1);
	assert(s_Faults == 4);
	ReceiveBoundsTests();
	puts("LPC1769 data transfer segmentation, callbacks, reentrancy and retirement: PASS");
}
