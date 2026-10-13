/* LPC1769 EP0 FIFO regression. MIT License, I-SYST, 2026. */
#define main ClockTests
#include "hw17_test.c"
#undef main
#include <string.h>

static unsigned s_Realizations;
static unsigned s_Reads;
static unsigned s_Writes;
static unsigned s_Validations;
static unsigned s_Clears;
static uint32_t s_TxWords[16];
static uint8_t s_Status;
static uint8_t s_Command;
static bool s_RealizeTimeout;
static bool s_CommandTimeout;
static unsigned s_Acks;

void TestEndpointClear(uint32_t Value)
{
	assert(s_Mask == 1 && (Value == 1U || Value == 2U));
	assert((LPC_USB->USBDevIntSt & 0x20U) == 0U);
	++s_Acks;
	if (!s_CommandTimeout)
	{
		SetReadOnly(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt & ~Value);
		SetReadOnly(&LPC_USB->USBCmdData, s_Status);
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | 0x20U);
	}
}

void TestSieClear(uint32_t Value)
{
	assert(s_Mask == 1 && (Value & ~0x130U) == 0);
	SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt & ~Value);
}
void TestSieCode(uint32_t Value)
{
	assert(s_Mask == 1);
	const unsigned phase = Value & 0xFF00U;
	if (phase == 0x500U)
	{
		s_Command = (uint8_t)(Value >> 16);
		if (s_Command == 0xFA)
			++s_Validations;
		if (s_Command == 0xF2)
			++s_Clears;
	}
	if (!s_CommandTimeout)
	{
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt |
			(phase == 0x200U ? 0x20U : 0x10U));
		SetReadOnly(&LPC_USB->USBCmdData, s_Status);
	}
}
void TestPacketSize(uint32_t Value)
{
	assert(Value == 64 && s_Mask == 1);
	++s_Realizations;
	if (!s_RealizeTimeout)
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | 0x100U);
}
void TestPacketWrite(uint32_t Value)
{
	assert(s_Writes < 16 && LPC_USB->USBCtrl == 2 && s_Mask == 1);
	s_TxWords[s_Writes++] = Value;
}
uint32_t TestPacketRead(void)
{
	assert(s_Reads < 16 && LPC_USB->USBCtrl == 1 && s_Mask == 1);
	uint32_t word = 0;
	for (unsigned i = 0; i < 4; ++i)
		word |= (s_Reads * 4 + i) << (8 * i);
	++s_Reads;
	return word;
}
static void Prepare(void)
{
	LPC_USB->USBReEp = 0; // model caller's endpoint retirement
	assert(Lpc17UsbHwRelease());
	LPC_SC->SCS = 0x60;
	SetReadOnly(&LPC_SC->PLL1STAT, (7U << 8) | (1U << 5) | 3U);
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	assert(Lpc17UsbHwPrepare(12000000));
	LPC_USB->USBReEp = 3; // silicon reset realizes both control endpoints
	s_CommandTimeout = false;
	s_Status = 0;
}
int main(void)
{
	ClockTests();
	uint8_t status = 0xCC;
	assert(!Lpc17UsbEp0Ack(false, &status) && status == 0xCC && s_Acks == 0);
	assert(!Lpc17UsbEp0Realize());
	assert(!Lpc17UsbEp0Write(0, 0));
	Prepare();
	assert(!Lpc17UsbEp0Ack(false, &status) && s_Acks == 0);
	assert(Lpc17UsbEp0Realize() && s_Realizations == 2);
	assert(!Lpc17UsbEp0Ack(false, 0) && s_Acks == 0);
	for (unsigned in = 0; in < 2; ++in)
	{
		s_Mask = in;
		s_Status = in ? 0x12 : 0x6D;
		SetReadOnly(&LPC_USB->USBEpIntSt, 0x83);
		SetReadOnly(&LPC_USB->USBDevIntSt, 0x3F);
		SetReadOnly(&LPC_USB->USBCmdData, 0xCC);
		assert(Lpc17UsbEp0Ack(in != 0, &status) && status == s_Status);
		assert(LPC_USB->USBEpIntSt == (0x83U & ~(1U << in)));
		assert(LPC_USB->USBDevIntSt == 0x3F && s_Mask == in);
	}
	SetReadOnly(&LPC_USB->USBEpIntSt, 0);
	assert(Lpc17UsbEp0Ack(false, &status)); // overwrite recovery without pending IRQ
	const unsigned acks = s_Acks;
	LPC_SC->PCONP &= ~(1UL << 31);
	assert(!Lpc17UsbEp0Ack(false, &status) && s_Acks == acks);
	LPC_SC->PCONP |= 1UL << 31;
	SetReadOnly(&LPC_USB->USBClkSt, 0);
	assert(!Lpc17UsbEp0Ack(false, &status) && s_Acks == acks);
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	s_Status = 0;
	assert(!Lpc17UsbEp0Realize());
	uint8_t data[66];
	uint8_t source[66];
	for (unsigned i = 0; i < sizeof(source); ++i)
		source[i] = (uint8_t)i;
	for (int length = 0; length <= 64; ++length)
	{
		memset(data, 0xCC, sizeof(data));
		s_Reads = 0;
		SetReadOnly(&LPC_USB->USBRxPLen, 0x800U | (unsigned)length);
		s_Mask = 0;
		assert(Lpc17UsbEp0Read(data + 1, length) == length);
		assert(s_Reads == (unsigned)(length + 3) / 4 && s_Mask == 0);
		assert(data[0] == 0xCC && data[length + 1] == 0xCC);
		for (int i = 0; i < length; ++i)
			assert(data[i + 1] == i);
		s_Writes = 0;
		assert(Lpc17UsbEp0Write(source + 1, length));
		assert(s_Writes == (unsigned)(length + 3) / 4 && LPC_USB->USBCtrl == 0);
		for (int i = 0; i < length; ++i)
			assert((uint8_t)(s_TxWords[i / 4] >> (8 * (i % 4))) == source[i + 1]);
	}
	assert(s_Validations == 65 && s_Clears == 65);
	assert(!Lpc17UsbEp0Write(source, 65) && !Lpc17UsbEp0Write(0, 1));
	assert(!Lpc17UsbEp0Write(source, -1));
	SetReadOnly(&LPC_USB->USBRxPLen, 0x800);
	assert(Lpc17UsbEp0Read(0, 0) == 0 && Lpc17UsbEp0Write(0, 0));
	s_Status = 1; // IN full; OUT overwritten
	s_Writes = 0;
	assert(!Lpc17UsbEp0Write(source, 1) && s_Writes == 0);
	SetReadOnly(&LPC_USB->USBRxPLen, 0x808);
	memset(data, 0xCC, sizeof(data));
	s_Reads = 0;
	assert(Lpc17UsbEp0Read(data, 8) == -2 && data[0] == 0xCC);
	s_Status = 0;
	s_Reads = 0;
	const unsigned clears = s_Clears;
	assert(Lpc17UsbEp0Read(data, 7) == -1 && s_Clears == clears && s_Reads == 0);
	assert(LPC_USB->USBCtrl == 0 && data[0] == 0xCC);
	SetReadOnly(&LPC_USB->USBRxPLen, 0);
	assert(Lpc17UsbEp0Read(data, 64) == -1 && LPC_USB->USBCtrl == 0);
	assert(!Lpc17UsbEp0Write(source, 1));
	Prepare();
	s_RealizeTimeout = true;
	assert(!Lpc17UsbEp0Realize() && !Lpc17UsbEp0Write(source, 1));
	s_RealizeTimeout = false;
	Prepare();
	assert(Lpc17UsbEp0Realize());
	s_CommandTimeout = true;
	SetReadOnly(&LPC_USB->USBRxPLen, 0x808);
	s_Reads = 0;
	assert(Lpc17UsbEp0Read(data, 8) == -1 && data[0] == 0xCC);
	assert(LPC_USB->USBCtrl == 0);
	Prepare();
	assert(Lpc17UsbEp0Realize());
	s_CommandTimeout = true;
	s_Mask = 0;
	status = 0xCC;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0x3F);
	assert(!Lpc17UsbEp0Ack(false, &status) && status == 0xCC && s_Mask == 0);
	const unsigned timedout = s_Acks;
	s_CommandTimeout = false;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0x3F); // late completion cannot revive SIE
	assert(!Lpc17UsbEp0Ack(true, &status) && status == 0xCC && s_Acks == timedout);
	assert(!Lpc17UsbSieCommand(0));
	Prepare();
	assert(Lpc17UsbEp0Realize() && Lpc17UsbEp0Ack(false, &status));
	puts("LPC1769 EP0 realization, FIFO, interrupt acknowledgement and timeouts: PASS");
}
