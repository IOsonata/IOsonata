/* LPC1769 EP0 FIFO regression. MIT License, I-SYST, 2026. */
#define main ClockTests
#include "hw17_test.c"
#undef main
#include <string.h>

static unsigned s_Realizations;
static uint32_t s_ExpectedSize = 64;
static uint32_t s_ExpectedPhysical;
static bool s_DataRealize;
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
static unsigned s_AckTimeout;
static uint32_t s_AckOrder;
static bool s_AckDistinct;
static bool s_AckArrival;
static bool s_InitTrace;
static unsigned s_InitWrites;
static uint16_t s_InitData[6];
static unsigned s_CodeCount;
static unsigned s_DropCode;
static bool s_BusTrace;
static bool s_BusArrival;
static bool s_ResetArrival;

void TestEndpointClear(uint32_t Value)
{
	assert(s_Mask == 1 && (Value == 1U || Value == 2U));
	assert((LPC_USB->USBDevIntSt & 0x20U) == 0U);
	++s_Acks;
	s_AckOrder = (s_AckOrder << 4) | Value;
	if (!s_CommandTimeout && s_Acks != s_AckTimeout)
	{
		SetReadOnly(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt & ~Value);
		SetReadOnly(&LPC_USB->USBCmdData, s_AckDistinct ?
			(Value == 1U ? 0x6DU : 0x12U) : s_Status);
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | 0x20U);
		if (s_AckArrival && Value == 2U)
			SetReadOnly(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt | 1U);
	}
}

void TestSieClear(uint32_t Value)
{
	assert(s_Mask == 1 && (Value & ~0x138U) == 0);
	SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt & ~Value);
}
void TestSieCode(uint32_t Value)
{
	assert(s_Mask == 1);
	const unsigned phase = Value & 0xFF00U;
	++s_CodeCount;
	if (s_CodeCount == s_DropCode)
		s_CommandTimeout = true;
	if (phase == 0x500U)
	{
		s_Command = (uint8_t)(Value >> 16);
		if (s_Command == 0xFA)
			++s_Validations;
		if (s_Command == 0xF2)
			++s_Clears;
		if (s_BusTrace)
			assert(s_Command == 0xFE && (LPC_USB->USBDevIntSt & 8U) == 0U);
	}
	if (s_InitTrace && phase == 0x100U)
	{
		assert(s_InitWrites < 6);
		s_InitData[s_InitWrites++] = ((uint16_t)s_Command << 8) | (uint8_t)(Value >> 16);
	}
	if (!s_CommandTimeout)
	{
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt |
			(phase == 0x200U ? 0x20U : 0x10U));
		SetReadOnly(&LPC_USB->USBCmdData, s_Status);
		if (s_BusArrival && phase == 0x200U)
			SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | 8U);
	}
}
void TestPacketSize(uint32_t Value)
{
	assert(Value == s_ExpectedSize && s_Mask == 1);
	if (s_DataRealize)
	{
		assert(LPC_USB->USBEpInd == s_ExpectedPhysical);
		assert((LPC_USB->USBReEp & (1UL << s_ExpectedPhysical)) != 0);
		assert((LPC_USB->USBDevIntSt & 0x100U) == 0);
	}
	++s_Realizations;
	if (s_ResetArrival)
		SetReadOnly(&LPC_USB->USBEpIntSt, LPC_USB->USBEpIntSt | 1U);
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
static void DeviceTests(void)
{
	LPC_USB->USBEpIntEn = 0;
	s_AckTimeout = 0;
	Prepare();
	LPC_USB->USBDMAIntEn = 1;
	const unsigned codes = s_CodeCount;
	assert(!Lpc17UsbDeviceInit() && s_CodeCount == codes);
	LPC_USB->USBDMAIntEn = 0;
	s_InitTrace = true;
	s_InitWrites = 0;
	s_Mask = 0;
	assert(Lpc17UsbDeviceInit() && s_Mask == 0);
	s_InitTrace = false;
	const uint16_t expected[] = {0xFE00, 0xF301, 0xD800, 0xD080, 0x4000, 0x4100};
	assert(s_InitWrites == 6 && memcmp(expected, s_InitData, sizeof(expected)) == 0);
	assert(LPC_USB->USBEpIntEn == 0 && LPC_USB->USBDevIntEn == 0);
	assert(LPC_USB->USBEpIntPri == 0 && LPC_USB->USBDevIntPri == 0);
	assert(!Lpc17UsbDeviceInit());
	uint8_t status = 0xCC;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0x200);
	assert(Lpc17UsbDeviceStatus(&status) == 0 && status == 0xCC);
	assert(Lpc17UsbDeviceStatus(0) == -1);
	s_BusTrace = true;
	s_BusArrival = true;
	s_Status = 0x1F;
	s_Mask = 1;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0x208);
	assert(Lpc17UsbDeviceStatus(&status) == 1 && status == 0x1F && s_Mask == 1);
	assert((LPC_USB->USBDevIntSt & 0x208) == 0x208); // new event survives read
	s_BusArrival = false;
	s_CommandTimeout = true;
	status = 0xCC;
	assert(Lpc17UsbDeviceStatus(&status) == -1 && status == 0xCC);
	assert(!Lpc17UsbEp0Write(0, 0));
	s_BusTrace = false;
	for (unsigned phase = 1; phase <= 12; ++phase)
	{
		Prepare();
		s_DropCode = s_CodeCount + phase;
		assert(!Lpc17UsbDeviceInit());
		assert(LPC_USB->USBEpIntEn == 0 && LPC_USB->USBDevIntEn == 0);
		assert(!Lpc17UsbEp0Write(0, 0));
	}
	s_DropCode = 0;
	for (unsigned ack = 1; ack <= 2; ++ack)
	{
		Prepare();
		s_AckTimeout = s_Acks + ack;
		assert(!Lpc17UsbDeviceInit() && !Lpc17UsbEp0Write(0, 0));
	}
	s_AckTimeout = 0;
	Prepare();
	assert(Lpc17UsbDeviceInit());
}

static void BusResetTests(void)
{
	s_Status = 2; // hardware leaves control endpoints stalled on bus reset
	for (unsigned mask = 0; mask < 2; ++mask)
	{
		LPC_USB->USBDevIntEn = 0x0C;
		LPC_USB->USBEpIntEn = 3;
		SetReadOnly(&LPC_USB->USBEpIntSt, 3);
		s_ResetArrival = true;
		s_Mask = mask;
		const unsigned realized = s_Realizations;
		assert(Lpc17UsbBusReset() && s_Mask == mask);
		assert(s_Realizations == realized + 2);
		assert(LPC_USB->USBEpIntSt == 1); // event during restoration survives
		assert(LPC_USB->USBDevIntEn == 0x0C && LPC_USB->USBEpIntEn == 3);
		s_ResetArrival = false;
	}
	for (unsigned phase = 1; phase <= 6; ++phase)
	{
		LPC_USB->USBDevIntEn = 0;
		LPC_USB->USBEpIntEn = 0;
		Prepare();
		assert(Lpc17UsbDeviceInit());
		LPC_USB->USBDevIntEn = 0x0C;
		LPC_USB->USBEpIntEn = 3;
		s_DropCode = s_CodeCount + phase;
		assert(!Lpc17UsbBusReset());
		assert(LPC_USB->USBDevIntEn == 0 && LPC_USB->USBEpIntEn == 0);
		assert(!Lpc17UsbEp0Write(0, 0));
	}
	s_DropCode = 0;
	Prepare();
	assert(Lpc17UsbDeviceInit());
	s_Status = 5; // delayed reset handling must not discard the first SETUP
	assert(!Lpc17UsbBusReset() && !Lpc17UsbEp0Write(0, 0));
	Prepare();
	assert(Lpc17UsbDeviceInit());
	s_RealizeTimeout = true;
	assert(!Lpc17UsbBusReset());
	s_RealizeTimeout = false;
	Prepare();
	assert(Lpc17UsbDeviceInit() && Lpc17UsbBusReset());
}

static void DataRealizeTests(void)
{
	LPC_USB->USBDevIntEn = 0;
	LPC_USB->USBEpIntEn = 0;
	Prepare();
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	assert(Lpc17UsbDeviceInit());
	s_DataRealize = true;
	// Enumerate logical endpoints, transfer types and packet-size boundaries.
	const uint16_t bulkMask = 0xC924;
	const uint16_t intMask = 0x2492;
	for (unsigned ep = 0; ep <= 16; ++ep)
	{
		for (unsigned type = 0; type <= 4; ++type)
		{
			for (unsigned size = 0; size <= 65; ++size)
			{
				for (unsigned in = 0; in <= 1; ++in)
				{
					bool expected = ep < 16 && size > 0 && size <= 64 &&
						((type == 3 && (intMask & (1U << ep)) != 0) ||
						(type == 2 && (bulkMask & (1U << ep)) != 0 &&
						(size == 8 || size == 16 || size == 32 || size == 64)));
					LPC_USB->USBReEp = 3;
					SetReadOnly(&LPC_USB->USBDevIntSt, 0x10C);
					s_ExpectedSize = size;
					s_ExpectedPhysical = ep * 2 + in;
					s_Mask = in;
					const unsigned count = s_Realizations;
					assert(Lpc17UsbDataRealize(ep, in != 0, type, size) == expected);
					assert(s_Mask == in && s_Realizations == count + expected);
					assert(LPC_USB->USBReEp == (expected ? 3U | (1UL << s_ExpectedPhysical) : 3U));
					assert(LPC_USB->USBDevIntSt == (expected ? 0xCU : 0x10CU));
				}
			}
		}
	}
	LPC_USB->USBReEp = 3;
	s_ExpectedSize = 64;
	// All supported directions fit together, including physical endpoint 31.
	for (unsigned ep = 1; ep < 16; ++ep)
	{
		unsigned type = (intMask & (1U << ep)) != 0 ? 3 : 2;
		if (((intMask | bulkMask) & (1U << ep)) == 0)
			continue;
		for (unsigned in = 0; in <= 1; ++in)
		{
			s_ExpectedPhysical = ep * 2 + in;
			assert(Lpc17UsbDataRealize(ep, in != 0, type, 64));
			const unsigned count = s_Realizations;
			assert(!Lpc17UsbDataRealize(ep, in != 0, type, 64));
			assert(s_Realizations == count);
		}
	}
	assert(LPC_USB->USBReEp == 0xFCF3CF3FU);
	LPC_USB->USBReEp = 3;
	const unsigned count = s_Realizations;
	LPC_USB->USBDevIntEn = 8;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBDevIntEn = 0;
	LPC_USB->USBEpIntEn = 3;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBEpIntEn = 0;
	SetReadOnly(&LPC_USB->USBEpDMASt, 4);
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	SetReadOnly(&LPC_USB->USBEpDMASt, 0);
	LPC_USB->USBDMAIntEn = 1;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBDMAIntEn = 0;
	LPC_USB->USBCtrl = 1;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBCtrl = 0;
	LPC_USB->USBReEp = 0x43; // ISO memory ownership is unsupported.
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBReEp = 1;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_USB->USBReEp = 3;
	LPC_SC->PCONP &= ~(1UL << 31);
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	LPC_SC->PCONP |= 1UL << 31;
	SetReadOnly(&LPC_USB->USBClkSt, 0);
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	assert(!Lpc17UsbDataRealize(255, false, 3, 64));
	assert(!Lpc17UsbDataRealize(1, false, 255, 64));
	assert(!Lpc17UsbDataRealize(1, false, 3, 65535));
	assert(s_Realizations == count && LPC_USB->USBReEp == 3);
	s_ExpectedPhysical = 2;
	s_RealizeTimeout = true;
	assert(!Lpc17UsbDataRealize(1, false, 3, 64));
	assert(LPC_USB->USBReEp == 7); // Unknown RAM state is retained on timeout.
	s_RealizeTimeout = false;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0x100);
	assert(!Lpc17UsbDataRealize(2, false, 2, 64));
	assert(!Lpc17UsbEp0Write(0, 0));
	s_DataRealize = false;
	Prepare();
	assert(Lpc17UsbDeviceInit());
	puts("LPC1769 bulk/interrupt realization, fixed types, sizes and timeout recovery: PASS");
}

int main(void)
{
	ClockTests();
	uint8_t collected[2] = {0xCC, 0xDD};
	assert(Lpc17UsbEp0Collect(collected) == -1);
	assert(collected[0] == 0xCC && collected[1] == 0xDD);
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
	assert(Lpc17UsbEp0Collect(0) == -1);
	s_AckDistinct = true;
	for (unsigned pending = 0; pending < 4; ++pending)
	{
		for (unsigned enabled = 0; enabled < 4; ++enabled)
		{
			const unsigned selected = pending & enabled;
			LPC_USB->USBEpIntEn = enabled | 0x80U;
			SetReadOnly(&LPC_USB->USBEpIntSt, pending | 0x80U);
			s_AckOrder = 0;
			s_Mask = pending & 1U;
			assert(Lpc17UsbEp0Collect(collected) == (int)selected);
			assert(collected[0] == ((selected & 1U) ? 0x6D : 0));
			assert(collected[1] == ((selected & 2U) ? 0x12 : 0));
			assert(s_AckOrder == (selected == 3U ? 0x21U : selected));
			assert(LPC_USB->USBEpIntSt == ((pending | 0x80U) & ~selected));
			assert(s_Mask == (pending & 1U));
		}
	}
	LPC_USB->USBEpIntEn = 3;
	SetReadOnly(&LPC_USB->USBEpIntSt, 2);
	s_AckArrival = true;
	assert(Lpc17UsbEp0Collect(collected) == 2 && LPC_USB->USBEpIntSt == 1);
	s_AckArrival = false;
	assert(Lpc17UsbEp0Collect(collected) == 1 && collected[0] == 0x6D);
	s_AckDistinct = false;
	LPC_USB->USBEpIntEn = 0;
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
	for (unsigned failure = 1; failure <= 2; ++failure)
	{
		LPC_USB->USBEpIntEn = 0;
		Prepare();
		assert(Lpc17UsbEp0Realize());
		LPC_USB->USBEpIntEn = 3;
		SetReadOnly(&LPC_USB->USBEpIntSt, 3);
		s_AckTimeout = s_Acks + failure;
		collected[0] = 0xCC;
		collected[1] = 0xDD;
		s_Mask = failure - 1;
		assert(Lpc17UsbEp0Collect(collected) == -1);
		assert(collected[0] == 0xCC && collected[1] == 0xDD);
		assert(s_Acks == s_AckTimeout && s_Mask == failure - 1);
		assert(Lpc17UsbEp0Collect(collected) == -1 && s_Acks == s_AckTimeout);
	}
	DeviceTests();
	BusResetTests();
	DataRealizeTests();
	puts("LPC1769 device init, bus status, EP0 FIFO, interrupts and timeouts: PASS");
}
