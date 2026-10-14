/* LPC1769 SIE command regression. MIT License, I-SYST, 2026. */
#define main ClockTests
#include "hw17_test.c"
#undef main

static uint32_t s_Codes[8];
static unsigned s_Count;
static uint32_t s_DropPhase;

void TestSieClear(uint32_t Value)
{
	assert(s_Mask == 1);
	assert((Value & ~0x30U) == 0);
	SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt & ~Value);
}
void TestSieCode(uint32_t Value)
{
	assert(s_Mask == 1 && s_Count < 8);
	s_Codes[s_Count++] = Value;
	const uint32_t phase = Value & 0xFF00U;
	const uint32_t flag = phase == 0x200U ? 0x20U : 0x10U;
	assert((LPC_USB->USBDevIntSt & flag) == 0);
	if (phase != s_DropPhase)
	{
		SetReadOnly(&LPC_USB->USBDevIntSt, LPC_USB->USBDevIntSt | flag);
		if (phase == 0x200U)
			SetReadOnly(&LPC_USB->USBCmdData, 0xA5);
	}
}
static void Prepare(void)
{
	assert(Lpc17UsbHwRelease());
	LPC_SC->SCS = 0x60;
	SetReadOnly(&LPC_SC->PLL1STAT, (7U << 8) | (1U << 5) | 3U);
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	assert(Lpc17UsbHwPrepare(12000000));
	s_Count = 0;
	s_DropPhase = 0;
}
int main(void)
{
	ClockTests();
	uint8_t data = 0x55;
	assert(!Lpc17UsbSieRead(0xFE, &data));
	assert(data == 0x55 && s_Count == 0);
	Prepare();
	s_Mask = 0;
	SetReadOnly(&LPC_USB->USBDevIntSt, 0xFFFF);
	assert(Lpc17UsbSieCommand(0xFA));
	assert(s_Count == 1 && s_Codes[0] == 0xFA0500 && s_Mask == 0);
	assert((LPC_USB->USBDevIntSt & ~0x30U) == (0xFFFFU & ~0x30U));
	assert(Lpc17UsbSieWrite(0xD0, 0x81));
	assert(s_Count == 3 && s_Codes[1] == 0xD00500 && s_Codes[2] == 0x810100);
	assert(!Lpc17UsbSieRead(0xFE, 0) && s_Count == 3);
	s_Mask = 1;
	assert(Lpc17UsbSieRead(0xFE, &data));
	assert(data == 0xA5 && s_Count == 5 && s_Mask == 1);
	assert(s_Codes[3] == 0xFE0500 && s_Codes[4] == 0xFE0200);
	for (unsigned phase = 1; phase <= 3; ++phase)
	{
		Prepare();
		s_DropPhase = phase == 1 ? 0x500 : phase == 2 ? 0x100 : 0x200;
		SetReadOnly(&LPC_USB->USBDevIntSt, 0x30); // stale completion cannot pass
		data = 0x55;
		const bool result = phase == 2 ? Lpc17UsbSieWrite(0xFE, 0xAA) :
			Lpc17UsbSieRead(0xFE, &data);
		assert(!result && data == 0x55 && s_Mask == 1);
		const unsigned count = s_Count;
		SetReadOnly(&LPC_USB->USBDevIntSt, 0x30); // late completion
		assert(!Lpc17UsbSieCommand(0xFA) && s_Count == count);
	}
	Prepare();
	assert(Lpc17UsbSieCommand(0xFA));
	SetReadOnly(&LPC_USB->USBClkSt, 0);
	assert(!Lpc17UsbSieCommand(0xFA) && s_Count == 1);
	SetReadOnly(&LPC_USB->USBClkSt, 0x12);
	assert(Lpc17UsbHwRelease());
	assert(!Lpc17UsbSieCommand(0xFA) && s_Count == 1);
	puts("LPC1769 SIE phases, stale flags, timeout latch and interrupt state: PASS");
}
