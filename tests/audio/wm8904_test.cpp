#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "audio/wm8904.h"

static unsigned s_Delay;

class CodecControl : public DeviceIntrf {
public:
	uint16_t Registers[256]{};
	unsigned Writes = 0;
	unsigned FailAt = 0;
	uint16_t Id = 0x8904;
	bool ReadFailure = false;
	bool MclkPresent = false;
	uint32_t SysClk = 4000000;
	unsigned BlockedAccesses = 0;
	int DropReg = -1;
	bool ServoStuck = false;
	unsigned ServoReads = 0;

	operator DevIntrf_t *()
	{
		return nullptr;
	}

	uint32_t Rate(uint32_t Value)
	{
		return Value;
	}

	uint32_t Rate()
	{
		return 100000;
	}

	DEVINTRF_TYPE Type()
	{
		return DEVINTRF_TYPE_I2C;
	}

	int Read(uint32_t Addr, const uint8_t *pReg, int RegLen, uint8_t *pData, int Len)
	{
		assert(Addr == 0x1A && RegLen == 1 && Len == 2);
		if (ReadFailure || (*pReg != 0x16 && !MclkPresent && (Registers[0x16] & 4)))
		{
			return 0;
		}
		uint16_t value = *pReg == 0 ? Id : Registers[*pReg];

		if (*pReg == 0x4D)
		{
			assert(Registers[0x43] == 3 && Registers[0x5A] == 0x33);
			value = !ServoStuck && ++ServoReads >= 3 ? 3 : 0;
		}

		pData[0] = value >> 8;
		pData[1] = value;
		return 2;
	}

	int Write(uint32_t Addr, const uint8_t *pReg, int RegLen, const uint8_t *pData, int Len)
	{
		assert(Addr == 0x1A && RegLen == 1 && Len == 2);
		if (++Writes == FailAt)
		{
			return 1;
		}
		uint16_t value = uint16_t(pData[0]) << 8 | pData[1];

		// Rev 4.1 p100: double DAC OSR requires SYSCLK >= 6 MHz,
		// including while muted during startup or shutdown.
		if (*pReg == 0x21 && (value & 0x40))
		{
			assert(SysClk >= 6000000);
		}
		// WM8904 Rev 4.1: with CLK_SYS_ENA set, only R22 is accessible
		// without MCLK. Transferring the bytes does not apply the setting.
		if (*pReg != 0x16 && !MclkPresent && (Registers[0x16] & 4))
		{
			BlockedAccesses++;
			return 2;
		}
		if (*pReg == DropReg)
		{
			return 2;
		}
		if (*pReg == 0x5A && (value & 0x88))
		{
			assert(!ServoStuck && ServoReads >= 3 && Registers[0x43] == 3);
			assert(Registers[0x21] & 8);
		}
		if (*pReg == 0x21 && !(value & 8))
		{
			assert(Registers[0x5A] == 0xFF && ServoReads >= 3);
		}
		if (*pReg == 0)
		{
			assert(value == 0);
			memset(Registers, 0, sizeof(Registers));
			ServoReads = 0;
		}
		else
		{
			Registers[*pReg] = value;
		}
		return 2;
	}
};

extern "C" void msDelay(uint32_t Delay)
{
	s_Delay += Delay;
}

int main()
{
	CodecControl control;
	Wm8904 codec;
	Wm8904Cfg_t cfg = {
		.DevAddr = 0x1A,
		.Freq = 15625,
		.MClkFreq = 4000000,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Format = I2SFMT_NORMAL,
		.MicInput = 2,
		.Gain = 18000,
		.bHeadphoneEn = false,
		.HeadphoneGain = -24000
	};

	assert(!codec.Valid() && !codec.Enable());
	assert(!codec.Init(cfg, nullptr));
	assert(codec.Init(cfg, &control));
	assert(control.BlockedAccesses == 0);
	assert(codec.Valid() && codec.DeviceID() == 0x8904 && s_Delay == 100);
	assert(control.Registers[0x15] == 0xC02);
	assert(control.Registers[0x16] == 6 && control.Registers[0x19] == 2);
	assert(control.Registers[0x27] == 0x1800 && control.Registers[0x79] == 8);
	assert(control.Registers[0x12] == 2 && control.Registers[0x0C] == 0);
	assert(control.Registers[0x06] == 0);
	assert(!codec.Mute(true) && codec.Valid());
	assert(control.Registers[0x18] == 0x10 && control.Registers[0x0A] == 1);
	assert(control.Registers[0x24] == 239 && control.Registers[0x25] == 0x1EF);
	unsigned writes = control.Writes;

	assert(codec.Enable() && control.Writes == writes);
	control.MclkPresent = true;
	assert(codec.Gain(999999) == 17625 && codec.Gain() == 17625);
	assert(control.Registers[0x25] == 0x1EF);
	assert(codec.Gain(-999999) == -71625 && control.Registers[0x25] == 0x101);
	assert(codec.Gain(-200) == -375 && codec.Gain(200) == 375);
	control.MclkPresent = false;
	codec.Disable();
	assert(control.BlockedAccesses == 0);
	assert(codec.Valid() && control.Registers[0x12] == 0 && control.Registers[0x27] == 0);
	assert(control.Registers[0x16] == 0 && control.Registers[0x24] == 0x100);
	assert(codec.Enable() && control.Registers[0x12] == 2 && control.Registers[0x25] == 0x1C1);
	codec.Reset();
	assert(codec.Valid() && control.Registers[0x27] == 0x1800);
	assert(control.BlockedAccesses == 0);
	codec.PowerOff();
	assert(!codec.Valid() && !codec.Enable() && control.Registers[0x16] == 0);
	cfg.MicInput = 1;
	cfg.SampleWidth = I2SSWIDTH_24BITS;
	cfg.Format = I2SFMT_ALIGN_RIGHT;
	assert(codec.Init(cfg, &control) && control.Registers[0x27] == 0x1000 && control.Registers[0x19] == 8);
	cfg.Format = I2SFMT_ALIGN_LEFT;
	assert(codec.Init(cfg, &control) && control.Registers[0x19] == 9);
	codec.PowerOff();
	cfg.MClkFreq = 4000256;
	assert(!codec.Init(cfg, &control));
	cfg.MClkFreq = 3999999;
	assert(!codec.Init(cfg, &control));
	cfg.MClkFreq = 4000000;
	cfg.MicInput = 0;
	assert(!codec.Init(cfg, &control));
	cfg.MicInput = 2;
	cfg.SampleWidth = I2SSWIDTH_8BITS;
	assert(!codec.Init(cfg, &control));
	cfg.SampleWidth = I2SSWIDTH_16BITS;
	control.Id = 0;
	assert(!codec.Init(cfg, &control) && !codec.Valid());
	control.Id = 0x8904;
	control.ReadFailure = true;
	assert(!codec.Init(cfg, &control) && !codec.Valid());
	control.ReadFailure = false;
	control.Writes = 0;
	assert(codec.Init(cfg, &control));
	writes = control.Writes;
	codec.PowerOff();
	for (unsigned failure = 1; failure <= writes; failure++)
	{
		control.Writes = 0;
		control.FailAt = failure;
		assert(!codec.Init(cfg, &control));
		assert(!codec.Valid() && !codec.Enable());
		codec.PowerOff();
	}
	control.FailAt = 0;
	const int droppedRegisters[] = { 0x15, 0x19, 0x18, 0x79, 0x27, 0x12, 0x24, 0x16 };

	for (int reg : droppedRegisters)
	{
		control.DropReg = reg;
		assert(!codec.Init(cfg, &control));
		assert(!codec.Valid());
		control.DropReg = -1;
		codec.PowerOff();
	}
	// MCU-only reset can leave SYSCLK enabled while the MCU stops MCLK.
	control.Registers[0x16] = 6;
	assert(codec.Init(cfg, &control));
	assert(control.BlockedAccesses == 0);
	control.FailAt = control.Writes + 1;
	codec.Disable();
	assert(!codec.Valid());
	control.FailAt = 0;
	control.MclkPresent = true;
	cfg.MicInput = 0;
	cfg.bHeadphoneEn = true;
	cfg.HeadphoneGain = -24000;
	control.Writes = 0;
	assert(codec.Init(cfg, &control));
	unsigned headphoneWrites = control.Writes;

	assert(control.Registers[0x12] == 0x0C && control.Registers[0x27] == 0);
	assert(control.Registers[0x0E] == 3 && control.Registers[0x62] == 1);
	assert(control.Registers[0x06] == 0);
	assert(control.Registers[0x68] == 1);
	assert(control.Registers[0x5A] == 0xFF && control.Registers[0x21] == 0x0600);
	assert(control.Registers[0x39] == 33 && control.Registers[0x3A] == 0xA1);
	assert(codec.HeadphoneGain() == -24000 && control.ServoReads == 3);
	assert(codec.Mute(true) && control.Registers[0x21] == 0x0608);
	assert(control.Registers[0x12] == 0x0C && control.Registers[0x0E] == 3);
	assert(control.Registers[0x16] == 6 && control.Registers[0x5A] == 0xFF);
	assert(codec.Mute(false) && control.Registers[0x21] == 0x0600);
	assert(!codec.MicrophoneClock(false));
	assert(!codec.MicrophoneClockOnly());
	assert(codec.HeadphoneGain(999999) == 6000 && control.Registers[0x39] == 63);
	assert(codec.HeadphoneGain(-999999) == -57000 && control.Registers[0x39] == 0);
	assert(codec.HeadphoneGain(-24100) == -24000);
	control.MclkPresent = false;
	codec.Disable();
	assert(codec.Valid() && control.Registers[0x5A] == 0 && control.Registers[0x62] == 0);
	assert(control.Registers[0x0E] == 0 && control.Registers[0x12] == 0);
	assert(!codec.Enable() && !codec.Valid());
	assert(control.Registers[0x62] == 0);
	control.MclkPresent = true;
	assert(codec.Init(cfg, &control));
	codec.PowerOff();
	control.ServoStuck = true;
	unsigned delayBefore = s_Delay;

	assert(!codec.Init(cfg, &control) && !codec.Valid());
	assert(s_Delay - delayBefore == 400);
	assert(control.Registers[0x62] == 0 && control.Registers[0x5A] == 0);
	control.ServoStuck = false;
	for (unsigned failure = 1; failure <= headphoneWrites; failure++)
	{
		control.Writes = 0;
		control.FailAt = failure;
		assert(!codec.Init(cfg, &control) && !codec.Valid());
		assert(control.Registers[0x62] == 0 && control.Registers[0x5A] == 0);
		codec.PowerOff();
	}
	control.FailAt = 0;
	cfg.MicInput = 2;
	assert(codec.Init(cfg, &control));
	assert(control.Registers[0x12] == 0x0E && control.Registers[0x27] == 0x1800);
	assert(control.Registers[0x68] == 0 && control.Registers[0x62] == 1);
	assert(control.Registers[0x06] == 0);
	assert(codec.Mute(true) && control.Registers[0x21] == 0x0608);
	assert(control.Registers[0x12] == 0x0E && control.Registers[0x27] == 0x1800);
	assert(control.Registers[0x16] == 6 && control.Registers[0x79] == 8);
	assert(codec.Mute(false) && control.Registers[0x21] == 0x0600);
	uint16_t captureRegisters[256];

	memcpy(captureRegisters, control.Registers, sizeof(captureRegisters));
	assert(codec.MicrophoneClock(false) && control.Registers[0x79] == 2);
	captureRegisters[0x79] = 2;
	assert(memcmp(captureRegisters, control.Registers, sizeof(captureRegisters)) == 0);
	assert(codec.MicrophoneClock(true) && control.Registers[0x79] == 8);
	captureRegisters[0x79] = 8;
	assert(memcmp(captureRegisters, control.Registers, sizeof(captureRegisters)) == 0);
	control.Registers[0x1A] = 0x15;
	memcpy(captureRegisters, control.Registers, sizeof(captureRegisters));
	unsigned clockOnlyWrites = control.Writes;

	assert(codec.MicrophoneClockOnly());
	clockOnlyWrites = control.Writes - clockOnlyWrites;
	captureRegisters[0x12] = 0x0C;
	captureRegisters[0x27] = 0;
	captureRegisters[0x1A] = 0x115;
	captureRegisters[0x16] |= 8;
	captureRegisters[0x79] = 1;
	assert(memcmp(captureRegisters, control.Registers, sizeof(captureRegisters)) == 0);
	const uint32_t clockRequests[] = { 4000000, 2000000, 1333333, 1000000, 727273, 666667, 500000, 333333, 250000 };

	for (unsigned code = 0; code < sizeof(clockRequests) / sizeof(clockRequests[0]); code++)
	{
		assert(codec.MicrophoneClockOnly(clockRequests[code]) == clockRequests[code]);
		captureRegisters[0x1A] = (code << 8) | 0x15;
		assert(memcmp(captureRegisters, control.Registers, sizeof(captureRegisters)) == 0);
	}
	assert(codec.MicrophoneClockOnly(UINT32_MAX) == 4000000);
	assert(codec.MicrophoneClockOnly(0) == 250000);
	assert(codec.MicrophoneClock(false));
	assert(control.Registers[0x79] == 2 && control.Registers[0x12] == 0x0C && control.Registers[0x27] == 0);
	assert(codec.MicrophoneClockOnly(1000000) == 1000000);
	assert(control.Registers[0x79] == 1 && control.Registers[0x1A] == 0x315);

	codec.Disable();
	assert(control.Registers[0x16] == 0 && !codec.MicrophoneClockOnly());
	assert(codec.Enable());
	assert(control.Registers[0x16] == 6 && control.Registers[0x79] == 8);
	assert(control.Registers[0x12] == 0x0E && control.Registers[0x27] == 0x1800);
	for (unsigned failure = 1; failure <= clockOnlyWrites; failure++)
	{
		assert(codec.Init(cfg, &control));
		control.FailAt = control.Writes + failure;
		assert(!codec.MicrophoneClockOnly() && !codec.Valid());
		assert(control.Registers[0x12] == 0 && control.Registers[0x62] == 0);
		control.FailAt = 0;
	}
	const int clockOnlyDropped[] = { 0x27, 0x12, 0x1A, 0x16, 0x79 };

	for (int reg : clockOnlyDropped)
	{
		assert(codec.Init(cfg, &control));
		control.DropReg = reg;
		assert(!codec.MicrophoneClockOnly() && !codec.Valid());
		control.DropReg = -1;
	}
	assert(codec.Init(cfg, &control));
	control.ReadFailure = true;
	assert(!codec.MicrophoneClockOnly() && !codec.Valid());
	control.ReadFailure = false;
	assert(codec.Init(cfg, &control));
	control.DropReg = 0x79;
	assert(!codec.MicrophoneClock(false) && !codec.Valid());
	assert(control.Registers[0x12] == 0 && control.Registers[0x62] == 0);
	control.DropReg = -1;
	assert(codec.Init(cfg, &control));
	assert(control.Registers[0x79] == 8);
	control.FailAt = control.Writes + 1;
	assert(!codec.MicrophoneClock(false) && !codec.Valid());
	control.FailAt = 0;
	assert(codec.Init(cfg, &control));
	control.ReadFailure = true;
	assert(!codec.MicrophoneClock(false) && !codec.Valid());
	control.ReadFailure = false;
	codec.PowerOff();
	// Clock limits and DAC OSR selection at both datasheet boundaries.
	const uint32_t clocks[] = { 2999999, 3000000, 4000000, 5999999, 6000000, 12288000 };

	for (uint32_t clock : clocks)
	{
		cfg.Freq = clock / 256;
		cfg.MClkFreq = clock;
		control.SysClk = clock;
		if (clock < 3000000)
		{
			assert(!codec.Init(cfg, &control) && !codec.Valid());
			continue;
		}
		assert(codec.Init(cfg, &control));
		assert(control.Registers[0x21] == (clock >= 6000000 ? 0x0640 : 0x0600));
		assert(codec.Mute(true));
		assert(control.Registers[0x21] == (clock >= 6000000 ? 0x0648 : 0x0608));
		assert(codec.Mute(false));
		codec.Disable();
		assert(!codec.Mute(true));
		assert(control.Registers[0x21] == (clock >= 6000000 ? 0x0648 : 0x0608));
		assert(codec.Enable());
		codec.PowerOff();
	}
	cfg.Freq = 15625;
	cfg.MClkFreq = 4000000;
	control.SysClk = 4000000;
	control.DropReg = 0x21;
	assert(!codec.Init(cfg, &control) && !codec.Valid());
	control.DropReg = -1;
	// Failed or ignored live mute writes must shut down the outputs.
	assert(codec.Init(cfg, &control));
	assert(codec.Mute(true));
	control.DropReg = 0x21;
	assert(!codec.Mute(false) && !codec.Valid());
	assert(control.Registers[0x12] == 0 && control.Registers[0x62] == 0);
	control.DropReg = -1;
	assert(codec.Init(cfg, &control));
	control.FailAt = control.Writes + 1;
	assert(!codec.Mute(true) && !codec.Valid());
	control.FailAt = 0;
	assert(codec.Init(cfg, &control));
	control.ReadFailure = true;
	assert(!codec.Mute(true) && !codec.Valid());
	control.ReadFailure = false;
	assert(codec.Init(cfg, &control));
	control.MclkPresent = false;
	assert(!codec.Mute(true) && !codec.Valid());
	assert(control.Registers[0x12] == 0 && control.Registers[0x62] == 0);
	control.MclkPresent = true;
	puts("WM8904 capture/playback, clock, gain, servo ordering/timeout and failures PASS");
}
