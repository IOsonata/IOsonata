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
		assert(Addr == 0x1A && RegLen == 1 && Len == 2 && pReg[0] == 0);
		if (ReadFailure)
		{
			return 0;
		}
		pData[0] = Id >> 8;
		pData[1] = Id;
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

		if (*pReg == 0)
		{
			assert(value == 0);
			memset(Registers, 0, sizeof(Registers));
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
		.Gain = 18000
	};

	assert(!codec.Valid() && !codec.Enable());
	assert(!codec.Init(cfg, nullptr));
	assert(codec.Init(cfg, &control));
	assert(codec.Valid() && codec.DeviceID() == 0x8904 && s_Delay == 100);
	assert(control.Registers[0x15] == 0xC02);
	assert(control.Registers[0x16] == 6 && control.Registers[0x19] == 2);
	assert(control.Registers[0x27] == 0x1800 && control.Registers[0x79] == 8);
	assert(control.Registers[0x12] == 2 && control.Registers[0x0C] == 0);
	assert(control.Registers[0x18] == 0x10 && control.Registers[0x0A] == 1);
	assert(control.Registers[0x24] == 240 && control.Registers[0x25] == 0x1F0);
	unsigned writes = control.Writes;

	assert(codec.Enable() && control.Writes == writes);
	assert(codec.Gain(999999) == 23625 && codec.Gain() == 23625);
	assert(control.Registers[0x25] == 0x1FF);
	assert(codec.Gain(-999999) == -71625 && control.Registers[0x25] == 0x101);
	assert(codec.Gain(-200) == -375 && codec.Gain(200) == 375);
	codec.Disable();
	assert(codec.Valid() && control.Registers[0x12] == 0 && control.Registers[0x27] == 0);
	assert(control.Registers[0x16] == 4 && control.Registers[0x24] == 0x100);
	assert(codec.Enable() && control.Registers[0x12] == 2 && control.Registers[0x25] == 0x1C1);
	codec.Reset();
	assert(codec.Valid() && control.Registers[0x27] == 0x1800);
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
	assert(codec.Init(cfg, &control));
	control.FailAt = control.Writes + 1;
	codec.Disable();
	assert(!codec.Valid());
	puts("WM8904 registers, endian, rate, gain, lifecycle and write failures PASS");
}
