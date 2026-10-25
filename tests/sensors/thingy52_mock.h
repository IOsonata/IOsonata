#pragma once
#include <assert.h>
#include <string.h>
#include <vector>
#include "device_intrf.h"

struct Operation {
	bool Write;
	uint8_t Reg;
	int Len;
	uint8_t Data[8];
};

class MockIntrf : public DeviceIntrf {
public:
	MockIntrf() : FailRead(-1), FailWrite(-1), ShortRead(-1), ShortWrite(-1),
		CommandFails(false), Ccs(false), Sx(false), Wake(false), RequireWake(false)
	{
		memset(Reg, 0, sizeof(Reg));
		memset(Algorithm, 0, sizeof(Algorithm));
		memset(Raw, 0, sizeof(Raw));
	}
	operator DevIntrf_t * () override { return nullptr; }
	DEVINTRF_TYPE Type() override { return DEVINTRF_TYPE_I2C; }
	uint32_t Rate(uint32_t Rate) override { return Rate; }
	uint32_t Rate() override { return 100000; }
	void Disable() override {}
	void Enable() override {}
	int Rx(uint32_t, uint8_t *, int) override { return 0; }
	int Tx(uint32_t, const uint8_t *pData, int Len) override
	{
		assert(!RequireWake || Wake);
		assert(Ccs && Len == 1 && pData[0] == 0xF4);
		if (CommandFails)
			return 0;
		Reg[0] |= 0x80;
		return 1;
	}
	bool StartRx(uint32_t) override { return false; }
	int RxData(uint8_t *, int) override { return 0; }
	void StopRx() override {}
	bool StartTx(uint32_t) override { return false; }
	int TxData(const uint8_t *, int) override { return 0; }
	void StopTx() override {}
	bool RequestToSend(int) override { return true; }
	void Reset() override {}
	int Read(uint32_t, const uint8_t *pCmd, int CmdLen, uint8_t *pData, int Len) override
	{
		int count;
		const uint8_t *source;

		assert(CmdLen == 1 && Len > 0 && Len <= 8);
		assert(!RequireWake || Wake);
		if (*pCmd == FailRead)
			return 0;
		count = *pCmd == ShortRead ? Len - 1 : Len;
		source = Ccs && *pCmd == 2 ? Algorithm : Ccs && *pCmd == 3 ? Raw : Reg + *pCmd;
		memcpy(pData, source, count);
		Ops.push_back({ false, *pCmd, Len, {} });
		if (Ccs && *pCmd == 2)
			Reg[0] &= ~8;
		if (Ccs && *pCmd == 0xE0)
			Reg[0] &= ~1;
		return count;
	}
	int Write(uint32_t, const uint8_t *pCmd, int CmdLen, const uint8_t *pData, int Len) override
	{
		Operation op = { true, *pCmd, Len, {} };
		int i;
		int count;

		assert(CmdLen == 1 && Len > 0 && Len <= 8);
		assert(!RequireWake || Wake);
		memcpy(op.Data, pData, Len);
		Ops.push_back(op);
		if (*pCmd == FailWrite)
			return 0;
		count = *pCmd == ShortWrite ? Len - 1 : Len;
		for (i = 0; i < count; ++i)
		{
			if (Sx && *pCmd >= 0x18 && *pCmd <= 0x1B)
				Reg[(uint8_t)(*pCmd + i)] &= ~pData[i];
			else
				Reg[(uint8_t)(*pCmd + i)] = pData[i];
		}
		if (Ccs && *pCmd == 0xFF && count == 4)
		{
			Reg[0] = 0x10;
			Reg[1] = 0;
		}
		if (Sx && *pCmd == 0x7D && pData[0] == 0x34)
		{
			memset(Reg, 0, sizeof(Reg));
			Reg[0x0E] = 0xFF;
			Reg[0x0F] = 0xFF;
			Reg[0x10] = 0xFF;
			Reg[0x11] = 0xFF;
		}
		return count;
	}
	void PrepareCcs()
	{
		Ccs = true;
		Reg[0] = 0x10;
		Reg[0x20] = 0x81;
		Reg[0x21] = 0x12;
	}
	void Sample()
	{
		Reg[0] = 0x98;
		Algorithm[0] = 0x04;
		Algorithm[1] = 0xD2; // 1234 ppm
		Algorithm[2] = 0;
		Algorithm[3] = 56;
		Algorithm[4] = 0x98;
		Algorithm[6] = 0x29; // 10uA, ADC=0x123
		Algorithm[7] = 0x23;
		Raw[0] = 0x33; // 12uA, ADC=0x345; deliberately different from ALG_RESULT
		Raw[1] = 0x45;
	}
	uint8_t Reg[256];
	uint8_t Algorithm[8];
	uint8_t Raw[2];
	int FailRead;
	int FailWrite;
	int ShortRead;
	int ShortWrite;
	bool CommandFails;
	bool Ccs;
	bool Sx;
	bool Wake;
	bool RequireWake;
	std::vector<Operation> Ops;
};

