// Exercise the real C and C++ register transfer paths against a RAM register file.
#include "cc3xx_intrf.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
static uint32_t s_Registers[1024];
static bool s_Enabled;
uintptr_t Cc3xxBase() { return (uintptr_t)s_Registers; }
bool Cc3xxEnable() { s_Enabled = true; return true; }
void Cc3xxDisable() { s_Enabled = false; }
int main()
{
	Cc3xxIntrf intrf;
	DeviceIntrf *api = &intrf;
	uint8_t address[6] = {}, input[34], output[34];
	uint32_t offset = 0x400, value = 0;
	int owner, other;
	assert(intrf.Init());
	DevIntrf_t *dev = intrf;
	intrf.Enable();
	assert(s_Enabled);
	memcpy(address+1, &offset, 4);
	for (unsigned i = 0; i < sizeof(input); ++i) input[i] = (uint8_t)(3*i+1);
	for (int len : {4, 16, 32}) {
		memset(output, 0xA5, sizeof(output));
		assert(api->Write(0, address+1, 4, input+1, len) == len);
		assert(intrf.DeviceIntrf::Read(0, address+1, 4, output+1, len) == len);
		assert(memcmp(input+1, output+1, len) == 0);
		assert(output[0] == 0xA5 && output[len+1] == 0xA5);
		memset(output, 0xA5, sizeof(output));
		assert(intrf.DeviceIntrf::Write(0, address+1, 4, input+1, len) == len);
		assert(api->Read(0, address+1, 4, output+1, len) == len);
		assert(memcmp(input+1, output+1, len) == 0);
		assert(output[0] == 0xA5 && output[len+1] == 0xA5);
	}
	// A transfer in progress must block both complete-transfer implementations.
	assert(intrf.StartTx(0));
	assert(api->Write(0, address+1, 4, input+1, 4) == 0);
	assert(api->Read(0, address+1, 4, output+1, 4) == 0);
	assert(intrf.TxData(address+1, 4) == 4);
	assert(intrf.TxData(input+1, 4) == 4);
	intrf.StopTx();
	assert(api->Read(0, address+1, 4, output+1, 4) == 4);
	assert(memcmp(input+1, output+1, 4) == 0);
	assert(intrf.OpHold(&owner));
	assert(!intrf.StartRx(CC3XX_ADDR_RNG));
	assert(api->Write(0, address+1, 4, input+1, 4) == 4);
	assert(!intrf.OpRelease(&other));
	assert(!intrf.OpHold(&other));
	assert(intrf.OpRelease(&owner));
	// Malformed word transfers must not touch the register file or caller buffer.
	memset(output, 0xA5, sizeof(output));
	for (int len : {-1, 0, 1, 3, 5}) {
		assert(api->Read(0, address+1, 4, output+1, len) == 0);
		assert(api->Write(0, address+1, 4, input+1, len) == 0);
	}
	assert(api->Read(0, nullptr, 4, output, 4) == 0);
	assert(api->Write(0, address+1, 3, input, 4) == 0);
	assert(api->Read(1, address+1, 4, output, 4) == 0);
	assert(api->Write(1, address+1, 4, input, 4) == 0);
	address[1]++;
	assert(api->Read(0, address+1, 4, output, 4) == 0);
	assert(api->Write(0, address+1, 4, input, 4) == 0);
	address[1]--;
	for (uint8_t b : output) assert(b == 0xA5);
	assert(!atomic_flag_test_and_set(&dev->bBusy));
	atomic_flag_clear(&dev->bBusy);
	// CPU overhead comparison only: RAM registers do not model peripheral timing.
	for (bool generic : {true, false}) {
		auto start = std::chrono::steady_clock::now();
		for (unsigned i = 0; i < 1000000; ++i) {
			if (generic) {
				assert(intrf.DeviceIntrf::Write(0, address+1, 4, (uint8_t *)&i, 4) == 4);
				assert(intrf.DeviceIntrf::Read(0, address+1, 4, (uint8_t *)&value, 4) == 4);
			} else {
				assert(api->Write(0, address+1, 4, (uint8_t *)&i, 4) == 4);
				assert(api->Read(0, address+1, 4, (uint8_t *)&value, 4) == 4);
			}
			assert(value == i);
		}
		printf("%s: %lld us for 1000000 register write/read pairs (host only)\n",
			generic ? "Generic transfers" : "CC3xx transfers",
			(long long)std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now()-start).count());
	}
	intrf.Disable();
	assert(!s_Enabled);
	puts("CC3xx transport: C/C++ paths, unaligned buffers, bounds and exclusion passed");
}
