// Exercise the real shared receive example over a fragmented in-memory UART.
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
static unsigned errors, frames;
static int TestPrintf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	if (std::strstr(format, "PRBS %u errors")) errors++;
	if (std::strstr(format, "frames %u errors %u"))
	{
		frames = va_arg(ap, unsigned);
		assert(va_arg(ap, unsigned) == 0);
	}
	va_end(ap);
	return 0;
}
#define printf TestPrintf
#define main SlipExampleMain
#include "../../exemples/uart/uart_slip_prbs_rx.cpp"
#undef main
#undef printf

struct Finished {};
static std::vector<uint8_t> wire;
static size_t offset;
static bool gap, fragment;
static int Read(DevIntrf_t *, uint8_t *buf, int len)
{
	assert(len == 1);
	if (offset == wire.size()) throw Finished{};
	if (fragment && (gap = !gap)) return 0;
	*buf = wire[offset++];
	return 1;
}
bool UARTInit(UARTDev_t * const dev, const UARTCfg_t *cfg)
{
	(void)cfg;
	dev->DevIntrf.RxData = Read;
	dev->DevIntrf.StartRx = [](DevIntrf_t *, uint32_t) { return true; };
	dev->DevIntrf.StopRx = [](DevIntrf_t *) {};
	return true;
}
int main(int argc, char **)
{
	fragment = argc > 1;
	uint8_t value = 0xff;
	for (unsigned frame = 0; frame < 256; frame++)
	{
		// Empty, exact-buffer, and oversized frames, with escaped payloads.
		unsigned length = frame % 3 == 0 ? 0 : frame % 3 == 1 ? 600 : 1500;
		for (unsigned i = 0; i < length; i++)
		{
			value = Prbs8(value);
			if (value == 0xc0 || value == 0xdb)
			{
				wire.push_back(0xdb);
				wire.push_back(value == 0xc0 ? 0xdc : 0xdd);
			}
			else wire.push_back(value);
		}
		wire.push_back(0xc0);
	}
	try { SlipExampleMain(); assert(false); }
	catch (const Finished &) {}
	assert(errors == 0 && frames == 256 && offset == wire.size());
	std::puts(fragment ? "PASS fragmented SLIP example" : "PASS buffered SLIP example");
}
