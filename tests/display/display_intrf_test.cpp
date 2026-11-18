// Exercise SysLog through the production display adapter, without UART.
#include <cassert>
#include <cstdio>
#include <string>
#include "display/display_intrf.h"
#include "syslog.h"

class CaptureDisplay : public Display {
public:
	std::string Text;
	using Device::Valid;
	CaptureDisplay() { Valid(true); }
	bool Init(const DisplayCfg_t &, DeviceIntrf *) override { return true; }
	bool Enable() override { return true; }
	void Disable() override {}
	void Reset() override {}
	void Clear() override { Text.clear(); }
protected:
	void Print(const char *pText, uint32_t) override { Text += pText; }
};

extern "C" void IOPinSet(int, int) {}
extern "C" void IOPinClear(int, int) {}

int main()
{
	CaptureDisplay display;
	DisplayIntrf output;
	SysLog log;
	alignas(4) uint8_t memory[SYSLOG_MEMSIZE(4, 256)];
	const SysLogCfg_t cfg = {memory, sizeof(memory), 256, true};
	const uint8_t raw[] = {'1', '0', '0', '%', '\r', '\n'};

	assert(output.Tx(0, raw, sizeof(raw)) == 0);
	assert(!output.Init(nullptr));
	assert(output.Init(&display));
	assert(log.Init(cfg, &output));
	assert(output.Tx(0, raw, sizeof(raw)) == sizeof(raw));
	assert(display.Text == "100%\r\n");
	display.Clear();
	std::string longText(200, 'A');
	longText.replace(30, 4, "%s%n");
	assert(log.Printf("%s", longText.c_str()) == 200);
	assert(display.Text == longText);
	assert(log.Flush() == 0);

	output.Disable();
	assert(log.Printf("queued\r\n") == 8);
	assert(display.Text == longText);
	output.Enable();
	assert(log.Flush() == 8);
	assert(display.Text == longText + "queued\r\n");
	assert(log.Flush() == 0);

	assert(output.StartTx(0));
	assert(!output.StartTx(0));
	output.StopTx();
	assert(output.StartTx(0));
	output.StopTx();
	assert(!output.StartRx(0));
	assert(output.Tx(0, raw, sizeof(raw)) == sizeof(raw));

	display.Valid(false);
	assert(log.Printf("invalid") == 7);
	assert(log.Flush() == 0);
	display.Valid(true);
	assert(log.Flush() == 7);
	output.PowerOff();
	output.Enable();
	assert(output.Tx(0, raw, sizeof(raw)) == 0);
	assert(output.Init(&display));
	const uint8_t nul[] = {'a', 0, 'b'};
	display.Clear();
	assert(output.Tx(0, nul, sizeof(nul)) == 3);
	assert(display.Text == "ab");
	std::puts("Display SysLog: long text, literal %, queue/retry, lifecycle and locking PASS");
}
