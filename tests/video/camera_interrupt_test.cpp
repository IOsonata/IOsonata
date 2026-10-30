#include <cassert>
#include <cstdio>
#include <cstring>
#include "video/cam_arducam_mega.h"

static CamArducamMega s_Camera;
static uint8_t *s_Target;
static int s_Length;
static int s_FrameOffset;
static int s_Register;
static bool s_Pending;
static bool s_Inline;
static bool s_Timeout;
static bool s_OneByteSync;
static int s_Completions;

static int Event(DevIntrf_t *, DEVINTRF_EVT evt, uint8_t *, int len)
{
	s_Camera.IntrfEvent(evt, len);
	return 0;
}

static void Complete()
{
	if (!s_Pending)
	{
		return;
	}
	if (s_Target)
	{
		for (int i = 0; i < s_Length; i++)
		{
			int value = 0;
			switch (s_Register)
			{
				case ARDUCAM_MEGA_STATUS_REG: value = 6; break;
				case ARDUCAM_MEGA_SENSOR_ID_REG: value = 0x85; break;
				case ARDUCAM_MEGA_FIFO_SIZE_REG: value = 700 & 255; break;
				case ARDUCAM_MEGA_FIFO_SIZE_REG + 1: value = 700 >> 8; break;
				case ARDUCAM_MEGA_BURST_FIFO_READ: value = s_FrameOffset++; break;
			}
			s_Target[i] = value;
		}
	}
	s_Pending = false;
	s_Completions++;
	Event(nullptr, DEVINTRF_EVT_COMPLETED, s_Target, s_Length);
}

void usDelay(uint32_t)
{
	if (!s_Timeout)
	{
		Complete();
	}
}

class MockSpi : public DeviceIntrf {
public:
	DevIntrf_t Dev = {};
	operator DevIntrf_t *() override { return &Dev; }
	uint32_t Rate(uint32_t rate) override { return rate; }
	uint32_t Rate() override { return 8000000; }
	int Read(uint32_t, const uint8_t *cmd, int cmdlen, uint8_t *buffer, int len) override
	{
		assert(!s_Pending);
		s_Target = buffer;
		s_Length = len;
		s_Register = cmd[0];
		if (cmd[0] == ARDUCAM_MEGA_BURST_FIFO_READ)
		{
			assert(cmdlen == (s_FrameOffset == 0 ? 2 : 1));
		}
		s_Pending = true;
		if (!Dev.bIntEn || (s_OneByteSync && len == 1))
		{
			Complete();
			return len;
		}
		if (s_Inline) Complete();
		return -1;
	}
	int Tx(uint32_t, const uint8_t *data, int len) override
	{
		assert(!s_Pending && len >= 2 && (data[0] & 0x80));
		s_Target = nullptr;
		s_Length = len;
		s_Pending = true;
		if (!Dev.bIntEn)
		{
			Complete();
			return len;
		}
		if (s_Inline) Complete();
		return -1;
	}
};

int main()
{
	MockSpi spi;
	VideoDevCfg_t cfg = {};
	cfg.OpMode = VIDEO_OPMODE_SINGLE;
	cfg.OutFmt = {VIDEO_FMT_JPEG, 320, 240, 0};
	for (int mode = 0; mode < 4; mode++)
	{
		spi.Dev.bIntEn = mode != 0;
		spi.Dev.EvtCB = Event;
		s_Inline = mode == 2;
		s_OneByteSync = mode == 3;
		s_FrameOffset = 0;
		assert(s_Camera.Init(cfg, &spi));
		assert(s_Camera.Start());
		assert(s_Camera.UpdateData());
		assert(s_Camera.FrameLength() == 700);
		uint8_t frame[976] = {};
		assert(s_Camera.ReadFrame(frame, sizeof(frame)) == 700);
		for (int i = 0; i < 700; i++) assert(frame[i] == (uint8_t)i);
		assert(!s_Pending && s_Camera.ReadFrame(frame, sizeof(frame)) == 0);
		assert(s_Camera.Brightness(1));
	}
	// No callback cannot support interrupt transfers.
	spi.Dev.EvtCB = nullptr;
	assert(!s_Camera.Init(cfg, &spi));
	spi.Dev.EvtCB = Event;
	assert(s_Camera.Init(cfg, &spi));
	assert(s_Camera.Start());
	assert(s_Camera.UpdateData());
	s_FrameOffset = 0;
	s_Timeout = true;
	uint8_t output[32];
	memset(output, 0xA5, sizeof(output));
	assert(s_Camera.ReadFrame(output, sizeof(output)) == 0);
	assert(s_Pending && !s_Camera.Init(cfg, &spi));
	Complete(); // Late DMA writes only camera-owned storage.
	for (uint8_t b : output) assert(b == 0xA5);
	assert(s_Camera.ReadFrame(output, sizeof(output)) == 0);
	s_Timeout = false;
	assert(s_Camera.Init(cfg, &spi));
	printf("camera polling, delayed IRQ, inline IRQ, one-byte fallback and timeout: PASS (%d completions)\n", s_Completions);
}
