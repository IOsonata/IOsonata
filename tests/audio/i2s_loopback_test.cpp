#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define main LoopbackDemoMain
#include "../../exemples/audio/i2s_loopback_demo.cpp"
#undef main

static const unsigned s_RxChunks[] = { 4, 28, 256 };
static const unsigned s_TxChunks[] = { 4, 12, 256 };
static uint8_t s_Input[2048];
static uint8_t s_Output[2048];
static unsigned s_InputSize;
static unsigned s_InputPos;
static unsigned s_OutputPos;
static unsigned s_RxChunk;
static unsigned s_TxChunk;
static unsigned s_RxCalls;
static unsigned s_TxCalls;
static bool s_Stall;
static bool s_Forever;
static uint64_t s_Ticks;

static void ControlDisable(DevIntrf_t *)
{
}

extern "C" void I2CBusReset(I2CDev_t *)
{
	assert(false);
}

extern "C" void I2CSetReadRqstData(I2CDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void I2CSetWriteRqstBuffer(I2CDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY Phy)
{
	assert(false);
	return Phy;
}

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(false);
}

extern "C" void IOPinSet(int, int)
{
	assert(false);
}

extern "C" void IOPinClear(int, int)
{
	assert(false);
}

static uint64_t TickCount(TimerDev_t *)
{
	return ++s_Ticks;
}

extern "C" bool TimerInit(TimerDev_t *, const TimerCfg_t *)
{
	assert(false);
	return false;
}

static bool Start(DevIntrf_t *, uint32_t Addr)
{
	assert(Addr == 0);
	return true;
}

static void Stop(DevIntrf_t *)
{
}

static int Receive(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	s_RxCalls++;
	if (s_Forever)
	{
		memset(pBuff, 0, Len);
		return Len;
	}
	unsigned count = s_InputSize - s_InputPos;

	if (count > unsigned(Len))
	{
		count = Len;
	}
	if (count > s_RxChunk)
	{
		count = s_RxChunk;
	}
	memcpy(pBuff, s_Input + s_InputPos, count);
	s_InputPos += count;
	return count;
}

static int Transmit(DevIntrf_t *, const uint8_t *pData, int Len)
{
	s_TxCalls++;
	if (s_Stall && s_TxCalls % 3 == 0)
	{
		return 0;
	}
	unsigned count = s_TxChunk;

	if (count > unsigned(Len))
	{
		count = Len;
	}
	assert(s_OutputPos + count <= sizeof(s_Output));
	memcpy(s_Output + s_OutputPos, pData, count);
	s_OutputPos += count;
	return count;
}

static void ResetModel()
{
	s_InputSize = sizeof(s_Input);
	s_InputPos = 0;
	s_OutputPos = 0;
	s_RxCalls = 0;
	s_TxCalls = 0;
	s_Stall = false;
	s_Forever = false;
	s_Pending = 0;
	s_Offset = 0;
	s_Frames = 0;
	s_Peak = 0;
	s_AudioError = AUDIO_ERROR_NONE;
	DevIntrf_t *audio = s_Audio;

	atomic_flag_clear(&audio->bBusy);
	memset(s_Output, 0xA5, sizeof(s_Output));
}

int main()
{
	DevIntrf_t *control = s_Control;
	DevIntrf_t *audio = s_Audio;
	TimerDev_t *clock = s_Clock;

	control->Disable = ControlDisable;
	atomic_store(&control->EnCnt, 1);
	audio->StartRx = Start;
	audio->StartTx = Start;
	audio->StopRx = Stop;
	audio->StopTx = Stop;
	audio->RxData = Receive;
	audio->TxData = Transmit;
	audio->MaxRetry = 0;
	clock->Freq = 1000;
	clock->GetTickCount = TickCount;
	assert(s_AudioCfg.Chan == I2SCHAN_STEREO && s_AudioCfg.bDmaEn && s_AudioCfg.bIntEn);
	assert(s_AudioCfg.pRxFifoMem == s_RxMem && s_AudioCfg.pTxFifoMem == s_TxMem);
	assert(s_AudioCfg.RxFifoMemSize >= int(CFIFO_TOTAL_MEMSIZE(8192, 4)));
	assert(s_AudioPins[I2S_SDI_IOPIN_IDX].PortNo == IOPORTC);
	assert(s_AudioPins[I2S_SDO_IOPIN_IDX].PortNo == IOPORTD);
	for (unsigned i = 0; i < sizeof(s_Input); i++)
	{
		s_Input[i] = i * 37 + 19;
	}
	s_Input[0] = 0;
	s_Input[1] = 0x80;
	for (unsigned rx : s_RxChunks)
	{
		for (unsigned tx : s_TxChunks)
		{
			ResetModel();
			s_RxChunk = rx;
			s_TxChunk = tx;
			s_Stall = true;
			for (unsigned i = 0; i < 4096 && s_OutputPos < sizeof(s_Input); i++)
			{
				unsigned calls = s_RxCalls;
				bool pending = s_Offset < s_Pending;

				PumpAudio();
				assert(!pending || s_RxCalls == calls);
				assert(s_AudioError == AUDIO_ERROR_NONE);
			}
			assert(s_OutputPos == sizeof(s_Input));
			assert(memcmp(s_Input, s_Output, sizeof(s_Input)) == 0);
			assert(s_Frames == sizeof(s_Input) / 4 && s_Peak == 32768);
			assert(s_LastSample > 0 && s_Offset == s_Pending);
			assert(!PumpAudio());
		}
	}
	ResetModel();
	s_RxChunk = 256;
	s_TxChunk = 256;
	assert(PrepareStream());
	assert(s_InputPos == sizeof(s_Input) && s_OutputPos == 512 && s_Frames == 0);
	for (unsigned i = 0; i < s_OutputPos; i++)
	{
		assert(s_Output[i] == 0);
	}
	ResetModel();
	s_Forever = true;
	assert(!PrepareStream() && s_RxCalls == 256);
	ResetModel();
	s_RxChunk = 2;
	assert(!PumpAudio() && s_AudioError != AUDIO_ERROR_NONE && s_OutputPos == 0);
	ResetModel();
	s_RxChunk = 256;
	s_TxChunk = 2;
	assert(!PumpAudio() && s_AudioError != AUDIO_ERROR_NONE);
	ResetModel();
	s_TxChunk = 128;
	assert(!PrepareStream());
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_TX_READY, nullptr, 0);
	assert(s_AudioError == AUDIO_ERROR_NONE);
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(s_AudioError != AUDIO_ERROR_NONE);
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_STATECHG, nullptr, 0);
	assert(s_AudioError != AUDIO_ERROR_NONE);
	assert(strcmp(AudioErrorText(0), "SAI/DMA error") == 0);
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(strcmp(AudioErrorText(0), "SAI/DMA error") == 0);
	ResetModel();
	s_LastSample = 100;
	assert(AudioErrorText(600) == nullptr);
	assert(strcmp(AudioErrorText(601), "RX timeout") == 0);
	ResetModel();
	s_LastSample = UINT32_MAX - 100;
	assert(AudioErrorText(399) == nullptr);
	assert(strcmp(AudioErrorText(400), "RX timeout") == 0);
	ResetModel();
	s_LastSample = 100;
	s_Pending = 256;
	s_Offset = 128;
	assert(strcmp(AudioErrorText(601), "TX stalled") == 0);
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(strcmp(AudioErrorText(0), "RX FIFO overflow") == 0);
	puts("I2S loopback config, exact forwarding, short TX, stalls, startup and errors PASS");
}
