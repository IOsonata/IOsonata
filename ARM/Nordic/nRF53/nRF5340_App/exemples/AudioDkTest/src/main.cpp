/**-------------------------------------------------------------------------
@file	main.cpp

@brief	nRF5340 Audio DK hardware diagnostic and 48-kHz audio test

@author	Nguyen Hoan Hoang
@date	Oct. 24, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "board.h"
#include "idelay.h"
#include "stddev.h"
#include "coredev/i2c.h"
#include "coredev/i2s.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "pwrmgnt/pm_npm1100.h"
#include "sensors/ina230.h"
#include "audio/cs47l63.h"
#include "storage/sdcard.h"

// Set to 0 to run continuous audio without periodic I2S restarts.
#define AUDIO_TEST_I2S_RESTART 0

static const IOPinCfg_t s_UartPins[] = {
	{ 1, 4, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 1, 5, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
};
alignas(4) static uint8_t s_UartMem[CFIFO_MEMSIZE(256)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = 0, .pIOPinMap = s_UartPins,
	.NbIOPins = 2, .Rate = 1000000, .DataBits = 8,
	.Parity = UART_PARITY_NONE, .StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true, .IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL, .bFifoBlocking = true,
	.RxMemSize = 0, .pRxMem = NULL,
	.TxMemSize = sizeof(s_UartMem), .pTxMem = s_UartMem,
	.bDMAMode = false
};
static const IOPinCfg_t s_I2cPins[] = {
	{ 1, 2, 0, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN },
	{ 1, 3, 0, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN }
};
static const I2CCfg_t s_I2cCfg = {
	.DevNo = AUDIO_I2C_DEVNO, .Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER, .pIOPinMap = s_I2cPins,
	.NbIOPins = 2, .Rate = 400000, .MaxRetry = 3,
	.AddrType = I2CADDR_TYPE_NORMAL, .NbSlaveAddr = 0,
	.SlaveAddr = { 0 }, .bDmaEn = false, .bIntEn = false,
	.IntPrio = 6, .EvtCB = NULL
};
static const IOPinCfg_t s_SpiPins[] = {
	{ 0, 8, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 10, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 9, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 11, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 17, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
};
static const SPICfg_t s_SpiCfg = {
	.DevNo = AUDIO_SPI_DEVNO, .Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER, .pIOPinMap = s_SpiPins,
	.NbIOPins = 5, .Rate = 8000000, .DataSize = 8,
	.MaxRetry = 3, .BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO, .bDmaEn = true,
	.bIntEn = false, .IntPrio = 6, .DummyByte = 0, .EvtCB = NULL
};
static const IOPinCfg_t s_I2sPins[] = {
	{ 0, 14, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 16, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 15, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 13, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
};
alignas(4) static uint8_t s_I2sRxFifo[CFIFO_TOTAL_MEMSIZE(512, 4)];
alignas(4) static uint8_t s_I2sTxFifo[CFIFO_TOTAL_MEMSIZE(512, 4)];
static const I2SCfg_t s_I2sCfg = {
	.DevNo = 0, .pIOPinMap = s_I2sPins, .NbIOPins = 5,
	.Mode = I2SMODE_MASTER, .Format = I2SFMT_NORMAL,
	.SampleWidth = I2SSWIDTH_16BITS, .Chan = I2SCHAN_STEREO,
	.Freq = 48000, .MaxRetry = 3, .bDmaEn = true,
	.bIntEn = true, .IntPrio = 6, .EvtCB = NULL,
	.pRxFifoMem = s_I2sRxFifo, .RxFifoMemSize = sizeof(s_I2sRxFifo),
	.pTxFifoMem = s_I2sTxFifo, .TxFifoMemSize = sizeof(s_I2sTxFifo)
};

static UART s_Uart;
static I2C s_I2c;
static SPI s_Spi;
static I2S s_I2s;
static Cs47l63 s_Codec;
static PmnPM1100 s_Pmic;
static Ina230 s_Monitors[4];
#ifdef AUDIO_TEST_SD_CARD
static SDCard s_Card;
#endif
static const uint8_t s_Addresses[4] = { 0x44, 0x45, 0x41, 0x40 };
static const uint32_t s_Shunts[4] = { 510000, 2200000, 2200000, 1000000 };
static const int16_t s_Sine[32] = {
	0, 799, 1567, 2276, 2896, 3406, 3784, 4017,
	4096, 4017, 3784, 3406, 2896, 2276, 1567, 799,
	0, -799, -1567, -2276, -2896, -3406, -3784, -4017,
	-4096, -4017, -3784, -3406, -2896, -2276, -1567, -799
};

// Integer RMS without libm: input is the mean-square magnitude of 128 samples.
static uint32_t AudioSqrt(uint64_t value)
{
	uint32_t root = 0;
	for (uint32_t bit = 1U << 15; bit != 0; bit >>= 1)
	{
		uint32_t candidate = root | bit;
		if ((uint64_t)candidate * candidate <= value)
		{
			root = candidate;
		}
	}
	return root;
}

static void PrintUnsigned64(uint64_t value)
{
	uint32_t high = (uint32_t)(value / 1000000000ULL);
	uint32_t low = (uint32_t)(value % 1000000000ULL);

	if (high != 0U)
	{
		printf("%lu%09lu", (unsigned long)high, (unsigned long)low);
	}
	else
	{
		printf("%lu", (unsigned long)low);
	}
}

static void PrintSigned64(int64_t value)
{
	uint64_t magnitude;

	if (value < 0)
	{
		putchar('-');
		magnitude = (uint64_t)(-(value + 1)) + 1U;
	}
	else
	{
		magnitude = (uint64_t)value;
	}
	PrintUnsigned64(magnitude);
}

static bool CheckCodecPlayback()
{
	uint32_t en;
	uint32_t status;
	uint32_t volume;
	uint32_t clock;
	uint32_t asp;
	uint32_t mixer;

	if (!s_Codec.ReadReg(0x4804U, en) ||
		!s_Codec.ReadReg(0x4808U, status) ||
		!s_Codec.ReadReg(0x4818U, volume) ||
		!s_Codec.ReadReg(0x1404U, clock) ||
		!s_Codec.ReadReg(0x6000U, asp) ||
		!s_Codec.ReadReg(0x8100U, mixer))
	{
		puts("CS47L63 playback register read failed");
		return false;
	}
	printf("Codec OUT_EN=%08lX OUT_STS=%08lX VOL=%08lX\n",
		(unsigned long)en, (unsigned long)status, (unsigned long)volume);
	printf("Codec SYSCLK=%08lX ASP1=%08lX OUT_MIX=%08lX\n",
		(unsigned long)clock, (unsigned long)asp, (unsigned long)mixer);
	if ((en & 2U) == 0 || (status & 2U) == 0)
	{
		puts("CS47L63 output path not active");
		return false;
	}
	if ((volume & 0x100U) != 0)
	{
		puts("CS47L63 output muted");
		return false;
	}
	return true;
}

static bool InitDevices()
{
	PmnPM1100Cfg_t pmicCfg;
	Cs47l63Cfg_t codecCfg;
	Cs47l63AudioCfg_t audioCfg;
	Ina230Cfg_t monitorCfg;

	pmicCfg.IsetPin = { AUDIO_NPM_ISET_PORT, AUDIO_NPM_ISET_PIN,
		IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	if (!s_Pmic.Init(pmicCfg))
	{
		return false;
	}

	// Select the onboard CS47L63 rather than external codec header.
	IOPinClear(AUDIO_CODEC_IF_PORT, AUDIO_CODEC_IF_PIN);
	IOPinConfig(AUDIO_CODEC_IF_PORT, AUDIO_CODEC_IF_PIN, IOPINOP_GPIO,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinClear(AUDIO_CODEC_IF_PORT, AUDIO_CODEC_IF_PIN);

	if (!s_I2c.Init(s_I2cCfg) || !s_Spi.Init(s_SpiCfg))
	{
		return false;
	}
	for (int i = 0; i < 4; i++)
	{
		monitorCfg.DevAddr = s_Addresses[i];
		monitorCfg.RShuntMicroOhm = s_Shunts[i];
		monitorCfg.CurrentLsbMicroAmp = 1;
		monitorCfg.Config = INA230_CONFIG_AVG_1024 |
			INA230_CONFIG_VBUSCT_4156US |
			INA230_CONFIG_VSHCT_4156US |
			INA230_CONFIG_MODE_SHUNT_BUS_CONT;
		if (!s_Monitors[i].Init(monitorCfg, &s_I2c))
		{
			uint8_t reg = INA230_REG_CONFIG;
			uint8_t raw[2] = { 0, 0 };
			int readCount = s_I2c.Read(s_Addresses[i], &reg, 1, raw, 2);
			printf("INA230 %02X init failed: config read=%d", s_Addresses[i], readCount);
			if (readCount == 2)
			{
				printf(", config=%04X", (unsigned)((raw[0] << 8) | raw[1]));
			}
			puts("");
			return false;
		}
		printf("INA230 %02X: %lu uV, ", s_Addresses[i],
			(unsigned long)s_Monitors[i].BusVoltageMicroVolt());
		PrintSigned64(s_Monitors[i].CurrentMicroAmp());
		printf(" uA, ");
		PrintUnsigned64(s_Monitors[i].PowerMicroWatt());
		puts(" uW");
	}

	codecCfg.DevAddr = AUDIO_CODEC_CS;
	codecCfg.ResetPin = { 0, 18, IOPINOP_GPIO, IOPINDIR_OUTPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL };
	codecCfg.IrqPin = { 0, 19, IOPINOP_GPIO, IOPINDIR_INPUT,
		IOPINRES_PULLUP, IOPINTYPE_NORMAL };
	codecCfg.BootTimeoutMs = 200;
	if (!s_Codec.Init(codecCfg, &s_Spi))
	{
		puts("CS47L63 identification/boot failed");
		return false;
	}

	if (!s_I2s.Init(s_I2sCfg) || s_I2s.Rate() != 48000)
	{
		puts("nRF5340 48-kHz I2S initialization failed");
		return false;
	}

	audioCfg.SampleRate = 48000;
	audioCfg.Input = CS47L63_INPUT_PDM;
	audioCfg.HeadphoneEnable = true;
	audioCfg.HeadphoneGainMilliDb = -6000;
	if (!s_Codec.ConfigureAudio(audioCfg))
	{
		puts("CS47L63 48-kHz microphone/headphone initialization failed");
		return false;
	}
	printf("CS47L63 revision %u, I2S %lu Hz\n",
		(unsigned)s_Codec.Revision(), (unsigned long)s_I2s.Rate());
	msDelay(10);
	if (!CheckCodecPlayback())
	{
		puts("Codec playback not ready; continuing I2S diagnostics");
	}
	return true;
}

int main()
{
	alignas(4) uint32_t tx[128];
	alignas(4) uint32_t rx[128];
	unsigned phase;
	unsigned blocks;
#if AUDIO_TEST_I2S_RESTART
	unsigned reports = 0;
	unsigned restarts = 0;
#endif
	int count;

	if (!s_Uart.Init(s_UartCfg))
	{
		while (1);
	}
#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, NULL, _IONBF, 0);
#endif
	puts("nRF5340 Audio DK device test");
	if (!InitDevices())
	{
		puts("Device test FAIL");
		return 1;
	}
	// SD card probing is opt-in: the generic driver performs lengthy
	// synchronous command retries when no card answers. Audio streaming
	// must not depend on optional removable storage.
#ifdef AUDIO_TEST_SD_CARD
	puts("SD card probe starting");
	if (s_Card.Init(&s_Spi, (uint8_t *)NULL, 0))
	{
		printf("SD card sectors: %lu\n", (unsigned long)s_Card.GetNbSect());
	}
	else
	{
		puts("SD card absent/not initialized");
	}
#else
	puts("SD card probe skipped (define AUDIO_TEST_SD_CARD to enable)");
#endif
	puts("Starting continuous I2S audio streaming (1.5 kHz tone, -6 dB output gain)");
#if AUDIO_TEST_I2S_RESTART
	puts("I2S restart test enabled: stop for 100 ms every ten microphone reports");
#endif

	phase = 0;
	blocks = 0;
	while (1)
	{
		for (unsigned i = 0; i < 128; i++)
		{
			// nRF I2S 16-bit stereo DMA packs left/right into one
			// 32-bit word. Advance the 32-sample tone once per frame.
			uint32_t sample = (uint16_t)(s_Sine[phase++ & 31] * 4);
			tx[i] = sample | (sample << 16);
		}
		count = s_I2s.Tx(0, (const uint8_t *)tx, sizeof(tx));
		if (count != (int)sizeof(tx))
		{
			usDelay(100);
			continue;
		}
		if (++blocks == 750)
		{
			int received;
			blocks = 0;
			received = s_I2s.Rx(0, (uint8_t *)rx, sizeof(rx));
			if (received == (int)sizeof(rx))
			{
				// Audio DK has one onboard PDM microphone. The codec can
				// expose its signal in both I2S slots; analyze one mono slot.
				int64_t sum = 0;
				uint64_t squares = 0;
				uint32_t peak = 0;
				for (unsigned i = 0; i < 128; i++)
				{
					int32_t sample = (int16_t)(uint16_t)rx[i];
					uint32_t magnitude = sample < 0 ? (uint32_t)-sample : (uint32_t)sample;
					if (magnitude > peak)
					{
						peak = magnitude;
					}
					sum += sample;
					squares += (uint64_t)(int64_t)sample * sample;
				}
				// Remove DC offset from the measured signal power.
				uint64_t power = (squares - (uint64_t)(sum * sum / 128)) / 128;
				printf("I2S mono mic peak=%lu RMS(AC)=%lu VBAT=%lu uV\n",
					(unsigned long)peak, (unsigned long)AudioSqrt(power),
					(unsigned long)s_Monitors[0].BusVoltageMicroVolt());
			}
			else
			{
				puts("I2S microphone capture timeout");
			}
#if AUDIO_TEST_I2S_RESTART
			// Restart the peripheral in place every ten reports. Do not
			// reinitialize the codec or MCU: this exercises I2S STOPPED,
			// IRQ shutdown, FIFO reset and restart handling.
			if (++reports == 10)
			{
				reports = 0;
				printf("I2S restart %u: muting and stopping for 100 ms\n", ++restarts);
				if (!s_Codec.HeadphoneMute(true))
				{
					puts("Codec headphone mute failed");
				}
				msDelay(10);
				s_I2s.Disable();
				msDelay(100);
				s_I2s.Enable();
				// Allow the DMA to queue and transmit fresh audio blocks
				// before enabling the headphone output again.
				for (unsigned i = 0; i < 8; i++)
				{
					for (unsigned j = 0; j < 128; j++)
					{
						uint32_t sample = (uint16_t)(s_Sine[phase++ & 31] * 4);
						tx[j] = sample | (sample << 16);
					}
					unsigned retry = 10000;
					while (s_I2s.Tx(0, (const uint8_t *)tx, sizeof(tx)) != (int)sizeof(tx) &&
						retry > 0)
					{
						retry--;
						usDelay(100);
					}
					if (retry == 0)
					{
						puts("I2S restart TX timeout");
						break;
					}
				}
				msDelay(10);
				if (!s_Codec.HeadphoneMute(false))
				{
					puts("Codec headphone unmute failed");
				}
				printf("I2S restart %u: streaming resumed\n", restarts);
			}
#endif
		}
	}
}
