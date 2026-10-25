/**-------------------------------------------------------------------------
@file	main.cpp
@brief	nRF5340 Audio DK hardware diagnostic and 48-kHz audio test
@author	Hoang Nguyen Hoan
@date	Oct. 24, 2026
@license
MIT License. Copyright (c) 2026 I-SYST inc.
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
#include "coredev/iopinctrl.h"
#include "pwrmgnt/pm_npm1100.h"
#include "sensors/ina230.h"
#include "audio/cs47l63.h"
#include "storage/sdcard.h"

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
static SDCard s_Card;
static const uint8_t s_Addresses[4] = { 0x44, 0x45, 0x41, 0x40 };
static const uint32_t s_Shunts[4] = { 510000, 2200000, 2200000, 1000000 };
static const int16_t s_Sine[32] = {
	0, 799, 1567, 2276, 2896, 3406, 3784, 4017,
	4096, 4017, 3784, 3406, 2896, 2276, 1567, 799,
	0, -799, -1567, -2276, -2896, -3406, -3784, -4017,
	-4096, -4017, -3784, -3406, -2896, -2276, -1567, -799
};

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
			printf("INA230 %02X failed\n", s_Addresses[i]);
			return false;
		}
		printf("INA230 %02X: %lu uV, %lld uA, %llu uW\n",
			s_Addresses[i],
			(unsigned long)s_Monitors[i].BusVoltageMicroVolt(),
			(long long)s_Monitors[i].CurrentMicroAmp(),
			(unsigned long long)s_Monitors[i].PowerMicroWatt());
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
	audioCfg.HeadphoneGainMilliDb = -24000;
	if (!s_Codec.ConfigureAudio(audioCfg))
	{
		puts("CS47L63 48-kHz microphone/headphone initialization failed");
		return false;
	}
	printf("CS47L63 revision %u, I2S %lu Hz\n",
		(unsigned)s_Codec.Revision(), (unsigned long)s_I2s.Rate());
	return true;
}

int main()
{
	alignas(4) uint32_t tx[128];
	alignas(4) uint32_t rx[128];
	unsigned phase;
	unsigned blocks;
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
	// SD card is optional and can be absent; do not write user data.
	if (s_Card.Init(&s_Spi))
	{
		printf("SD card sectors: %lu\n", (unsigned long)s_Card.GetNbSect());
	}
	else
	{
		puts("SD card absent/not initialized");
	}

	phase = 0;
	blocks = 0;
	while (1)
	{
		for (unsigned i = 0; i < 128; i += 2)
		{
			// Each 32-bit DMA word carries a signed 16-bit slot.
			// Both channels use the same 1.5-kHz tone at 48 kHz.
			uint32_t sample = (uint32_t)(int32_t)s_Sine[phase++ & 31];
			tx[i] = sample;
			tx[i + 1] = sample;
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
				printf("Audio TX/RX continuing, mic sample %08lX, VBAT %lu uV\n",
					(unsigned long)rx[0],
					(unsigned long)s_Monitors[0].BusVoltageMicroVolt());
			}
			else
			{
				puts("I2S capture timeout");
			}
		}
	}
}
