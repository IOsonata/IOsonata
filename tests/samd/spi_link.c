// Runtime-selected C SPI configuration keeps every supported mode available.
#include "coredev/spi.h"

static volatile unsigned s_Mode;
static SPIDev_t s_Spi;
static const IOPinCfg_t s_Pins[] = {
	{0, 7, IOPINOP_PERIPHC}, {0, 9, IOPINOP_PERIPHD},
	{0, 6, IOPINOP_PERIPHC}, {0, 8, IOPINOP_GPIO}
};

int main(void)
{
	unsigned mode = s_Mode;
	SPICfg_t cfg = {
		.DevNo = 0, .Phy = SPIPHY_NORMAL,
		.Mode = mode == 4 ? SPIMODE_SLAVE : SPIMODE_MASTER,
		.pIOPinMap = s_Pins, .NbIOPins = 4, .Rate = 1000000,
		.DataSize = 8, .ChipSel = SPICSEL_AUTO,
		.bDmaEn = (mode & 2) != 0, .bIntEn = (mode & 1) != 0,
		.IntPrio = 3, .DummyByte = 0xff
	};

	if (!SPIInit(&s_Spi, &cfg))
	{
		return 1;
	}
	SPIPowerOff(&s_Spi);
	return 0;
}
