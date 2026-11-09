/**-------------------------------------------------------------------------
@file spi_link_test.c
@brief Link the same SPI driver through the C and C++ public interfaces.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include "coredev/spi.h"
#include "esp32xx_spi.h"
static const IOPinCfg_t s_Pins[] = {
	{0, 6, ESP32_PINOP_SPI2_SCK, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 7, ESP32_PINOP_SPI2_MISO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 2, ESP32_PINOP_SPI2_MOSI, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
int main(void)
{
	SPICfg_t cfg = {0};
	uint8_t data;
	uint8_t reg = 0;
#ifdef __cplusplus
	SPI bus;
#else
	SPIDev_t bus = {0};
#endif

	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 4;
	cfg.DataSize = 8;
	cfg.Rate = 100000;
#ifdef __cplusplus
	if (bus.Init(cfg))
	{
		bus.Read(0, &reg, 1, &data, 1);
		bus.Write(0, &reg, 1, &data, 1);
		bus.PowerOff();
	}
#else
	if (SPIInit(&bus, &cfg))
	{
		DeviceIntrfRead(&bus.DevIntrf, 0, &reg, 1, &data, 1);
		DeviceIntrfWrite(&bus.DevIntrf, 0, &reg, 1, &data, 1);
		SPIPowerOff(&bus);
	}
#endif
	return 0;
}
