/**-------------------------------------------------------------------------
@file i2c_link_test.c
@brief Link the same I2C driver through the C and C++ public interfaces.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include "coredev/i2c.h"
#include "esp32xx_i2c.h"
static const IOPinCfg_t s_Pins[] = {
	{0, 4, ESP32_PINOP_I2C0_SDA, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	{0, 5, ESP32_PINOP_I2C0_SCL, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
};
int main(void)
{
	I2CCfg_t cfg = {0};
	uint8_t data;
	uint8_t reg = 0;
#ifdef __cplusplus
	I2C bus;
#else
	I2CDev_t bus = {0};
#endif

	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
#ifdef __cplusplus
	if (bus.Init(cfg))
	{
		bus.Read(0x50, &reg, 1, &data, 1);
		bus.Write(0x50, &reg, 1, &data, 1);
		bus.PowerOff();
	}
#else
	if (I2CInit(&bus, &cfg))
	{
		I2CRead(&bus, 0x50, &reg, 1, &data, 1);
		I2CWrite(&bus, 0x50, &reg, 1, &data, 1);
		I2CPowerOff(&bus);
	}
#endif
	return 0;
}
