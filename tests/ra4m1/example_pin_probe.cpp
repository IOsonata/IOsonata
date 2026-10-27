/* Example pin-map checks against the native MCU package masks, not wiring.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <cassert>
#include <cstdio>
#include "example_api.h"
// The package masks (Ra4m1PinValid) of the port. Not the example_include
// iopinctrl.h test double: that one only records LED toggles.
#include "../../ARM/Renesas/RA4/RA4M1/include/iopinctrl.h"
#include "board.h"

[[maybe_unused]] static bool IsPin(int Port, int Pin, int P, int N)
{
	return Port == P && Pin == N;
}

#if defined(I2C_MASTER_DEVNO) || defined(I2C_SLAVE_DEVNO)
// Legal IIC pins, PSEL 7: IIC0 SCL P204/P400/P408, SDA P401/P407, IIC1 SCL
// P100/P205, SDA P101/P206 (ARM/Renesas/RA4/RA4M1/README.md, i2c_ra4m1.cpp)
static bool I2cPinOk(int DevNo, bool bScl, int Port, int Pin)
{
	if (DevNo == 0)
	{
		return bScl ? IsPin(Port, Pin, 2, 4) || IsPin(Port, Pin, 4, 0) || IsPin(Port, Pin, 4, 8) :
					  IsPin(Port, Pin, 4, 1) || IsPin(Port, Pin, 4, 7);
	}
	return bScl ? IsPin(Port, Pin, 1, 0) || IsPin(Port, Pin, 2, 5) :
				  IsPin(Port, Pin, 1, 1) || IsPin(Port, Pin, 2, 6);
}
#endif

#if defined(SPI_MASTER_DEVNO) || defined(SPI_SLAVE_DEVNO)
// RSPI pins, PSEL 6 (ARM/Renesas/RA4/RA4M1/README.md, spi_ra4m1.cpp)
// Signal 0 RSPCK, 1 MISO, 2 MOSI, 3 SSL0
static bool SpiPinOk(int DevNo, int Sig, int Port, int Pin)
{
	static const int pins[2][4][3] = {
		{ {412, 102, -1}, {410, 100, -1}, {411, 101, -1}, {413, 103, -1} },
		{ {111, 204, -1}, {110, 202, -1}, {109, 203, -1}, {108, 112, 205} },
	};
	for (int i = 0; i < 3; i++)
	{
		if (pins[DevNo][Sig][i] == Port * 100 + Pin)
		{
			return true;
		}
	}
	return false;
}
#endif

int main()
{
#ifdef LED_PINS_MAP
	static_assert(LED0_PINOP == IOPINOP_GPIO && LED1_PINOP == IOPINOP_GPIO &&
				  LED2_PINOP == IOPINOP_GPIO, "named LED pin operations");
	const IOPinCfg_t leds[] = LED_PINS_MAP;
	assert(leds[0].PortNo == LED0_PORT && leds[0].PinNo == LED0_PIN);
	assert(leds[1].PortNo == LED1_PORT && leds[1].PinNo == LED1_PIN);
	assert(leds[2].PortNo == LED2_PORT && leds[2].PinNo == LED2_PIN);
	static_assert(sizeof(leds)/sizeof(leds[0]) == 3, "three example outputs");
	for (const auto &pin : leds) {
		assert(Ra4m1PinValid(pin.PortNo, pin.PinNo));
		assert(pin.PinOp == IOPINOP_GPIO && pin.PinDir == IOPINDIR_OUTPUT);
	}
#endif
#ifdef BUTTON_PINS_MAP
	const IOPinCfg_t buttons[] = BUTTON_PINS_MAP;
	assert(buttons[0].PortNo == BUT1_PORT && buttons[0].PinNo == BUT1_PIN &&
		   buttons[0].PinOp == BUT1_PINOP);
	assert(Ra4m1PinValid(buttons[0].PortNo, buttons[0].PinNo));
	assert(buttons[0].PortNo == 0 && buttons[0].PinNo == 0 && BUT1_INT == 6);
	assert(buttons[0].PinDir == IOPINDIR_INPUT && buttons[0].Res == IOPINRES_PULLUP);
#endif
#ifdef UART_RX_PORT
	// SCI0, PSEL 4: RXD0/TXD0 on P100/P101 or P410/P411
	static_assert(UART_DEVNO == 0, "SCI0");
	static_assert(UART_RX_PINOP == IOPINOP_FUNC3 && UART_TX_PINOP == IOPINOP_FUNC3, "SCI PSEL 4");
	assert(Ra4m1PinValid(UART_RX_PORT, UART_RX_PIN) && Ra4m1PinValid(UART_TX_PORT, UART_TX_PIN));
	assert((IsPin(UART_RX_PORT, UART_RX_PIN, 1, 0) && IsPin(UART_TX_PORT, UART_TX_PIN, 1, 1)) ||
		   (IsPin(UART_RX_PORT, UART_RX_PIN, 4, 10) && IsPin(UART_TX_PORT, UART_TX_PIN, 4, 11)));
#ifdef UART_DMA_MODE
	assert(!UART_DMA_MODE);
#endif
#endif
#ifdef UART_PINS
	const IOPinCfg_t pins[] = UART_PINS;
	assert(pins[0].PortNo == UART_RX_PORT && pins[0].PinNo == UART_RX_PIN &&
		   pins[0].PinOp == UART_RX_PINOP && pins[0].PinDir == IOPINDIR_INPUT);
	assert(pins[1].PortNo == UART_TX_PORT && pins[1].PinNo == UART_TX_PIN &&
		   pins[1].PinOp == UART_TX_PINOP && pins[1].PinDir == IOPINDIR_OUTPUT);
	static_assert(sizeof(pins)/sizeof(pins[0]) == 2, "RX and TX only");
#endif
#ifdef I2C_MASTER_DEVNO
	static_assert(I2C_MASTER_DEVNO != I2C_SLAVE_DEVNO, "master and slave on two IIC units");
	static_assert(I2C_MASTER_SDA_PINOP == IOPINOP_FUNC6 && I2C_MASTER_SCL_PINOP == IOPINOP_FUNC6, "IIC PSEL 7");
	static_assert(I2C_SLAVE_SDA_PINOP == IOPINOP_FUNC6 && I2C_SLAVE_SCL_PINOP == IOPINOP_FUNC6, "IIC PSEL 7");
	assert(Ra4m1PinValid(I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN));
	assert(Ra4m1PinValid(I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN));
	assert(Ra4m1PinValid(I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN));
	assert(Ra4m1PinValid(I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN));
	assert(I2cPinOk(I2C_MASTER_DEVNO, false, I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN));
	assert(I2cPinOk(I2C_MASTER_DEVNO, true, I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN));
	assert(I2cPinOk(I2C_SLAVE_DEVNO, false, I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN));
	assert(I2cPinOk(I2C_SLAVE_DEVNO, true, I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN));
	assert(!I2C_MASTER_DMA_ENABLE && !I2C_MASTER_INT_ENABLE);
	assert(!I2C_SLAVE_DMA_ENABLE && I2C_SLAVE_INT_ENABLE);
#endif
#ifdef SPI_MASTER_DEVNO
	static_assert(SPI_MASTER_DEVNO != SPI_SLAVE_DEVNO, "master and slave on two RSPI units");
	static_assert(SPI_MASTER_SCK_PINOP == IOPINOP_FUNC5 && SPI_MASTER_MISO_PINOP == IOPINOP_FUNC5 &&
				  SPI_MASTER_MOSI_PINOP == IOPINOP_FUNC5, "RSPI PSEL 6");
	static_assert(SPI_MASTER_CS_PINOP == IOPINOP_GPIO, "master chip select is a GPIO");
	static_assert(SPI_SLAVE_SCK_PINOP == IOPINOP_FUNC5 && SPI_SLAVE_MISO_PINOP == IOPINOP_FUNC5 &&
				  SPI_SLAVE_MOSI_PINOP == IOPINOP_FUNC5 && SPI_SLAVE_CS_PINOP == IOPINOP_FUNC5,
				  "RSPI PSEL 6, slave select SSL0");
	assert(Ra4m1PinValid(SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN));
	assert(Ra4m1PinValid(SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN));
	assert(Ra4m1PinValid(SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN));
	assert(Ra4m1PinValid(SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN));
	assert(Ra4m1PinValid(SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN));
	assert(Ra4m1PinValid(SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN));
	assert(Ra4m1PinValid(SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN));
	assert(Ra4m1PinValid(SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN));
	assert(SpiPinOk(SPI_MASTER_DEVNO, 0, SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN));
	assert(SpiPinOk(SPI_MASTER_DEVNO, 1, SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN));
	assert(SpiPinOk(SPI_MASTER_DEVNO, 2, SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN));
	assert(SpiPinOk(SPI_SLAVE_DEVNO, 0, SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN));
	assert(SpiPinOk(SPI_SLAVE_DEVNO, 1, SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN));
	assert(SpiPinOk(SPI_SLAVE_DEVNO, 2, SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN));
	assert(SpiPinOk(SPI_SLAVE_DEVNO, 3, SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN));
	// The RSPI slave supports CPHA = 1 only: modes 1 and 3
	static_assert((SPI_TEST_MODE_MASK & ~0x0A) == 0 && SPI_TEST_MODE_MASK != 0, "modes 1 and 3");
	assert(!SPI_MASTER_DMA_ENABLE && !SPI_SLAVE_DMA_ENABLE && SPI_SLAVE_INT_ENABLE);
#endif
	printf("PASS: example pin map, %d-pin package\n", RA4M1_PACKAGE_PINS);
}
