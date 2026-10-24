/** Thingy:91 ADXL372 high-g accelerometer test. */
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "sensors/accel_adxl372.h"
#include "board.h"

#define UART_FIFO_SIZE CFIFO_MEMSIZE(256)
static uint8_t s_UartTxBuffer[UART_FIFO_SIZE];
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
static const UARTCfg_t s_UartCfg = {
	UART_DEVNO, s_UartPins, sizeof(s_UartPins)/sizeof(s_UartPins[0]), 115200, 8,
	UART_PARITY_NONE, 1, UART_FLWCTRL_NONE, true, 1, NULL, true, 0, NULL,
	UART_FIFO_SIZE, s_UartTxBuffer,
};
static const IOPinCfg_t s_SpiPins[] = {
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_CS0_PORT, SPI_CS0_PIN, SPI_CS0_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{SPI_CS1_PORT, SPI_CS1_PIN, SPI_CS1_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};
static const SPICfg_t s_SpiCfg = {
	.DevNo=SPI_DEVNO, .Phy=SPIPHY_NORMAL, .Mode=SPIMODE_MASTER,
	.pIOPinMap=s_SpiPins, .NbIOPins=sizeof(s_SpiPins)/sizeof(s_SpiPins[0]),
	.Rate=4000000, .DataSize=8, .MaxRetry=5, .BitOrder=SPIDATABIT_MSB,
	.DataPhase=SPIDATAPHASE_FIRST_CLK, .ClkPol=SPICLKPOL_LOW,
	.ChipSel=SPICSEL_AUTO, .bDmaEn=false, .bIntEn=false, .IntPrio=7,
	.DummyByte=0xFF, .EvtCB=NULL,
};
static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr=1, .OpMode=SENSOR_OPMODE_CONTINUOUS, .Freq=400000, .Scale=200,
	.FltrFreq=0, .Inter=0, .IntPol=DEVINTR_POL_HIGH,
	.IntHandler=NULL, .bFifoEn=false,
};
UART g_Uart;
SPI g_Spi;
AccelAdxl372 g_Accel;

int main()
{
	AccelSensorData_t data;

	if (!g_Uart.Init(s_UartCfg))
		return 1;
	g_Uart.printf("Thingy:91 ADXL372 test\r\n");
	if (!g_Spi.Init(s_SpiCfg) || !g_Accel.Init(s_AccelCfg, &g_Spi, NULL) ||
		!g_Accel.StartSampling())
	{
		g_Uart.printf("ADXL372 initialization failed\r\n");
		return 1;
	}
	while (1)
	{
		if (g_Accel.UpdateData() && g_Accel.Read(data))
		{
			g_Uart.printf("X=%.2f g Y=%.2f g Z=%.2f g\r\n",
				(double)data.X, (double)data.Y, (double)data.Z);
		}
		msDelay(100);
	}
}
