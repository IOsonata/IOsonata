/** Thingy:91 X GD25LE255E read-only SPI NOR validation. */
#include <stdint.h>

#include "coredev/spi.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "storage/nvm.h"
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
	{SPI_CS_PORT, SPI_CS_PIN, SPI_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};
static const SPICfg_t s_SpiCfg = {
	.DevNo=SPI_DEVNO, .Phy=SPIPHY_NORMAL, .Mode=SPIMODE_MASTER,
	.pIOPinMap=s_SpiPins, .NbIOPins=sizeof(s_SpiPins)/sizeof(s_SpiPins[0]),
	.Rate=8000000, .DataSize=8, .MaxRetry=5, .BitOrder=SPIDATABIT_MSB,
	.DataPhase=SPIDATAPHASE_FIRST_CLK, .ClkPol=SPICLKPOL_LOW,
	.ChipSel=SPICSEL_AUTO, .bDmaEn=false, .bIntEn=false, .IntPrio=7,
	.DummyByte=0xFF, .EvtCB=NULL,
};
static const NvmCfg_t s_FlashCfg = {
	.DevNo=0, .BaseAddr=0, .TotalSize=32ULL*1024ULL*1024ULL,
	.EraseSize=4096, .SectorSize=512, .PageSize=256, .WriteGran=1,
	.AddrSize=4, .DevId=0x1960C8, .DevIdSize=3,
	.RdCmd={0,0}, .WrCmd={0,0}, .QuadEnable={0,0}, .WrProtMask=0,
	.WrProtPin={-1,-1}, .WriteDelayUs=0, .bIntEn=false,
	.EvtHandler=NULL, .pWaitCB=NULL, .pInitCB=NULL,
};
UART g_Uart;
SPI g_Spi;
Nvm g_Flash;

int main()
{
	uint8_t data[16];
	int i;

	if (!g_Uart.Init(s_UartCfg))
		return 1;
	g_Uart.printf("Thingy:91 X GD25LE255E test\r\n");
	if (!g_Spi.Init(s_SpiCfg) || !g_Flash.Init(s_FlashCfg, &g_Spi))
	{
		g_Uart.printf("GD25LE255E initialization/JEDEC check failed\r\n");
		return 1;
	}
	g_Uart.printf("JEDEC=0x%06lx size=%lu bytes\r\n",
		(unsigned long)g_Flash.ReadId(3), (unsigned long)g_Flash.Size());
	if (g_Flash.Read(0, data, sizeof(data)) != (int)sizeof(data))
	{
		g_Uart.printf("flash read failed\r\n");
		return 1;
	}
	g_Uart.printf("offset 0:");
	for (i = 0; i < (int)sizeof(data); i++)
		g_Uart.printf(" %02x", (unsigned)data[i]);
	g_Uart.printf("\r\n");
	while (1)
		msDelay(1000);
}
