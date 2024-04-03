/**-------------------------------------------------------------------------
@example	UartPrbsTx based on nRF Connect SDK


@brief	The firmware generates and transmits Prbs packets over UART interface.

@author Thinh Tran
@date 	Feb 08, 2024

@license

Copyright (c) 2024, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

uint8_t Prbs8(uint8_t CurVal);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The size of the Tx and Rx buffers */
#define TX_BUFF_SIZE 16
#define TX_BUFF_SIZE 16

/* The receiving timeout period */
#define RECEIVE_TIMEOUT 100

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* The Tx and Rx buffers */
static uint8_t rx_buf[TX_BUFF_SIZE] = {0};
static uint8_t tx_buf[TX_BUFF_SIZE];

static uint8_t g_Seed = 0xFF;
static uint8_t g_Ret = 0;


/* The callback function for UART */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{

	case UART_TX_DONE:
	{
		if (g_Ret == 0)
		{
		for (int i = 0; i < TX_BUFF_SIZE; i++)
			{
				tx_buf[i] = g_Seed;
				g_Seed = Prbs8(g_Seed);
			}
		}
		g_Ret = uart_tx(uart, tx_buf, TX_BUFF_SIZE, SYS_FOREVER_MS);
	}
		break;
	case UART_RX_RDY:
		if ((evt->data.rx.len) == 1)
		{
			if (evt->data.rx.buf[evt->data.rx.offset] == '1')
			{
				//gpio_pin_toggle_dt(&led0);
				printk("1\r\n");
			}
			else if (evt->data.rx.buf[evt->data.rx.offset] == '2')
			{
				//gpio_pin_toggle_dt(&led1);
				printk("2\r\n");
			}
			else if (evt->data.rx.buf[evt->data.rx.offset] == '3')
			{
				//gpio_pin_toggle_dt(&led2);
				printk("3\r\n");
			}
		}
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
		break;

	default:
		break;
	}
}

const struct uart_config uart_cfg = {
	.baudrate = 1000000,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE};

/**
 * Brief
 *
 * 8-bit PRBS generator ( X^7 + X^6 + 1 ).  Repetition period = 127
 *
 * @param  CurVal : Current PRBS value
 * 					Initial value must be non zero
 *
 * @return Next PRBS value
 */
uint8_t Prbs8(uint8_t CurVal)
{
	uint8_t newbit = (((CurVal >> 6) ^ (CurVal >> 5)) & 1);
	return ((CurVal << 1) | newbit) & 0x7f;
}

int main(void)
{
	int ret;

	int err = uart_configure(uart, &uart_cfg);
	if (err == -ENOSYS)
	{
		return -ENOSYS;
	}

	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret)
	{
		return 1;
	}

	if (!device_is_ready(uart))
	{
		printk("UART device not ready\r\n");
		return 1;
	}

	/* Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer */
	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (ret)
	{
		return 1;
	}

	/* Generate the first Prbs packet */
	for (int i = 0; i < TX_BUFF_SIZE; i++)
	{
		tx_buf[i] = g_Seed;
		g_Seed = Prbs8(g_Seed);
	}

	g_Ret = uart_tx(uart, tx_buf, TX_BUFF_SIZE, SYS_FOREVER_MS);

	while (1)
	{
		;
	}
}