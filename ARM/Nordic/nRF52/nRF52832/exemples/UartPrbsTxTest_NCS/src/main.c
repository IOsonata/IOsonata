/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Controlling LEDs through UART. Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
/* STEP 3 - Include the header file of the UART driver in main.c */
#include <zephyr/drivers/uart.h>

uint8_t Prbs8(uint8_t CurVal);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 10.1.1 - Define the size of the receive buffer */
#define RECEIVE_BUFF_SIZE 16

#define TX_BUFF_SIZE 16

/* STEP 10.2 - Define the receiving timeout period */
#define RECEIVE_TIMEOUT 100

/* STEP 5.1 - Get the device pointers of the LEDs through gpio_dt_spec */
/* The nRF7002dk has only 2 LEDs so this step uses a compile-time condition to reflect the DK you are building for */
#if 0
#if defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP) || defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP_NS)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#else
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#endif

/* STEP 4.1 - Get the device pointer of the UART hardware */
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
// const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(arduino_serial));


/* STEP 9.1 - Define the transmission buffer, which is a buffer to hold the data to be sent over UART */
static uint8_t welcom_tx_buf[] = {"nRF Connect SDK Fundamentals Course\n\r"
						   "Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\n\r"};

/* STEP 10.1.2 - Define the receive buffer */
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};


static uint8_t tx_buf[TX_BUFF_SIZE];

static uint8_t g_Seed = 0xFF;
static uint8_t g_Ret = 0;


/* STEP 7 - Define the callback function for UART */
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

	/* STEP 8 - Register the UART callback function */
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret)
	{
		return 1;
	}

	/* STEP 4.2 - Verify that the UART device is ready */
	if (!device_is_ready(uart))
	{
		printk("UART device not ready\r\n");
		return 1;
	}
	else
	{
		//printk("UART is ready\r\n");
	}

#if 0
	/* STEP 5.2 - Verify that the LED devices are ready */
	if (!device_is_ready(led0.port))
	{
		printk("GPIO device is not ready\r\n");
		return 1;
	}

	/* STEP 6 - Configure the GPIOs of the LEDs */
	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 1;
	}
	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 1;
	}
	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 1;
	}
#endif

	/* STEP 9.2 - Send the data over UART by calling uart_tx() */
	ret = uart_tx(uart, welcom_tx_buf, sizeof(welcom_tx_buf), SYS_FOREVER_MS);
	if (ret)
	{
		return 1;
	}
	/* STEP 10.3  - Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer */
	ret = uart_rx_enable(uart, rx_buf, sizeof rx_buf, RECEIVE_TIMEOUT);
	if (ret)
	{
		return 1;
	}

	//uint8_t d = 0xFF;
	//ret = 0;

#if 1
	for (int i = 0; i < TX_BUFF_SIZE; i++)
	{
		tx_buf[i] = g_Seed;
		g_Seed = Prbs8(g_Seed);
	}

	g_Ret = uart_tx(uart, tx_buf, TX_BUFF_SIZE, SYS_FOREVER_MS);
#endif


	while (1)
	{

		//uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);

		// BYTE MODE
		//uart_tx(uart, &d, sizeof(d), SYS_FOREVER_MS);
		//d = Prbs8(d);

		#if 0
		if (ret == 0)
		{
			for (int i = 0; i < TX_BUFF_SIZE; i++)
			{
				d = Prbs8(d);
				tx_buf[i] = d;
			}
		}
		ret = uart_tx(uart, tx_buf, TX_BUFF_SIZE, SYS_FOREVER_MS);
		#endif
	}
}