#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_cli.h"
//#include "nrf_cli_rtt.h"
#include "nrf_cli_types.h"

#include "boards.h"

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "board.h"

#include "nrf_cli.h"

static UARTDev_t s_UartDev;

    /**
     * @brief Function for initializing the CLI transport interface.
     *
     * @param[in] p_transport   Pointer to the transfer instance.
     * @param[in] p_config      Pointer to instance configuration.
     * @param[in] evt_handler   Event handler.
     * @param[in] p_context     Pointer to the context passed to event handler.
     *
     * @return Standard error code.
     */
ret_code_t nrf_cli_uart_init(nrf_cli_transport_t const * p_transport,
                   void const *                p_config,
                   nrf_cli_transport_handler_t evt_handler,
                   void *                      p_context)
{
	UARTInit(&s_UartDev, (UARTCfg_t*)p_config);

	return NRF_SUCCESS;
}

/**
 * @brief Function for uninitializing the CLI transport interface.
 *
 * @param[in] p_transport  Pointer to the transfer instance.
 *
 * @return Standard error code.
 */
ret_code_t nrf_cli_uart_uninit(nrf_cli_transport_t const * p_transport)
{
	UARTPowerOff(&s_UartDev);

	return NRF_SUCCESS;
}

/**
 * @brief Function for reconfiguring the transport to work in blocking mode.
 *
 * @param p_transport  Pointer to the transfer instance.
 * @param blocking     If true, the transport is enabled in blocking mode.
 *
 * @return NRF_SUCCESS on successful enabling, error otherwise (also if not supported).
 */
ret_code_t nrf_cli_uart_enable(nrf_cli_transport_t const * p_transport,
                     bool                        blocking)
{
	UARTEnable(&s_UartDev);

	return NRF_SUCCESS;
}

/**
 * @brief Function for writing data to the transport interface.
 *
 * @param[in] p_transport  Pointer to the transfer instance.
 * @param[in] p_data       Pointer to the source buffer.
 * @param[in] length       Source buffer length.
 * @param[in] p_cnt        Pointer to the sent bytes counter.
 *
 * @return Standard error code.
 */
ret_code_t nrf_cli_uart_write(nrf_cli_transport_t const * p_transport,
                    const void *                p_data,
                    size_t                      length,
                    size_t *                    p_cnt)
{
	*p_cnt = UARTTx(&s_UartDev, (uint8_t*)p_data, length);

	return NRF_SUCCESS;
}

/**
 * @brief Function for reading data from the transport interface.
 *
 * @param[in] p_transport  Pointer to the transfer instance.
 * @param[in] p_data       Pointer to the destination buffer.
 * @param[in] length       Destination buffer length.
 * @param[in] p_cnt        Pointer to the received bytes counter.
 *
 * @return Standard error code.
 */
ret_code_t nrf_cli_uart_read(nrf_cli_transport_t const * p_transport,
                   void *                      p_data,
                   size_t                      length,
                   size_t *                    p_cnt)
{
	*p_cnt = UARTRx(&s_UartDev, (uint8_t*)p_data, length);

	return NRF_SUCCESS;
}

const nrf_cli_transport_api_t g_nRFCliUartApi = {
	.init = nrf_cli_uart_init,
	.uninit = nrf_cli_uart_uninit,
	.enable = nrf_cli_uart_enable,
	.write = nrf_cli_uart_write,
	.read = nrf_cli_uart_read
};
