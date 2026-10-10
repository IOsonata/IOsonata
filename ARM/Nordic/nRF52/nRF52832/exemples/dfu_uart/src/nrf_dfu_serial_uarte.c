/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**-------------------------------------------------------------------------
@file	nrf_dfu_serial_uarte.c

@brief	Nordic DFU serial transport over UARTE0, nrfx 4 driver

Takes the place of nrf_dfu_serial_uart.c of the nRF5 SDK. That one goes
through the legacy nrf_drv_uart layer, which does not work with the nrfx 4
UARTE driver: its instance macro gives a null register address, the legacy
configuration is cast to the nrfx configuration although the layouts differ
and nrfx_uarte_rx does not exist anymore.

This file calls the nrfx 4 UARTE driver directly. SLIP framing, buffer pool
and request handling are the same as the SDK transport.

Reception runs without stop on two 1 byte buffers. Each byte goes to the SLIP
decoder as it arrives, so a request is handled as soon as its last byte is in.

Responses are sent blocking, from the scheduler context of the request
handler.

Adapted for IOsonata by Hoang Nguyen Hoan, Oct. 10, 2026

----------------------------------------------------------------------------*/
#include <string.h>

#include "serial_dfu/nrf_dfu_serial.h"
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_dfu_transport.h"
#include "nrf_dfu_req_handler.h"
#include "slip.h"
#include "nrf_balloc.h"
#include "nrfx_uarte.h"

#define NRF_LOG_MODULE_NAME nrf_dfu_serial_uart
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define NRF_SERIAL_OPCODE_SIZE          (sizeof(uint8_t))
#define NRF_UART_MAX_RESPONSE_SIZE_SLIP (2 * NRF_SERIAL_MAX_RESPONSE_SIZE + 1)
#define RX_BUF_SIZE                     (64)	// 64 bytes payload
#define OPCODE_OFFSET                   (sizeof(uint32_t) - NRF_SERIAL_OPCODE_SIZE)
#define DATA_OFFSET                     (OPCODE_OFFSET + NRF_SERIAL_OPCODE_SIZE)
#define UART_SLIP_MTU                   (2 * (RX_BUF_SIZE + 1) + 1)

NRF_BALLOC_DEF(m_payload_pool, (UART_SLIP_MTU + 1), NRF_DFU_SERIAL_UART_RX_BUFFERS);

// The UARTE0 interrupt reaches nrfx_uarte_irq_handler through the nrfx PRS
// box 4 handler, which nrfx_uarte_init registers. The IOsonata nrfx
// configuration enables that box.
static nrfx_uarte_t m_uarte = NRFX_UARTE_INSTANCE(NRF_UARTE0);

// The driver fills one byte while the other one is with the decoder
static uint8_t m_rx_byte[2];
static uint8_t m_rx_idx;

static nrf_dfu_serial_t m_serial;
static slip_t m_slip;
static uint8_t m_rsp_buf[NRF_UART_MAX_RESPONSE_SIZE_SLIP];
static volatile bool m_active;

static nrf_dfu_observer_t m_observer;

static uint32_t uart_dfu_transport_init(nrf_dfu_observer_t observer);
static uint32_t uart_dfu_transport_close(nrf_dfu_transport_t const * p_exception);

DFU_TRANSPORT_REGISTER(nrf_dfu_transport_t const uart_dfu_transport) =
{
	.init_func  = uart_dfu_transport_init,
	.close_func = uart_dfu_transport_close,
};

static void payload_free(void * p_buf)
{
	// The pointer was moved forward to the data, go back to the pool block
	uint8_t * p_buf_root = (uint8_t *)p_buf - DATA_OFFSET;
	nrf_balloc_free(&m_payload_pool, p_buf_root);
}

static ret_code_t rsp_send(uint8_t const * p_data, uint32_t length)
{
	uint32_t slip_len;

	(void)slip_encode(m_rsp_buf, (uint8_t *)p_data, length, &slip_len);

	if (nrfx_uarte_tx(&m_uarte, m_rsp_buf, slip_len, NRFX_UARTE_TX_BLOCKING) != 0)
	{
		return NRF_ERROR_BUSY;
	}

	return NRF_SUCCESS;
}

static void on_rx_byte(nrf_dfu_serial_t * p_transport, uint8_t data)
{
	if (slip_decode_add_byte(&m_slip, data) != NRF_SUCCESS)
	{
		return;
	}

	nrf_dfu_serial_on_packet_received(p_transport,
									  (uint8_t const *)m_slip.p_buffer,
									  m_slip.current_index);

	uint8_t * p_rx_buf = nrf_balloc_alloc(&m_payload_pool);
	if (p_rx_buf == NULL)
	{
		NRF_LOG_ERROR("Failed to allocate buffer");
		return;
	}
	NRF_LOG_INFO("Allocated buffer %x", p_rx_buf);

	// Start decoding the next packet
	m_slip.p_buffer      = &p_rx_buf[OPCODE_OFFSET];
	m_slip.current_index = 0;
	m_slip.state         = SLIP_STATE_DECODING;
}

static void uarte_evt_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
	switch (p_event->type)
	{
		case NRFX_UARTE_EVT_RX_BUF_REQUEST:
			(void)nrfx_uarte_rx_buffer_set(&m_uarte, &m_rx_byte[m_rx_idx], 1);
			m_rx_idx ^= 1;
			break;

		case NRFX_UARTE_EVT_RX_DONE:
			if (p_event->data.rx.length > 0)
			{
				on_rx_byte((nrf_dfu_serial_t *)p_context, p_event->data.rx.p_buffer[0]);
			}
			break;

		case NRFX_UARTE_EVT_RX_DISABLED:
			// The driver stops the receiver when a buffer came too late, start
			// it again unless the transport is being closed
			if (m_active)
			{
				(void)nrfx_uarte_rx_enable(&m_uarte, NRFX_UARTE_RX_ENABLE_CONT);
			}
			break;

		case NRFX_UARTE_EVT_ERROR:
			APP_ERROR_HANDLER(p_event->data.error.error_mask);
			break;

		default:
			break;
	}
}

static uint32_t uart_dfu_transport_init(nrf_dfu_observer_t observer)
{
	uint32_t err_code = NRF_SUCCESS;

	if (m_active)
	{
		return err_code;
	}

	NRF_LOG_DEBUG("serial_dfu_transport_init()");

	m_observer = observer;

	err_code = nrf_balloc_init(&m_payload_pool);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	uint8_t * p_rx_buf = nrf_balloc_alloc(&m_payload_pool);

	m_slip.p_buffer      = &p_rx_buf[OPCODE_OFFSET];
	m_slip.current_index = 0;
	m_slip.buffer_len    = UART_SLIP_MTU;
	m_slip.state         = SLIP_STATE_DECODING;

	m_serial.rsp_func              = rsp_send;
	m_serial.payload_free_func     = payload_free;
	m_serial.mtu                   = UART_SLIP_MTU;
	m_serial.p_rsp_buf             = &m_rsp_buf[NRF_UART_MAX_RESPONSE_SIZE_SLIP -
												NRF_SERIAL_MAX_RESPONSE_SIZE];
	m_serial.p_low_level_transport = &uart_dfu_transport;

	nrfx_uarte_config_t cfg = NRFX_UARTE_DEFAULT_CONFIG(TX_PIN_NUMBER, RX_PIN_NUMBER);

	if (NRF_DFU_SERIAL_UART_USES_HWFC)
	{
		cfg.rts_pin     = RTS_PIN_NUMBER;
		cfg.cts_pin     = CTS_PIN_NUMBER;
		cfg.config.hwfc = NRF_UARTE_HWFC_ENABLED;
	}
	cfg.p_context = &m_serial;

	m_rx_idx = 0;

	if (nrfx_uarte_init(&m_uarte, &cfg, uarte_evt_handler) != 0)
	{
		NRF_LOG_ERROR("Failed initializing uart");
		return NRF_ERROR_INTERNAL;
	}

	m_active = true;

	if (nrfx_uarte_rx_enable(&m_uarte, NRFX_UARTE_RX_ENABLE_CONT) != 0)
	{
		NRF_LOG_ERROR("Failed initializing rx");
		err_code = NRF_ERROR_INTERNAL;
	}

	NRF_LOG_DEBUG("serial_dfu_transport_init() completed");

	if (m_observer)
	{
		m_observer(NRF_DFU_EVT_TRANSPORT_ACTIVATED);
	}

	return err_code;
}

static uint32_t uart_dfu_transport_close(nrf_dfu_transport_t const * p_exception)
{
	if ((m_active == true) && (p_exception != &uart_dfu_transport))
	{
		m_active = false;
		nrfx_uarte_uninit(&m_uarte);
	}

	return NRF_SUCCESS;
}
