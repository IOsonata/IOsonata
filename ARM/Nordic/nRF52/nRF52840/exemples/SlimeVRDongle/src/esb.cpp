/*
 * esb.cpp
 *
 *  Created on: Nov 22, 2024
 *      Author: hoan
 */

#include <memory.h>

#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "sdk_common.h"
#include "nrf_cli.h"

#include "crc.h"
#include "nrf_mac.h"

#include "SlimeVRDongle.h"

void ProcessPairing(nrf_esb_payload_t *pPayload);

/** @brief Macro to create an initializer for a TX data packet.
 *
 * @details This macro generates an initializer. Using the initializer is more efficient
 *          than setting the individual parameters dynamically.
 *
 * @param[in]   _pipe   The pipe to use for the data packet.
 * @param[in]   ...     Comma separated list of character data to put in the TX buffer.
 *                      Supported values consist of 1 to 63 characters.
 *
 * @return  Initializer that sets up the pipe, length, and byte array for content of the TX data.
 */
#define NRF_ESB_CREATE_PAYLOADX(_pipe, ...)                                                  \
        {.length = NUM_VA_ARGS(__VA_ARGS__), .pipe = _pipe, .data = {__VA_ARGS__}};         \
        STATIC_ASSERT(NUM_VA_ARGS(__VA_ARGS__) > 0 && NUM_VA_ARGS(__VA_ARGS__) <= 63)

//static nrf_esb_payload_t rx_payload;

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOADX(0, 0, 0, 0, 0, 0, 0, 0, 0);
static nrf_esb_payload_t tx_payload_pair = NRF_ESB_CREATE_PAYLOADX(0, 0, 0, 0, 0, 0, 0, 0, 0);
static nrf_esb_payload_t tx_payload_timer = NRF_ESB_CREATE_PAYLOADX(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
static nrf_esb_payload_t tx_payload_sync = NRF_ESB_CREATE_PAYLOADX(0, 0, 0, 0, 0);
uint8_t pairing_buf[8] = {0};
static uint8_t discovered_trackers[256] = {0};

static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};
static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};


static bool esb_initialized = false;
static bool esb_paired = false;


void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	nrf_esb_payload_t rxpayload;

	nrf_cli_print(&m_cli_cdc_acm, "nrf_esb_event_handler %d", p_event->evt_id);

    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            while (nrf_esb_read_rx_payload(&rxpayload) == NRF_SUCCESS)
            {
            	nrf_cli_print(&m_cli_cdc_acm, "Rx %d", rxpayload.length);
            	nrf_esb_write_payload(&tx_payload_pair); // Add to TX buffer

                switch (rxpayload.length)
                {
					case 8:	// RX Pairing Packet
						ProcessPairing(&rxpayload);
						break;
					case 16:
						{
							uint8_t imu_id = rxpayload.data[1];
							if (discovered_trackers[imu_id] < DETECTION_THRESHOLD) // garbage filtering of nonexistent tracker
							{
								discovered_trackers[imu_id]++;
								return;
							}
							if (rxpayload.data[0] > 223) // reserved for receiver only
								break;

							//hid_write_packet_n(rxpayload.data, rxpayload.rssi); // write to hid endpoint
						}
						break;
					default:
						;
                }
            }
            nrf_cli_print(&m_cli_cdc_acm, "RX complete");
            break;
    }
    //NRF_GPIO->OUTCLR = 0xFUL << 12;
    //NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}


uint32_t esb_init()
{
	int err_code;

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.tx_output_power			= NRF_ESB_TX_POWER_8DBM;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, sizeof(addr_prefix));
    VERIFY_SUCCESS(err_code);

	esb_initialized = true;

	nrf_esb_flush_tx();
	nrf_esb_flush_rx();

	err_code = nrf_esb_start_rx();

	tx_payload_pair.noack = false;
	tx_payload_pair.length = 8;
	uint64_t addr = nrf_get_mac_address();
	memcpy(&tx_payload_pair.data[2], &addr, 6);
	nrf_cli_print(&m_cli_cdc_acm, "MAC %x%08x", (uint32_t)(addr >> 32UL), (uint32_t)(addr & 0xFFFFFFFFUL));
	nrf_esb_write_payload(&tx_payload_pair);

	return 0;
}

void EsbSetAddr(bool PairMode)
{
	if (PairMode)
	{
		memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
		memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
		memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));

	}
	else
	{
		// Not sure why SlimeNRF does this
		uint64_t addr = nrf_get_mac_address();
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t addr_buffer[16] = {0};
		for (int i = 0; i < 4; i++)
		{
			addr_buffer[i] = buf[i];
			addr_buffer[i + 4] = buf[i] + buf[4];
		}
		for (int i = 0; i < 8; i++)
			addr_buffer[i + 8] = buf[5] + i;
		for (int i = 0; i < 16; i++)
		{
			if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
				addr_buffer[i] += 8;
		}
		memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
		memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
		memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
	}
}

void ProcessPairing(nrf_esb_payload_t *pPayload)
{
	uint64_t addr = nrf_get_mac_address();
	uint64_t found_addr;

	tx_payload_pair.noack = false;
	tx_payload_pair.data[0] = 0; // Invalidate packet
	memcpy(&tx_payload_pair.data[2], &addr, 6);

	uint8_t cs = crc8_ccitt(&pPayload->data[2], 6, 7); // make sure the packet is valid
	if (cs == 0)
		cs = 8;

	if (cs == pPayload->data[0])
	{
		nrf_cli_print(&m_cli_cdc_acm, "Pair address valid");

		memcpy(&found_addr, pPayload->data, 8);

		found_addr >>= 16;

		nrf_cli_print(&m_cli_cdc_acm, "Pair address valid %x%08x", (uint32_t)(found_addr >> 32UL), (uint32_t)(found_addr & 0xFFFFFFFFUL));

		uint16_t send_tracker_id = AddTracker(found_addr); // Use new tracker id

		nrf_cli_print(&m_cli_cdc_acm, "id = %x", send_tracker_id);

		if (send_tracker_id < MAX_TRACKERS)
		{
			tx_payload_pair.data[0] = pPayload->data[0]; // Use checksum sent from device to make sure packet is for that device
			tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
		}
	}

}

