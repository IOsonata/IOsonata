//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2024, I-SYST inc. All rights reserved.
// Description : Hello World in C++
//============================================================================

#include "nrf_esb.h"
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util.h"

#include "crc.h"
#include "board.h"


#define RESET_MEMORY_TEST_BYTE  (0x0DUL)        /**< Known sequence written to a special register to check if this wake up is from System OFF. */
#define RAM_RETENTION_OFF       (0x00000003UL)  /**< The flag used to turn off RAM retention on nRF52. */

#define BTN_PRESSED     0                       /**< Value of a pressed button. */
#define BTN_RELEASED    1                       /**< Value of a released button. */

//#define NRF_ESB_LEGACY

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

#define NRF_ESB_CREATE_PAYLOAD1(_pipe, ...)                                                  \
        {.length = NUM_VA_ARGS(__VA_ARGS__), .pipe = _pipe, .data = {__VA_ARGS__}};         \
        STATIC_ASSERT(NUM_VA_ARGS(__VA_ARGS__) > 0 && NUM_VA_ARGS(__VA_ARGS__) <= 63)

static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD1(0, 0x01, 0x00);
static nrf_esb_payload_t tx_payload_pair = NRF_ESB_CREATE_PAYLOAD1(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

static nrf_esb_payload_t rx_payload;

static uint8_t paired_addr[8] = {0};
//static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};
static bool esb_paired = false;

//static uint32_t button_state_1;
//static uint32_t button_state_2;
//static uint32_t button_state_3;
//static uint32_t button_state_4;
static volatile bool esb_completed = false;
static uint8_t s_TrackerId;

void system_off( void )
{
#ifdef NRF51
    NRF_POWER->RAMON |= (POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos) |
                        (POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos);
#endif //NRF51
#ifdef NRF52
    NRF_POWER->RAM[0].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[1].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[2].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[3].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[4].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[5].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[6].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[7].POWER = RAM_RETENTION_OFF;
#endif //NRF52

    // Turn off LEDs before sleeping to conserve energy.
    //bsp_board_leds_off();

    // Set nRF5 into System OFF. Reading out value and looping after setting the register
    // to guarantee System OFF in nRF52.
    NRF_POWER->SYSTEMOFF = 0x1;
    (void) NRF_POWER->SYSTEMOFF;
    while (true);
}


void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            // Get the most recent element from the RX FIFO.
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS) ;

			if (!paired_addr[0]) // zero, not paired
			{
				if (rx_payload.length == 8)
					memcpy(paired_addr, rx_payload.data, sizeof(paired_addr));
			}
			else
			{
				if (rx_payload.length == 4)
				{
					// TODO: Device should never receive packets if it is already paired, why is this packet received?
					// This may be part of acknowledge
/*					if (!nrfx_timer_init_check(&m_timer))
					{
						LOG_WRN("Timer not initialized");
						break;
					}
					if (timer_state == false)
					{
						nrfx_timer_resume(&m_timer);
						timer_state = true;
					}
					nrfx_timer_clear(&m_timer);
					last_reset = 0;
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					LOG_DBG("RX, timer reset");*/
				}
			}            break;
    }

    esb_completed = true;
}


void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
    uint8_t base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
    uint8_t addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

#ifndef NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
#else // NRF_ESB_LEGACY
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_LEGACY_CONFIG;
#endif // NRF_ESB_LEGACY
    nrf_esb_config.retransmit_count         = 6;
    nrf_esb_config.selective_auto_ack       = false;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    tx_payload.length  = 3;
    tx_payload.pipe    = 0;
    tx_payload.data[0] = 0x00;

    return NRF_SUCCESS;
}


uint32_t gpio_check_and_esb_tx()
{
    uint32_t err_code;
    /*
    button_state_1 = nrf_gpio_pin_read(BUTTON_1);
    button_state_2 = nrf_gpio_pin_read(BUTTON_2);
    button_state_3 = nrf_gpio_pin_read(BUTTON_3);
    button_state_4 = nrf_gpio_pin_read(BUTTON_4);
    if (button_state_1 == BTN_PRESSED)
    {
        tx_payload.data[0] |= 1 << 0;
    }
    if (button_state_2 == BTN_PRESSED)
    {
        tx_payload.data[0] |= 1 << 1;
    }
    if (button_state_3 == BTN_PRESSED)
    {
        tx_payload.data[0] |= 1 << 2;
    }
    if (button_state_4 == BTN_PRESSED)
    {
        tx_payload.data[0] |= 1 << 3;
    }
    if (button_state_1 + button_state_2 + button_state_3 + button_state_4 == BTN_PRESSED + 3 * BTN_RELEASED)
    {
        tx_payload.noack = false;
        err_code = nrf_esb_write_payload(&tx_payload);
        VERIFY_SUCCESS(err_code);
    }
    else
    {
        esb_completed = true;
    }
*/
    return NRF_SUCCESS;
}


void gpio_init( void )
{
    //nrf_gpio_cfg_sense_input(BUTTON_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //nrf_gpio_cfg_sense_input(BUTTON_2, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //nrf_gpio_cfg_sense_input(BUTTON_3, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //nrf_gpio_cfg_sense_input(BUTTON_4, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

    // Workaround for PAN_028 rev1.1 anomaly 22 - System: Issues with disable System OFF mechanism
    nrf_delay_ms(1);

   // bsp_board_init(BSP_INIT_LEDS);
}


void recover_state()
{
    uint32_t            loop_count = 0;
    if ((NRF_POWER->GPREGRET >> 4) == RESET_MEMORY_TEST_BYTE)
    {
        // Take the loop_count value.
        loop_count          = (uint8_t)(NRF_POWER->GPREGRET & 0xFUL);
        NRF_POWER->GPREGRET = 0;
    }

    loop_count++;
    NRF_POWER->GPREGRET = ( (RESET_MEMORY_TEST_BYTE << 4) | loop_count);

    tx_payload.data[1] = loop_count << 4;
}

inline void esb_set_addr_paired(void)
{
	// Recreate receiver address
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = paired_addr[i + 2];
		addr_buffer[i + 4] = paired_addr[i + 2] + paired_addr[6];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = paired_addr[7] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

void esb_pair()
{
	if (!paired_addr[0]) // zero, no receiver paired
	{
		tx_payload_pair.noack = false;
		uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
		memcpy(&tx_payload_pair.data[2], addr, 6);

		uint8_t checksum = crc8_ccitt(&tx_payload_pair.data[2], 6, 0x07);
		if (checksum == 0)
			checksum = 8;
//		LOG_INF("Checksum: %02X", checksum);
		tx_payload_pair.data[0] = checksum; // Use checksum to make sure packet is for this device
//		set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
		while (paired_addr[0] != checksum)
		{
			if (paired_addr[0])
			{
				//LOG_INF("Incorrect checksum: %02X", paired_addr[0]);
				paired_addr[0] = 0; // Packet not for this device
			}
			nrf_esb_flush_rx();
			nrf_esb_flush_tx();
			nrf_esb_write_payload(&tx_payload_pair); // TODO: Does this still fail after a while?
			nrf_esb_start_tx();
			//k_msleep(1000);
		}
	}

	s_TrackerId = paired_addr[1];

	esb_set_addr_paired();
	esb_paired = true;
}

void HardwareInit()
{

}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{
    uint32_t err_code;
    // Initialize
    clocks_start();

    HardwareInit();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    esb_pair();

    gpio_init();

    // Recover state if the device was woken from System OFF.
    recover_state();

    // Check state of all buttons and send an esb packet with the button press if there is exactly one.
    err_code = gpio_check_and_esb_tx();
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        // Wait for esb completed and all buttons released before going to system off.
        if (esb_completed)
        {
            //if (nrf_gpio_pin_read(BUTTON_1) == BTN_RELEASED &&
            //    nrf_gpio_pin_read(BUTTON_2) == BTN_RELEASED &&
            //    nrf_gpio_pin_read(BUTTON_3) == BTN_RELEASED &&
            //    nrf_gpio_pin_read(BUTTON_4) == BTN_RELEASED
            //   )
            {
            //    system_off();
            }
        }
    }
}
