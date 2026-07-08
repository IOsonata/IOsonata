/**-------------------------------------------------------------------------
@example	mic_demo.cpp


@brief	Microphone demo

	This application demo shows how to use digital microphone with
i2s or pdm interface

@author	Hoang Nguyen Hoan
@date	Apr. 25, 2020

@license

Copyright (c) 2020, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

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
#include "nrf.h"

#include "idelay.h"
#include "coredev/i2s.h"
#include "coredev/pdm.h"
#include "nrf_socket.h"
//#include "sys/socket.h"
//#include <modem/at_cmd.h>
//#include <modem/at_notif.h>
//#include <bsd.h>
//#include <bsd_platform.h>
#include "nrf_modem.h"
#include "nrf_modem_os.h"


#include "board.h"

nrf_modem_init_params_t s_ModemParam = {
	{
		.ctrl = {0x0, 0x0},
	}
};

#if 0
static int gnss_ctrl(uint32_t ctrl)
{
	int retval;

	nrf_gnss_fix_retry_t    fix_retry    = 0;
	nrf_gnss_fix_interval_t fix_interval = 1;
	nrf_gnss_delete_mask_t	delete_mask  = 0;
	nrf_gnss_nmea_mask_t	nmea_mask    = NRF_GNSS_NMEA_GSV_MASK |
					       NRF_GNSS_NMEA_GSA_MASK |
					       NRF_GNSS_NMEA_GLL_MASK |
					       NRF_GNSS_NMEA_GGA_MASK |
					       NRF_GNSS_NMEA_RMC_MASK;

	if (ctrl == GNSS_INIT_AND_START) {
		gnss_fd = nrf_socket(NRF_AF_LOCAL,
				     NRF_SOCK_DGRAM,
				     NRF_PROTO_GNSS);

		if (gnss_fd >= 0) {
			printk("GPS Socket created\n");
		} else {
			printk("Could not init socket (err: %d)\n", gnss_fd);
			return -1;
		}

		retval = nrf_setsockopt(gnss_fd,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_FIX_RETRY,
					&fix_retry,
					sizeof(fix_retry));
		if (retval != 0) {
			printk("Failed to set fix retry value\n");
			return -1;
		}

		retval = nrf_setsockopt(gnss_fd,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_FIX_INTERVAL,
					&fix_interval,
					sizeof(fix_interval));
		if (retval != 0) {
			printk("Failed to set fix interval value\n");
			return -1;
		}

		retval = nrf_setsockopt(gnss_fd,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_NMEA_MASK,
					&nmea_mask,
					sizeof(nmea_mask));
		if (retval != 0) {
			printk("Failed to set nmea mask\n");
			return -1;
		}
	}

	if ((ctrl == GNSS_INIT_AND_START) ||
	    (ctrl == GNSS_RESTART)) {
		retval = nrf_setsockopt(gnss_fd,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_START,
					&delete_mask,
					sizeof(delete_mask));
		if (retval != 0) {
			printk("Failed to start GPS\n");
			return -1;
		}
	}

	if (ctrl == GNSS_STOP) {
		retval = nrf_setsockopt(gnss_fd,
					NRF_SOL_GNSS,
					NRF_SO_GNSS_STOP,
					&delete_mask,
					sizeof(delete_mask));
		if (retval != 0) {
			printk("Failed to stop GPS\n");
			return -1;
		}
	}

	return 0;
}
#endif
/*
static void config_regions(bool ram, size_t start, size_t end, uint32_t perm)
{
	const size_t region_size = ram ? RAM_SECURE_ATTRIBUTION_REGION_SIZE
					: FLASH_SECURE_ATTRIBUTION_REGION_SIZE;

	for (size_t i = start; i < end; i++) {
		if (ram) {
			NRF_SPU->RAMREGION[i].PERM = perm;
		} else {
			NRF_SPU->FLASHREGION[i].PERM = perm;
		}
	}

	PRINT("%02u %02u 0x%05x 0x%05x \t", start, end - 1,
				region_size * start, region_size * end);
	PRINT("%s", perm & (ram ? SRAM_SECURE : FLASH_SECURE) ? "Secure\t\t" :
								"Non-Secure\t");
	PRINT("%c", perm & (ram ? SRAM_READ : FLASH_READ)  ? 'r' : '-');
	PRINT("%c", perm & (ram ? SRAM_WRITE : FLASH_WRITE) ? 'w' : '-');
	PRINT("%c", perm & (ram ? SRAM_EXEC : FLASH_EXEC)  ? 'x' : '-');
	PRINT("%c", perm & (ram ? SRAM_LOCK : FLASH_LOCK)  ? 'l' : '-');
	PRINT("\n");
}
*/
void HardwareInit()
{
	NRF_REGULATORS_S->DCDCEN = REGULATORS_DCDCEN_DCDCEN_Enabled;
	NRF_CLOCK_S->LFCLKSRC = CLOCK_LFCLKSRCCOPY_SRC_LFXO;


	NRF_CLOCK_S->TASKS_LFCLKSTART = 1;

	do {

	} while (NRF_CLOCK_S->EVENTS_LFCLKSTARTED == 0);

//	*(volatile uint32_t *)0x40005C04 = 0x02ul;
/*	uint32_t r = NRF_SPU_S->PERIPHID[42].PERM;
	r &= ~(0x13);
	NRF_SPU_S->PERIPHID[42].PERM = r;
	r = NRF_SPU_S->PERIPHID[5].PERM;
	r &= ~(0x13);
	NRF_SPU_S->PERIPHID[5].PERM = r;

	//bsdlid_init(NULL, true);
	NVIC_SetPriority(BSD_NETWORK_IRQ, BSD_NETWORK_IRQ_PRIORITY);

	for (int i = 0; i < 6; i++)
	{
		NRF_SPU_S->RAMREGION[i + 8].PERM = 0x7;
	}
	const bsd_init_params_t init_params = {
		.trace_on = false,
		.bsd_memory_address = BSD_RESERVED_MEMORY_ADDRESS,
		.bsd_memory_size = BSD_RESERVED_MEMORY_SIZE
	};

	int init_ret = bsd_init(&init_params);
*/
//	msDelay(2000);
	uint32_t lte = NRF_POWER_S->POWERSTATUS;//  = POWER_POWERSTATUS_LTEMODEM_ON;
	//int res = bsdlid_init(NULL, true);
	int res = nrf_modem_init(&s_ModemParam, NORMAL_MODE);

	printf("res = %d %x\n", res, res);

	int fd = nrf_socket(NRF_AF_LTE, 0, NRF_PROTO_AT);

	int gnss_fd = nrf_socket(NRF_AF_LOCAL,
			     NRF_SOCK_DGRAM,
			     NRF_PROTO_GNSS);

	if (gnss_fd >= 0) {
		printf("GPS Socket created\n");
	} else {
		printf("Could not init socket (err: %d)\n", gnss_fd);
	}
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	uint32_t d[64];

	HardwareInit();

	while (1)
	{
        //__WFE();
         // Clear the event register.
         //__SEV();
         //__WFE();

		printf("%x\n", d[0]);
		__WFE();
	}

	return 0;
}
