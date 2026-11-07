#ifndef NRF_SOC_H_
#define NRF_SOC_H_
#include <stdint.h>
#define NRF_SOC_APP_PPI_CHANNELS_SD_ENABLED_MSK 0x0001FFFFUL
uint32_t sd_ppi_channel_assign(uint8_t ch, const volatile void *evt, const volatile void *task);
uint32_t sd_ppi_channel_enable_set(uint32_t mask);
uint32_t sd_ppi_channel_enable_clr(uint32_t mask);
#endif
