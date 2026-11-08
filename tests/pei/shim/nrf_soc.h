#ifndef NRF_SOC_H_
#define NRF_SOC_H_
#include <stdint.h>
// S140 v7+ exposes the reserved mask, not APP_PPI_CHANNELS.
#define NRF_SOC_SD_PPI_CHANNELS_SD_ENABLED_MSK 0xFFFE0000UL
#define NRF_SOC_SD_PPI_GROUPS_SD_ENABLED_MSK 0x30UL
#ifdef __cplusplus
extern "C" {
#endif
uint32_t sd_ppi_channel_assign(uint8_t ch, const volatile void *evt, const volatile void *task);
uint32_t sd_ppi_channel_enable_set(uint32_t mask);
uint32_t sd_ppi_channel_enable_clr(uint32_t mask);
uint32_t sd_ppi_group_assign(uint8_t group, uint32_t mask);
#ifdef __cplusplus
}
#endif
#endif
