/* SPDX-License-Identifier: MIT */
#ifndef TEST_NRF91_MODEM_AT_H
#define TEST_NRF91_MODEM_AT_H
#include <stddef.h>
#ifdef __cplusplus
extern "C"
{
#endif
int nrf_modem_at_printf(const char *pFormat, ...);
int nrf_modem_at_cmd(char *pBuffer, size_t Size, const char *pFormat, ...);
#ifdef __cplusplus
}
#endif
#endif
