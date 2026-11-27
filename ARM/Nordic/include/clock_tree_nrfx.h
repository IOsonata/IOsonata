/* Nordic clock tree configuration used by SystemInit and peripheral drivers. */
#ifndef IOSONATA_NRF_CLOCK_H
#define IOSONATA_NRF_CLOCK_H

#define NRF_PCLK_16M		0
#if defined(NRF5340_XXAA_APPLICATION)
#define NRF_PCLK_192M		1
#endif

#ifdef __cplusplus
extern "C" {
#endif

void nRFxClockInit(void);

#ifdef __cplusplus
}
#endif
#endif
