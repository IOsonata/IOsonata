/* MIT License. Copyright (c) 2026 I-SYST inc.
 * Private, pure-integer baud planner, RA4M1 manual section 28.2.17/18.
 */
#ifndef RA4M1_UART_BAUD_H
#define RA4M1_UART_BAUD_H
#include <stdint.h>
#include <stdbool.h>
#include "ra4m1_uart_regs.h"
#ifndef RA4M1_UART_MAX_ERROR_PPM
#define RA4M1_UART_MAX_ERROR_PPM 20000UL
#endif
struct Ra4m1UartBaud {
    uint32_t Actual;
    uint8_t Cks, Brr, Mddr, Semr;
};
static bool Ra4m1UartPlanBaud(uint32_t Clock, uint32_t Rate, Ra4m1UartBaud *Plan)
{
    if (!Plan || !Clock || Clock > 48000000U || !Rate || Rate > Clock / 6U)
        return false;
    static const uint8_t divisors[] = {32, 16, 8, 6};
    static const uint8_t modes[] = {0, SCI_SEMR_BGDM,
        SCI_SEMR_BGDM | SCI_SEMR_ABCS, SCI_SEMR_ABCSE};
    uint64_t best = UINT64_MAX;
    Ra4m1UartBaud result = {};
    /* Evaluate both adjacent BRR divisors for every legal MDDR. Compare in
     * microhertz, not truncated integer Hz or integer 256/M. Ties retain the
     * higher sampling margin and, within a mode, disabled modulation first.
     */
    for (unsigned mode = 0; mode < 4; ++mode) {
        for (unsigned cks = 0; cks < 4; ++cks) {
            uint32_t d = (uint32_t)divisors[mode] << (2U * cks);
            for (unsigned m = 256; m >= 128; --m) {
                uint64_t numerator = (uint64_t)Clock * m;
                uint64_t ideal = numerator / ((uint64_t)Rate * d * 256U);
                for (unsigned adjacent = 0; adjacent < 2; ++adjacent) {
                    uint64_t n = ideal + adjacent;
                    if (!n) n = 1;
                    if (n > 256) n = 256;
                    uint32_t denominator = d * (uint32_t)n * 256U;
                    uint64_t actual = numerator * 1000000U / denominator;
                    uint64_t desired = (uint64_t)Rate * 1000000U;
                    uint64_t error = actual > desired ? actual-desired : desired-actual;
                    if (error >= best) continue;
                    best = error;
                    result.Actual = (uint32_t)((numerator + denominator/2U) / denominator);
                    result.Cks = (uint8_t)cks;
                    result.Brr = (uint8_t)(n-1U);
                    result.Mddr = m == 256 ? 255 : (uint8_t)m;
                    result.Semr = modes[mode] | (m == 256 ? 0 : SCI_SEMR_BRME);
                }
            }
        }
    }
    if (best == UINT64_MAX || best > (uint64_t)Rate * RA4M1_UART_MAX_ERROR_PPM)
        return false;
    *Plan = result;
    return true;
}
#endif
