/* MIT License. Copyright (c) 2026 I-SYST inc.
 * RA4M1 R01UH0887EJ0110, chapters 10, 12 and 28. Private SCI subset.
 * SCI0/1 use non-FIFO mode, so all four channels share the byte-register path.
 */
#ifndef RA4M1_UART_REGS_H
#define RA4M1_UART_REGS_H
#include <stdint.h>
#define RA4M1_SCI_BASE(n) (0x40070000UL + 0x20UL * (n))
#define SCI_SMR  0x00U
#define SCI_BRR  0x01U
#define SCI_SCR  0x02U
#define SCI_TDR  0x03U
#define SCI_SSR  0x04U
#define SCI_RDR  0x05U
#define SCI_SCMR 0x06U
#define SCI_SEMR 0x07U
#define SCI_SNFR 0x08U
#define SCI_SIMR1 0x09U
#define SCI_SPMR 0x0DU
#define SCI_MDDR 0x12U
#define SCI_FCR  0x14U
#define SCI_DCCR 0x13U
#define SCI_SPTR 0x1CU
#define SCI_SCR_TEIE 0x04U
#define SCI_SCR_RE   0x10U
#define SCI_SCR_TE   0x20U
#define SCI_SCR_RIE  0x40U
#define SCI_SCR_TIE  0x80U
#define SCI_SSR_TEND 0x04U
#define SCI_SSR_PER  0x08U
#define SCI_SSR_FER  0x10U
#define SCI_SSR_ORER 0x20U
#define SCI_SSR_RDRF 0x40U
#define SCI_SSR_TDRE 0x80U
#define SCI_SSR_ERRORS (SCI_SSR_PER | SCI_SSR_FER | SCI_SSR_ORER)
#define SCI_SEMR_BRME 0x04U
#define SCI_SEMR_ABCSE 0x08U
#define SCI_SEMR_ABCS 0x10U
#define SCI_SEMR_BGDM 0x40U
#define SCI_SPTR_MARK 0x06U
#define RA4M1_UART_MSTPCRB 0x40047000UL
#endif
