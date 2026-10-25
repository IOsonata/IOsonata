#pragma once
#include <stdint.h>
#include <stddef.h>

// Register model only: AES uses OpenSSL, entropy uses RAND_bytes.
struct SamReg {
	unsigned id;
	uint32_t value;
	operator uint32_t() const;
	SamReg &operator=(uint32_t v);
	SamReg &operator|=(uint32_t v) { return *this = uint32_t(*this) | v; }
	SamReg &operator&=(uint32_t v) { return *this = uint32_t(*this) & v; }
};
struct SamKey { SamReg AESA_KEY; };
struct SamAes {
	SamReg AES_CR, AES_MR, AES_ISR;
	SamReg AES_KEYWR[8], AES_IDATAR[4], AES_ODATAR[4];
	SamReg AESA_CTRL, AESA_MODE, AESA_DRNGSEED, AESA_SR;
	SamKey AESA_KEY[8];
	SamReg AESA_IDATA, AESA_ODATA;
	SamAes();
};
struct SamPm { uint32_t PM_MCCTRL, PM_HSBMASK, PM_PBAMASK, PM_UNLOCK; };
struct SamClock { SamReg SCIF_GCCTRL; };
struct SamScif { SamClock SCIF_GCCTRL[5]; };
struct SamRng { SamReg TRNG_ISR, TRNG_ODATA, TRNG_IDR, TRNG_CR; SamRng(); };
struct SamPmc { uint32_t PMC_PCER1, PMC_PCSR1; };
extern SamAes g_Aes;
extern SamPm g_Pm;
extern SamScif g_Scif;
extern SamRng g_Rng;
extern SamPmc g_Pmc;
extern bool g_AesTimeout, g_RngTimeout;
extern unsigned g_AesBlocks;
#define AES (&g_Aes)
#define SAM4L_AESA (&g_Aes)
#define SAM4L_PM (&g_Pm)
#define SAM4L_SCIF (&g_Scif)
#define SAM4L_TRNG (&g_Rng)
#define PMC (&g_Pmc)
#define ID_AES 39U
#define AES_CR_START 1U
#define AES_CR_SWRST 256U
#define AES_MR_CKEY_PASSWD (14U << 20)
#define AES_MR_CIPHER 1U
#define AES_MR_KEYSIZE_AES256 (2U << 10)
#define AES_MR_KEYSIZE_AES128 0U
#define AES_ISR_DATRDY 1U
#define AES_ISR_URAD 256U
#define AESA_CTRL_ENABLE 1U
#define AESA_CTRL_NEWMSG 4U
#define AESA_CTRL_SWRST 256U
#define AESA_MODE_ENCRYPT 1U
#define AESA_MODE_CTYPE(x) ((x) << 16)
#define AESA_SR_IBUFRDY (1U << 16)
#define AESA_SR_ODATARDY 1U
#define PM_HSBMASK_HTOP0 (1U << 5)
#define PM_HSBMASK_AESA (1U << 9)
#define PM_PBAMASK_TRNG (1U << 17)
#define PM_UNLOCK_KEY(x) ((x) << 24)
#define PM_UNLOCK_ADDR(x) (x)
#define PM_MCCTRL_MCSEL_Msk 7U
#define PM_MCCTRL_MCSEL(x) (x)
#define PM_MCCTRL_MCSEL_PLL0 2U
#define PM_MCCTRL_MCSEL_DFLL0 3U
#define PM_MCCTRL_MCSEL_RC80M 4U
#define SCIF_GCCTRL_CEN 1U
#define SCIF_GCCTRL_DIVEN 2U
#define SCIF_GCCTRL_DIV_Msk (0xFFU << 16)
#define SCIF_GCCTRL_DIV(x) ((x) << 16)
#define SCIF_GCCTRL_OSCSEL_Msk (0x1FU << 8)
#define SCIF_GCCTRL_OSCSEL(x) ((x) << 8)
#define TRNG_CR_KEY(x) ((x) << 8)
#define TRNG_CR_ENABLE 1U
#define TRNG_IDR_DATRDY 1U
#define TRNG_ISR_DATRDY 1U
