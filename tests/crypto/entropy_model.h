#pragma once
#include <stdint.h>
#include <stddef.h>
extern unsigned g_Reads, g_DiscardReads;
extern bool g_Timeout, g_ModuleRunning;
struct EntropyReg {
	unsigned id;
	uint8_t value;
	operator uint8_t() const;
	EntropyReg &operator=(uint8_t v);
};
struct EntropyRegs {
	EntropyReg TRNGSDR{1, 0}, TRNGSCR0{2, 0}, TRNGSCR1{3, 0};
};
extern EntropyRegs g_Trng;
#define R9A02_TRNG (&g_Trng)
#define R9A02_TRNG_EN 8U
#define R9A02_TRNG_ST 4U
#define R9A02_TRNG_RDY 128U
#define R9A02_MSTPCR_C 2U
#define R9A02_MSTPCRC_TRNG (1U << 28)
inline void R9a02ModuleStart(unsigned, uint32_t) {}
inline bool R9a02ModuleRunning(unsigned, uint32_t) { return g_ModuleRunning; }
uint32_t ModelRomRead();
struct OtpApi { uint32_t (*rngRead)(); };
struct RomApi { OtpApi *otpApiBase; };
extern OtpApi g_Otp;
extern RomApi g_Rom;
extern RomApi *g_RomPointer;
#define ROM_API g_RomPointer
#define OTP_API (g_Rom.otpApiBase)
