#pragma once
#include <cstdint>
enum { C_CR, C_SR, C_DIN, C_DOUT, C_K0L, C_K0R, C_K1L, C_K1R, C_K2L, C_K2R, C_K3L, C_K3R,
	H_CR, H_SR, H_DIN, H_STR, R_EN, R_RST, H_HR=24, L_CR=32, L_SR, L_DIN, L_DOUT, L_K0, L_K1, L_K2, L_K3, L_K4, L_K5, L_K6, L_K7, REG_COUNT=44 };
uint32_t ModelRead(unsigned id);
void ModelWrite(unsigned id, uint32_t value);
struct ModelReg {
	unsigned id;
	operator uint32_t() const {return ModelRead(id);}
	ModelReg &operator=(uint32_t v) {ModelWrite(id,v);return *this;}
	ModelReg &operator|=(uint32_t v) {return *this=ModelRead(id)|v;}
	ModelReg &operator&=(uint32_t v) {return *this=ModelRead(id)&v;}
};
struct CrypRegs {ModelReg CR{C_CR}, SR{C_SR}, DIN{C_DIN}, DOUT{C_DOUT},
	K0LR{C_K0L}, K0RR{C_K0R}, K1LR{C_K1L}, K1RR{C_K1R},
	K2LR{C_K2L}, K2RR{C_K2R}, K3LR{C_K3L}, K3RR{C_K3R};};
struct HashRegs {ModelReg CR{H_CR}, SR{H_SR}, DIN{H_DIN}, STR{H_STR};
	ModelReg HR[8]={{24},{25},{26},{27},{28},{29},{30},{31}};};
struct RccRegs {ModelReg AHB2ENR{R_EN}, AHB2RSTR{R_RST};};
extern CrypRegs g_Cryp;
extern HashRegs g_Hash;
extern RccRegs g_Rcc;
#define CRYP (&g_Cryp)
#define HASH (&g_Hash)
#define HASH_DIGEST (&g_Hash)
#define RCC (&g_Rcc)

#ifdef STM32L4S9xx
struct AesRegs { ModelReg CR{L_CR}, SR{L_SR}, DINR{L_DIN}, DOUTR{L_DOUT},
	KEYR0{L_K0}, KEYR1{L_K1}, KEYR2{L_K2}, KEYR3{L_K3},
	KEYR4{L_K4}, KEYR5{L_K5}, KEYR6{L_K6}, KEYR7{L_K7}; };
extern AesRegs g_Aes;
#define AES (&g_Aes)
#endif
