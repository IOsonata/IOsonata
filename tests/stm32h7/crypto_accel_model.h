#pragma once
#include <cstdint>
enum { C_CR, C_SR, C_DIN, C_DOUT, C_K2L, C_K2R, C_K3L, C_K3R,
	H_CR, H_SR, H_DIN, H_STR, R_EN, R_RST, H_HR=16, REG_COUNT=32 };
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
	K2LR{C_K2L}, K2RR{C_K2R}, K3LR{C_K3L}, K3RR{C_K3R};};
struct HashRegs {ModelReg CR{H_CR}, SR{H_SR}, DIN{H_DIN}, STR{H_STR};
	ModelReg HR[8]={{16},{17},{18},{19},{20},{21},{22},{23}};};
struct RccRegs {ModelReg AHB2ENR{R_EN}, AHB2RSTR{R_RST};};
extern CrypRegs g_Cryp;
extern HashRegs g_Hash;
extern RccRegs g_Rcc;
#define CRYP (&g_Cryp)
#define HASH (&g_Hash)
#define HASH_DIGEST (&g_Hash)
#define RCC (&g_Rcc)
