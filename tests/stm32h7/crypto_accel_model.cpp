// Host register model, backed by OpenSSL. This checks driver sequences and
// failure handling, not physical peripheral timing or silicon errata.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include "stm32.h"
#include "crypto_stm32h7.h"
CrypRegs g_Cryp;
HashRegs g_Hash;
RccRegs g_Rcc;
static uint32_t regs[REG_COUNT];
static std::vector<uint8_t> input, output, message;
static bool prepared;
static unsigned fault, aesBlocks, hashRuns;
static CryptoStm32H7Aes *reenter;
static uint8_t key[16], block[16], dest[16];
static CryptoKey testKey{CRYPTO_KEY_AES_128,CRYPTO_KEY_LOC_PLAIN,
	CRYPTO_KEY_USE_ENCRYPT|CRYPTO_KEY_USE_DECRYPT,{{key,16}}};
static void bytes(std::vector<uint8_t>&v,uint32_t w){for(int i=3;i>=0;i--)v.push_back(w>>(8*i));}
uint32_t ModelRead(unsigned id)
{
	if(id==C_SR){
	 if(fault==1)return 0;
	 if(fault==3)return CRYP_SR_BUSY;
	 return CRYP_SR_IFNF | ((output.empty()||fault==2)?0:CRYP_SR_OFNE);
	}
	if(id==C_DOUT){assert(output.size()>=4);uint32_t w=0;
	 for(int i=0;i<4;i++){w=(w<<8)|output.front();output.erase(output.begin());}return w;}
	if(id==H_SR){if(fault==4)return HASH_SR_BUSY;if(fault==5)return 0;}
	return regs[id];
}
void ModelWrite(unsigned id,uint32_t value)
{
	regs[id]=value;
	if(id==R_RST){
	 if(value&RCC_AHB2RSTR_CRYPRST){for(unsigned i=0;i<=C_K3R;i++)regs[i]=0;
	  prepared=false;input.clear();output.clear();}
	 if(value&RCC_AHB2RSTR_HASHRST){regs[H_CR]=regs[H_SR]=regs[H_STR]=0;message.clear();}
	}
	if(id==C_CR){
	 if(value&CRYP_CR_FFLUSH){input.clear();output.clear();}
	 if((value&CRYP_CR_ALGOMODE)==CRYP_CR_ALGOMODE_AES_KEY)prepared=true;
	}
	if(id==C_DIN){
	 assert(regs[R_EN]&RCC_AHB2ENR_CRYPEN);
	 assert((regs[C_CR]&CRYP_CR_ALGOMODE)==CRYP_CR_ALGOMODE_AES_ECB);
	 assert(regs[C_CR]&CRYP_CR_CRYPEN);
	 assert((regs[C_CR]&(CRYP_CR_DATATYPE|CRYP_CR_KEYSIZE))==0);
	 if(reenter){auto *p=reenter;reenter=nullptr;uint8_t out[16];memset(out,0xA5,16);
	  assert(p->Cipher(CRYPTO_CIPHER_ECB,1,testKey,nullptr,0,block,16,out)==CRYPTO_STATUS_BUSY);
	  assert(P256IsZero(out,16));assert(!p->Enable());}
	 bytes(input,value);
	 if(input.size()==16){
	  std::vector<uint8_t> k;for(unsigned i=C_K2L;i<=C_K3R;i++)bytes(k,regs[i]);
	  bool decrypt=regs[C_CR]&CRYP_CR_ALGODIR;if(decrypt)assert(prepared);
	  EVP_CIPHER_CTX *ctx=EVP_CIPHER_CTX_new();assert(ctx);
	  assert(EVP_CipherInit_ex(ctx,EVP_aes_128_ecb(),nullptr,k.data(),nullptr,!decrypt));
	  EVP_CIPHER_CTX_set_padding(ctx,0);unsigned char out[32];int len;
	  assert(EVP_CipherUpdate(ctx,out,&len,input.data(),16));assert(len==16);
	  output.assign(out,out+16);EVP_CIPHER_CTX_free(ctx);input.clear();aesBlocks++;
	 }
	}
	if(id==H_CR&&(value&HASH_CR_INIT)){
	 assert(regs[R_EN]&RCC_AHB2ENR_HASHEN);
	 assert((value&HASH_CR_ALGO)==HASH_CR_ALGO);
	 assert((value&HASH_CR_DATATYPE)==HASH_CR_DATATYPE_1);
	 message.clear();regs[H_SR]=0;
	}
	if(id==H_DIN)for(unsigned i=0;i<4;i++)message.push_back(value>>(i*8));
	if(id==H_STR&&(value&HASH_STR_DCAL)){
	 unsigned bits=value&HASH_STR_NBLW;assert(bits%8==0);
	 if(bits)message.resize(message.size()-(4-bits/8));
	 unsigned char digest[32];SHA256(message.data(),message.size(),digest);
	 for(unsigned i=0;i<8;i++){uint32_t w=0;for(unsigned j=0;j<4;j++)w=(w<<8)|digest[4*i+j];regs[H_HR+i]=w;}
	 regs[H_SR]=HASH_SR_DCIS;hashRuns++;
	}
}
int CryptoEngineTest();
int main()
{
	int result=CryptoEngineTest();assert(result==0);
	for(unsigned i=0;i<16;i++)key[i]=i+1;
	CryptoStm32H7Aes a,b;CryptoStm32H7Hash h;assert(a.Enable()&&b.Enable()&&h.Enable());
	// Check actual hardware Cipher's in-place path, not just the software base.
	CryptoSoftAes ref;ref.Enable();uint8_t buf[66], expected[64], iv[16]={};
	for(auto alg:{CRYPTO_CIPHER_ECB,CRYPTO_CIPHER_CBC,CRYPTO_CIPHER_CTR}){
		for(int enc=0;enc<=1;enc++){
			for(unsigned i=0;i<66;i++)buf[i]=i;
			assert(ref.Cipher(alg,enc,testKey,iv,16,buf+1,64,expected)==CRYPTO_STATUS_OK);
			assert(a.Cipher(alg,enc,testKey,iv,16,buf+1,64,buf+1)==CRYPTO_STATUS_OK);
			assert(!memcmp(buf+1,expected,64)&&buf[0]==0&&buf[65]==65);
		}
	}
	reenter=&b;assert(a.Cipher(CRYPTO_CIPHER_ECB,1,testKey,nullptr,0,block,16,dest)==CRYPTO_STATUS_OK);
	for(unsigned f=1;f<=3;f++){
	 fault=f;memset(dest,0xA5,16);
	 assert(a.Cipher(CRYPTO_CIPHER_ECB,f==3?0:1,testKey,nullptr,0,block,16,dest)==CRYPTO_STATUS_FAIL);
	 assert(P256IsZero(dest,16));assert(regs[C_CR]==0);for(unsigned i=C_K2L;i<=C_K3R;i++)assert(regs[i]==0);
	 fault=0;assert(a.SelfTest()==0);
	}
	uint8_t digest[32];for(unsigned f=4;f<=5;f++){
	 fault=f;memset(digest,0xA5,32);
	 assert(h.Hash(CRYPTO_HASH_SHA256,block,16,digest)==CRYPTO_STATUS_FAIL);
	 assert(P256IsZero(digest,32));assert(regs[H_CR]==0);
	 fault=0;assert(h.SelfTest()==0);
	}
	a.Disable();assert(b.SelfTest()==0);
	printf("Register model: %u AES blocks, %u hashes; timeout recovery, wiping and cross-instance locking PASS\n",aesBlocks,hashRuns);
	return result;
}
