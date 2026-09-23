// Run the stage 0 boot on a memory image made by Python/dfu_prod_hex.py for
// a layout without slot 1: the payload in slot 0, the record after it. The
// boot must verify it with the key and start it.
//
// Usage: dfu_prod_boot <image dir> <flat dump from HOST_FLASH_BASE>

#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		return 2;
	}

	HostFlashInit(HOST_MEM_NOR);
	std::vector<uint8_t> dump = HostReadFile(argv[2]);
	memcpy(HostFlash(HOST_FLASH_BASE), dump.data(), dump.size());

	std::vector<uint8_t> key = HostReadFile(std::string(argv[1]) + "/key1.der");
	DfuImgKey_t k = { key.data(), (uint32_t)key.size() };
	DfuBootCfg_t boot = {
		.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem)),
		.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr),
		.pKey = &k,
		.NbKey = 1,
		.bVerifySlot0 = true,
		.bAllowNoRec = false,
	};

	if (setjmp(g_HostStartJmp) != 0)
	{
		printf("started 0x%lx\n", (unsigned long)g_HostStartAddr);
		return g_HostStartAddr == HOST_SLOT0 ? 0 : 1;
	}
	printf("not started: %d\n", DfuBootRun(boot));

	return 1;
}
