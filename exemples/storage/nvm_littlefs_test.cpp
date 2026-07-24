/**-------------------------------------------------------------------------
@example	nvm_littlefs_test.cpp

@brief	Host test running littlefs on NvmDiskIO.

		The point of the block adapter is that a filesystem cannot tell what it
		is sitting on. This runs the real littlefs, not a stand in, over two
		media that behave nothing alike underneath: one with an erase step that
		programs by clearing bits, and one that overwrites directly and has no
		erase at all. Both are formatted, mounted, written to, unmounted,
		mounted again and read back, and littlefs sees no difference.

		On a device the same code sits on a SPI NOR flash, an I2C EEPROM or the
		MCU internal memory by handing NvmDiskIO a different Nvm.

Build and run on the host:

  gcc -std=gnu11 -O1 -I littlefs -c littlefs/lfs.c -o lfs.o
  gcc -std=gnu11 -O1 -I littlefs -c littlefs/lfs_util.c -o lfs_util.o
  g++ -std=gnu++23 -O1 -I include -I include/storage -I littlefs -I Linux/include \
	  exemples/storage/nvm_littlefs_test.cpp src/storage/diskio_nvm.cpp \
	  src/storage/diskio_impl.cpp src/device.cpp src/device_intrf.cpp \
	  lfs.o lfs_util.o -o nvm_littlefs_test
  ./nvm_littlefs_test

@author	Hoang Nguyen Hoan
@date	July 24, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "storage/diskio_nvm.h"
extern "C" {
#include "lfs.h"
}

// The pin driver is per architecture; the driver only toggles a protect pin.
extern "C" {
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinSet(int, int) {}
void IOPinClear(int, int) {}
}

#define SIZE  (64u*1024u)
#define BLK   4096u

// Flash-like: erase to 0xFF, programming clears bits only.
class EraseNvm : public Nvm {
public: uint8_t v[SIZE];
	EraseNvm(){ memset(v,0xFF,sizeof(v)); Region(0,sizeof(v)); }
	uint32_t EraseSize() const override { return BLK; }
	uint32_t WriteGran() const override { return 1; }
	int Read(uint64_t o,void*b,uint32_t l) override { if(o+l>SIZE) return -EINVAL; memcpy(b,v+o,l); return l; }
	int Write(uint64_t o,const void*d,uint32_t l) override { if(o+l>SIZE) return -EINVAL;
		const uint8_t*p=(const uint8_t*)d; for(uint32_t i=0;i<l;i++) v[o+i]&=p[i]; return l; }
	int Erase(uint64_t o,uint32_t l) override { if(o+l>SIZE) return -EINVAL;
		if(o%BLK||l%BLK) return -EINVAL; memset(v+o,0xFF,l); return 0; }
};
// EEPROM-like: no erase at all, overwrites directly.
class DirectNvm : public Nvm {
public: uint8_t v[SIZE];
	DirectNvm(){ memset(v,0xFF,sizeof(v)); Region(0,sizeof(v)); }
	uint32_t EraseSize() const override { return 0; }
	uint32_t WriteGran() const override { return 1; }
	int Read(uint64_t o,void*b,uint32_t l) override { if(o+l>SIZE) return -EINVAL; memcpy(b,v+o,l); return l; }
	int Write(uint64_t o,const void*d,uint32_t l) override { if(o+l>SIZE) return -EINVAL;
		memcpy(v+o,d,l); return l; }
};

static NvmDiskIO *s_pDisk;
static int LfsRead(const struct lfs_config*, lfs_block_t b, lfs_off_t o, void *buf, lfs_size_t n)
{ return s_pDisk->Read(b,o,(uint8_t*)buf,n) == (int)n ? LFS_ERR_OK : LFS_ERR_IO; }
static int LfsProg(const struct lfs_config*, lfs_block_t b, lfs_off_t o, const void *buf, lfs_size_t n)
{ return s_pDisk->Write(b,o,(uint8_t*)buf,n) == (int)n ? LFS_ERR_OK : LFS_ERR_IO; }
static int LfsErase(const struct lfs_config*, lfs_block_t b)
{ s_pDisk->EraseSector(b,1); return LFS_ERR_OK; }
static int LfsSync(const struct lfs_config*) { return LFS_ERR_OK; }

static uint8_t s_RdBuf[16], s_PrBuf[16], s_LaBuf[16];
static struct lfs_config s_Cfg = {
	.context = nullptr,
	.read = LfsRead, .prog = LfsProg, .erase = LfsErase, .sync = LfsSync,
	.read_size = 16, .prog_size = 16, .block_size = BLK, .block_count = SIZE/BLK,
	.block_cycles = 500, .cache_size = 16, .lookahead_size = 16,
	.read_buffer = s_RdBuf, .prog_buffer = s_PrBuf, .lookahead_buffer = s_LaBuf,
};
static int fails=0;
#define CK(c,...) do{ if(!(c)){ printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)

static void RunFs(NvmDiskIO &Disk, const char *pWhat)
{
	printf("--- littlefs on %s\n", pWhat);
	s_pDisk = &Disk;
	lfs_t lfs; lfs_file_t f;

	CK(lfs_format(&lfs, &s_Cfg) == 0, "%s: format", pWhat);
	CK(lfs_mount(&lfs, &s_Cfg) == 0, "%s: mount", pWhat);

	const char *msg = "IOsonata NvmDiskIO";
	CK(lfs_file_open(&lfs,&f,"note.txt",LFS_O_WRONLY|LFS_O_CREAT) == 0, "%s: create", pWhat);
	CK(lfs_file_write(&lfs,&f,msg,strlen(msg)) == (int)strlen(msg), "%s: write", pWhat);
	CK(lfs_file_close(&lfs,&f) == 0, "%s: close", pWhat);
	CK(lfs_unmount(&lfs) == 0, "%s: unmount", pWhat);

	// Remount without touching the medium: the file must still be there.
	char back[64]; memset(back,0,sizeof(back));
	CK(lfs_mount(&lfs, &s_Cfg) == 0, "%s: remount", pWhat);
	CK(lfs_file_open(&lfs,&f,"note.txt",LFS_O_RDONLY) == 0, "%s: reopen", pWhat);
	CK(lfs_file_read(&lfs,&f,back,sizeof(back)) == (int)strlen(msg), "%s: read", pWhat);
	CK(strcmp(back,msg) == 0, "%s: contents survived the remount", pWhat);
	lfs_file_close(&lfs,&f);
	CK(lfs_unmount(&lfs) == 0, "%s: unmount again", pWhat);
}

static EraseNvm s_Flash;
static DirectNvm s_Eep;
int main(){
	NvmDiskIO d1, d2;
	CK(d1.Init(s_Flash), "mount the block layer on an erase-write medium");
	CK(d2.Init(s_Eep, BLK), "mount the block layer on a direct write medium");
	RunFs(d1, "erase-write medium");
	RunFs(d2, "direct write medium");
	printf(fails?"\nRESULT: %d FAIL\n":"\nRESULT: ALL PASS\n", fails);
	return fails; }
