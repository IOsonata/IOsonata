/**-------------------------------------------------------------------------
@file	isha256.c

@brief	SHA-256 computation

@author	Nguyen Hoan Hoang
@date	Aug. 17, 2014

@license

MIT License

Copyright (c) 2014 I-SYST inc. All rights reserved.

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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "istddef.h"
#include "isha256.h"

/*
 * Test cases
 * Data   : null, zero length
 * SHA256 : e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
 *
 * Data   : "abc"
 * SHA256 : BA7816BF 8F01CFEA 414140DE 5DAE2223 B00361A3 96177A9C B410FF61 F20015AD
 *
 * Data   : "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
 * SHA256 : 248D6A61 D20638B8 E5C02693 0C3E6039 A33CE459 64FF2167 F6ECEDD4 19DB06C1
 *
 * Data   : repeat 'a' 1000000 times
 * SHA256 : cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0
 *
 * Data   : "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
 * SHA256 :  cf5b16a7 78af8380 036ce59e 7b049237 0b249b11 e8f07a51 afac4503 7afee9d1
 *
 */

#define H0	0x6a09e667
#define H1	0xbb67ae85
#define H2	0x3c6ef372
#define H3	0xa54ff53a
#define H4	0x510e527f
#define H5	0x9b05688c
#define H6	0x1f83d9ab
#define H7	0x5be0cd19

static uint32_t g_Sha256KValue[] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 
	0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
	0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
	0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 
	0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 
	0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
	0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
	0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
	0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

inline uint32_t ROTR(uint32_t x, uint32_t n) 
{
    return (x >> n) | (x << (32-n));
}

inline uint32_t ROTL(uint32_t x, uint32_t n) 
{
	return (x << n) | (x >> (32 - n));
}
		
inline uint32_t SUM0(uint32_t x)
{ 
	return ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22);
}

inline uint32_t SUM1(uint32_t x)
{ 
	return ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25);
}

inline uint32_t SIGMA0(uint32_t x)
{ 
	return ROTR(x, 7) ^ ROTR(x, 18) ^ (x >> 3);
}

inline uint32_t SIGMA1(uint32_t x)
{ 
	return ROTR(x, 17) ^ ROTR(x, 19) ^ (x >> 10);
}

inline uint32_t CH(uint32_t x, uint32_t y, uint32_t z)
{ 
	return (x & y) ^ (~x & z); 
}

inline uint32_t MAJ(uint32_t x, uint32_t y, uint32_t z)
{ 
	return (x & y) ^ (x & z) ^ (y & z); 
}

void Sha256Compute(uint32_t *W, uint32_t *H)
{
	uint32_t a, b, c, d, e, f, g, h;

	a = H[0];
	b = H[1];
	c = H[2];
	d = H[3];
	e = H[4];
	f = H[5];
	g = H[6];
	h = H[7];


	for (int t = 0; t < 64; t++)
	{
		uint32_t T1;
		uint32_t T2;

		if (t > 15)
			W[t] = (SIGMA1(W[t-2]) + W[t-7] + SIGMA0(W[t-15]) + W[t-16]) & 0xffffffff;

		T1 = h + SUM1(e) + CH(e, f, g) + g_Sha256KValue[t] + W[t];
		T2 = SUM0(a) + MAJ(a, b, c);

		h = g;
		g = f;
		f = e;
		e = (d + T1);
		d = c;
		c = b;
		b = a;
		a = (T1 + T2);
	}

	H[0] = (H[0] + a);
	H[1] = (H[1] + b);
	H[2] = (H[2] + c);
	H[3] = (H[3] + d);
	H[4] = (H[4] + e);
	H[5] = (H[5] + f);
	H[6] = (H[6] + g);
	H[7] = (H[7] + h);
}

static int g_LastWIdx = 0;
static int g_LastOctet = 0;
static uint64_t g_TotalBitLen = 0;
static char g_Sha256Digest[66] = { 0,};
static uint32_t H[8] = { H0, H1, H2, H3, H4, H5, H6, H7 };
static uint32_t W[64];


/*
 * Generate SHA digest code.  Call this function until all data are processed.
 * set bLast parameter to true for last data packet to process.
 *
 * Make sure to have enough memory for returning results.  pRes must have at
 * least 65 bytes.
 *
 * @param 	pSrc 	: Pointer to source data
 * 			SrcLen	: Source data length in bytes
 *			bLast	: set true to indicate last data packet
 * 			pRes	: Pointer to buffer to store results of 64 characters
 * 					  if NULL is passed, internal buffer will be used
 *
 * 	@return	Pointer to digest string. If pRes is NULL, internal buffer is returned
 * 			NULL if incomplete
 */
char *Sha256(uint8_t *pData, int DataLen, bool bLast, char *pRes)
{
	uint8_t *p = pData;
	int t = 0, j = 0;
	char *digest = g_Sha256Digest;

	g_TotalBitLen += DataLen << 3;

	if (g_LastOctet || g_LastWIdx)
	{
		// We have incomplete buffer from previous call
		t = g_LastWIdx;

		// Fill up left over from previous 32bits
		for (int k = g_LastOctet; k < 4; k++)
		{
			W[t] |= *p << (24 - (k << 3));
			DataLen--;
			p++;
		}
		t++;
		// Fill up th rest of the 512 bits message
		for (; t < 16 && DataLen > 3; t++)
		{
			W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
			p += 4;
			DataLen -= 4;
		}
		if (t >= 16)
		{
			// We have complete 512
			Sha256Compute(W, H);
			memset(W, 0, sizeof(W));
			t = 0; j = 0;
		}
	}

	if (DataLen > 64)
	{
		// Process N complete 512 bits message
		int n = DataLen / 64;
		for (int i = 0; i < n; i++)
		{
			for (t = 0; t < 16; t++)
			{
				W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
				p += 4;
				DataLen -= 4;
			}
			Sha256Compute(W, H);
		}
		t = 0;
		j = 0;
		memset(W, 0, sizeof(W));
	}
	g_LastWIdx = 0;
	g_LastOctet = 0;
	if (DataLen)
	{
		// Still have incompleted data
		for (t = 0; t < 16 && DataLen > 3; t++)
		{
			W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
			p += 4;
			DataLen -= 4;
		}
		for (j = 0; j < DataLen; j++)
		{
			W[t] |= *p << (24 - (j << 3));
			p++;
		}
	}
	if (bLast == false)
	{
		// More data to come, remember where we are
		g_LastWIdx = t;
		g_LastOctet = j;

		return NULL;
	}

	if (bLast)
	{
		// All data processed.  add the 1 bit & data len
		W[t] |= 0x80 << (24 - (j << 3));
		t++;
		if (t > 14)
			Sha256Compute(W, H);
		{
			W[14] = g_TotalBitLen >> 32;
			W[15] = g_TotalBitLen & 0xffffffff;
		}
		Sha256Compute(W, H);

		if (pRes)
			digest = pRes;

		//sprintf(digest, "%08lX%08lX%08lX%08lX%08lX%08lX%08lX%08lX", H[0], H[1], H[2], H[3], H[4], H[5], H[6], H[7]);

		// Reset memory, ready for new processing

		H[0] = H0;
		H[1] = H1;
		H[2] = H2;
		H[3] = H3;
		H[4] = H4;
		H[5] = H5;
		H[6] = H6;
		H[7] = H7;

		memset(W, 0, sizeof(W));
		g_LastWIdx = 0;
		g_LastOctet = 0;
		g_TotalBitLen = 0;
	}

	return digest;
}

