// Randomized equivalence check: memcpy_fast vs memcpy, every size 0..512 at
// every source/destination alignment 0..7, plus guard bytes either side.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "memcpy_fast.h"

#define GUARD		16
#define MAXLEN		512
#define ALIGNS		8

static uint8_t s_Src[MAXLEN + ALIGNS + 2 * GUARD] __attribute__((aligned(16)));
static uint8_t s_Dst[MAXLEN + ALIGNS + 2 * GUARD] __attribute__((aligned(16)));
static uint8_t s_Ref[MAXLEN + ALIGNS + 2 * GUARD] __attribute__((aligned(16)));

static int s_Fail;

static void One(size_t len, unsigned da, unsigned sa, bool useInline)
{
	for (size_t i = 0; i < sizeof(s_Src); i++)
	{
		s_Src[i] = (uint8_t)(rand() & 0xFF);
	}
	memset(s_Dst, 0xA5, sizeof(s_Dst));
	memcpy(s_Ref, s_Dst, sizeof(s_Ref));

	uint8_t *d = s_Dst + GUARD + da;
	uint8_t *r = s_Ref + GUARD + da;
	const uint8_t *s = s_Src + GUARD + sa;

	void *ret = useInline ? memcpy_fast(d, s, len) : memcpy_fast_bulk(d, s, len);
	memcpy(r, s, len);

	if (ret != d)
	{
		printf("FAIL return len=%zu da=%u sa=%u inline=%d\n", len, da, sa, useInline);
		s_Fail++;
		return;
	}
	if (memcmp(s_Dst, s_Ref, sizeof(s_Dst)) != 0)
	{
		printf("FAIL content len=%zu da=%u sa=%u inline=%d\n", len, da, sa, useInline);
		s_Fail++;
	}
}

int main(void)
{
	srand(12345);
	long cases = 0;

	for (size_t len = 0; len <= MAXLEN; len++)
	{
		for (unsigned da = 0; da < ALIGNS; da++)
		{
			for (unsigned sa = 0; sa < ALIGNS; sa++)
			{
				One(len, da, sa, true);
				One(len, da, sa, false);
				cases += 2;
			}
		}
	}

	// Null and zero handling.
	uint8_t b[4] = { 1, 2, 3, 4 };
	if (memcpy_fast(b, b, 0) != b) { printf("FAIL zero size\n"); s_Fail++; }
	if (memcpy_fast_bulk(NULL, b, 4) != NULL) { printf("FAIL null dst\n"); s_Fail++; }
	if (memcpy_fast_bulk(b, NULL, 4) != b) { printf("FAIL null src\n"); s_Fail++; }
	if (b[0] != 1 || b[3] != 4) { printf("FAIL null src wrote\n"); s_Fail++; }

	printf("%ld cases, %d failures\n", cases, s_Fail);
	return s_Fail != 0;
}
