// Minimal runtime for the 32-bit ATT storage test, executed by Unicorn.
#include <stddef.h>
#include <stdarg.h>

extern "C" {

unsigned g_ServiceStorage[4];

void *memset(void *pDst, int Value, size_t Len)
{
	unsigned char *p = (unsigned char *)pDst;

	while (Len--)
	{
		*p++ = Value;
	}
	return pDst;
}

void *memcpy(void *pDst, const void *pSrc, size_t Len)
{
	unsigned char *p = (unsigned char *)pDst;
	const unsigned char *s = (const unsigned char *)pSrc;

	while (Len--)
	{
		*p++ = *s++;
	}
	return pDst;
}

int memcmp(const void *pA, const void *pB, size_t Len)
{
	const unsigned char *a = (const unsigned char *)pA;
	const unsigned char *b = (const unsigned char *)pB;

	while (Len--)
	{
		if (*a != *b)
		{
			return *a - *b;
		}
		a++;
		b++;
	}
	return 0;
}

size_t strlen(const char *pText)
{
	size_t len = 0;

	while (pText[len])
	{
		len++;
	}
	return len;
}

int puts(const char *)
{
	return 0;
}

// The service fixture prints four unsigned values: count, used, tail, pool.
int printf(const char *pFormat, ...)
{
	va_list args;
	unsigned i;

	va_start(args, pFormat);
	for (i = 0; i < 4; i++)
	{
		g_ServiceStorage[i] = va_arg(args, unsigned);
	}
	va_end(args);
	return 0;
}
}

asm(".global _start\n_start:\n"
	"andl $-16,%esp\n"
	"call main\n"
	"movl %eax,%ebx\n"
	"movl $1,%eax\n"
	"int $0x80\n");
