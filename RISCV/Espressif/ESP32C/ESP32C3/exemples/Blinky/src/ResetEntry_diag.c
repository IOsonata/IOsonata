/**-------------------------------------------------------------------------
@file	ResetEntry.c

@brief	*** TEMPORARY DIAGNOSTIC VERSION — NOT FOR PRODUCTION ***

	Stripped-down ResetEntry whose only job is to prove Direct Boot
	on the ESP32-C3 with all four watchdogs correctly disabled.

	Direct Boot is now confirmed working (ROM detects magic 0xaedb041d
	at flash offset 0..7 and hands off to flash + 8).  The remaining
	failure was the WDT-disable sequence: register offsets, SWD key,
	and SWD bit position all wrong in the previous version.  This
	revision uses the offsets/keys from the ESP-IDF C3 register file
	(verified against esp-idf-c3 commit-of-record).

	*** Corrections vs. the previous diagnostic and against
	     production system_esp32_system.c: ***

	   RTC CNTL register offsets (+ from base 0x60008000):
	     WDTCONFIG0:    0x90  (was 0x94 — wrong by 4 bytes)
	     WDTFEED:       0xA4  (was 0x9C — off by 8)
	     WDTWPROTECT:   0xA8  (was 0xA4 — off by 4)
	     SWD_CONF:      0xAC  (was 0xB0 — off by 4)
	     SWD_WPROTECT:  0xB0  (was 0xBC — off by 12)

	   Super WDT has its OWN write-protect key, different from RTC WDT:
	     SWD_WKEY:  0x8F1D312A  (was 0x50D83AA1)
	     RTC_WDT_WKEY: 0x50D83AA1 (unchanged, this one was right)

	   Super WDT disable is bit 30, not bit 31:
	     bit 31 = SWD_AUTO_FEED_EN (the OPPOSITE of disable)
	     bit 30 = SWD_DISABLE  ← what we want

	TIMG0 / TIMG1 offsets and the TIMG WDT key (0x50D83AA1) are
	correct as they were and unchanged here.

----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

extern uint32_t __StackTop;
extern unsigned long __global_pointer$;
extern unsigned long __heap_start__;
extern unsigned long __heap_end__;

__attribute__((naked, section(".reset"), used, retain, noreturn))
void ResetEntry(void)
{
	__asm volatile (
		/*===================================================================
		 * 1. Init sp / gp.
		 *===================================================================*/
		"la   sp, __StackTop                       \n\t"
		"la   gp, __global_pointer$                \n\t"

		/*===================================================================
		 * 2. Disable WDTs.
		 *
		 * Sequence per WDT:  unlock (write magic to WPROTECT) → feed (only
		 * if FEED reg exists for this WDT) → write CONFIG to disable →
		 * re-lock (write 0 to WPROTECT).
		 *
		 * RTC WDT WKEY  = 0x50D83AA1
		 * TIMG WDT WKEY = 0x50D83AA1
		 * SWD WKEY      = 0x8F1D312A   (different!)
		 *===================================================================*/

		/* --- TIMG0 WDT (base 0x6001_F000) --- */
		"li   t0, 0x6001F064                       \n\t"   /* WPROTECT (+0x64) */
		"li   t1, 0x50D83AA1                       \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x6001F060                       \n\t"   /* FEED     (+0x60) */
		"li   t1, 1                                \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x6001F048                       \n\t"   /* CONFIG0  (+0x48) */
		"sw   zero, 0(t0)                          \n\t"
		"li   t0, 0x6001F064                       \n\t"   /* re-lock          */
		"sw   zero, 0(t0)                          \n\t"

		/* --- TIMG1 WDT (base 0x6002_0000) --- */
		"li   t0, 0x60020064                       \n\t"
		"li   t1, 0x50D83AA1                       \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x60020060                       \n\t"
		"li   t1, 1                                \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x60020048                       \n\t"
		"sw   zero, 0(t0)                          \n\t"
		"li   t0, 0x60020064                       \n\t"
		"sw   zero, 0(t0)                          \n\t"

		/* --- RTC WDT (base 0x6000_8000) --- CORRECTED OFFSETS --- */
		"li   t0, 0x600080A8                       \n\t"   /* WPROTECT (+0xA8) */
		"li   t1, 0x50D83AA1                       \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x600080A4                       \n\t"   /* FEED     (+0xA4) */
		"li   t1, 1                                \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x60008090                       \n\t"   /* CONFIG0  (+0x90) */
		"sw   zero, 0(t0)                          \n\t"
		"li   t0, 0x600080A8                       \n\t"   /* re-lock          */
		"sw   zero, 0(t0)                          \n\t"

		/* --- Super WDT  --- CORRECTED KEY, OFFSETS, BIT --- */
		"li   t0, 0x600080B0                       \n\t"   /* SWD_WPROTECT (+0xB0) */
		"li   t1, 0x8F1D312A                       \n\t"   /* SWD_WKEY (different!) */
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x600080AC                       \n\t"   /* SWD_CONF (+0xAC) */
		"lw   t1, 0(t0)                            \n\t"
		"li   t2, 0x40000000                       \n\t"   /* bit 30 = SWD_DISABLE */
		"or   t1, t1, t2                           \n\t"
		"sw   t1, 0(t0)                            \n\t"
		"li   t0, 0x600080B0                       \n\t"   /* re-lock              */
		"sw   zero, 0(t0)                          \n\t"

		/*===================================================================
		 * 3. IO_MUX[GPIO 8] = MCU_SEL=1 (PERIPHS_IO_MUX_GPIO8_U @ 0x60009024).
		 *===================================================================*/
		"li   t0, 0x60009024                       \n\t"
		"li   t1, 0x00001000                       \n\t"
		"sw   t1, 0(t0)                            \n\t"

		/*===================================================================
		 * 4. Enable GPIO 8 as output.
		 *===================================================================*/
		"li   t0, 0x60004024                       \n\t"
		"li   t1, 0x00000100                       \n\t"
		"sw   t1, 0(t0)                            \n\t"

		/*===================================================================
		 * 5. Toggle GPIO 8 forever.
		 *===================================================================*/
		"li   t0, 0x60004008                       \n\t"   /* OUT_W1TS */
		"li   t1, 0x6000400C                       \n\t"   /* OUT_W1TC */
		"li   t2, 0x00000100                       \n\t"   /* mask     */

		"1:                                        \n\t"
		"sw   t2, 0(t0)                            \n\t"
		"li   t3, 800000                           \n\t"
		"2:                                        \n\t"
		"addi t3, t3, -1                           \n\t"
		"bnez t3, 2b                               \n\t"

		"sw   t2, 0(t1)                            \n\t"
		"li   t3, 800000                           \n\t"
		"3:                                        \n\t"
		"addi t3, t3, -1                           \n\t"
		"bnez t3, 3b                               \n\t"

		"j    1b                                   \n\t"
	);
}

/*--- libc syscall stubs kept so the rest of the lib still links ---*/

__attribute__((weak))
caddr_t _sbrk(ptrdiff_t incr)
{
	static uint8_t *heap_ptr = NULL;
	uint8_t *prev_heap;
	if (heap_ptr == NULL) heap_ptr = (uint8_t *)&__heap_start__;
	prev_heap = heap_ptr;
	if ((heap_ptr + incr) > (uint8_t *)&__heap_end__) {
		errno = ENOMEM;
		return (caddr_t)-1;
	}
	heap_ptr += incr;
	return (caddr_t)prev_heap;
}

__attribute__((weak)) int  _close(int fd)                                { (void)fd; return -1; }
__attribute__((weak)) int  _fstat(int fd, struct stat *st)               { (void)fd; st->st_mode = S_IFCHR; return 0; }
__attribute__((weak)) int  _isatty(int fd)                               { (void)fd; return 1; }
__attribute__((weak)) int  _lseek(int fd, int offset, int whence)        { (void)fd; (void)offset; (void)whence; return -1; }
__attribute__((weak)) int  _read(int fd, char *buf, size_t len)          { (void)fd; (void)buf; (void)len; return -1; }
__attribute__((weak)) int  _write(int fd, char *buf, size_t len)         { (void)fd; (void)buf; (void)len; return -1; }
__attribute__((weak)) void _exit(int status)                             { (void)status; while(1); }
__attribute__((weak)) void _kill(int pid, int sig)                       { (void)pid; (void)sig; }
__attribute__((weak)) int  _getpid(void)                                 { return -1; }
