/*--------------------------------------------------------------------------
File   : start_llvm.c

Author : Hoang Nguyen Hoan          Oct. 2026

Desc   : C runtime entry for the R9A02G021 e2 studio LLVM library.
         ResetEntryInit has initialized RAM and the MCU before calling
         _start. Run the C/C++ initializers without clearing RAM again.

Copyright (c) 2026, I-SYST, all rights reserved
----------------------------------------------------------------------------*/

extern void __libc_init_array(void);
extern int main(void);

__attribute__((noreturn))
void _start(void)
{
	__libc_init_array();
	(void)main();

	while (1)
	{
		__asm volatile("wfi");
	}
}
