/* Compile/run package bonding checks; not hardware validation. */
#include <assert.h>
#include <stdio.h>
#include "ra4m1_ioregs.h"
int main(void)
{
    unsigned count=0;
    for (int p=0; p<10; ++p)
        for (int n=0; n<16; ++n) count += Ra4m1PinValid(p,n) ? 1U : 0U;
#if RA4M1_PACKAGE_PINS == 100
    assert(count==84 && Ra4m1PinValid(3,5) && Ra4m1PinValid(8,9));
#elif RA4M1_PACKAGE_PINS == 64
    assert(count==52 && !Ra4m1PinValid(3,5) && Ra4m1PinValid(3,4));
#elif RA4M1_PACKAGE_PINS == 48
    assert(count==36 && !Ra4m1PinValid(4,2) && Ra4m1PinValid(5,0));
#elif RA4M1_PACKAGE_PINS == 40
    assert(count==28 && !Ra4m1PinValid(5,0) && Ra4m1PinValid(4,8));
#endif
    assert(!Ra4m1PinValid(-1,0) && !Ra4m1PinValid(10,0));
    assert(!Ra4m1PinValid(0,-1) && !Ra4m1PinValid(0,16));
    assert(Ra4m1PinValid(2,0) && Ra4m1PinValid(2,14) && Ra4m1PinValid(2,15));
    printf("PASS: %u bonded port pins, package %d\n",count,RA4M1_PACKAGE_PINS);
    return 0;
}
