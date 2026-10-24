// Internal operation exclusion for devices sharing the Silex register transport.
// Not an application interface. The transport busy flag still protects each transfer.
// MIT License, Copyright (c) 2026 I-SYST.
#ifndef IOSONATA_SILEX_INTERNAL_H
#define IOSONATA_SILEX_INTERNAL_H
#include <stdatomic.h>

inline atomic_flag &SilexOperationFlag()
{
	static atomic_flag busy = ATOMIC_FLAG_INIT;
	return busy;
}

inline bool SilexOpAcquire()
{
	return !atomic_flag_test_and_set(&SilexOperationFlag());
}

inline void SilexOpRelease()
{
	atomic_flag_clear(&SilexOperationFlag());
}
#endif
