#ifndef __BT_TEST_SDK_COMMON_H__
#define __BT_TEST_SDK_COMMON_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_error.h"

// The SDK gates every module on a per-module enable macro. The host build
// compiles the module unconditionally, so the gate is always open.
#define NRF_MODULE_ENABLED(module)		1

// Repeated pairing attempt protection lives in the SDK auth_status_tracker
// module, which is not part of this harness.
#define PM_RA_PROTECTION_ENABLED		0

typedef uint32_t ret_code_t;

#endif
