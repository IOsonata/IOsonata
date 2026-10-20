// TEST ONLY: the stddev.h declaration used by the shared timer example.
// Its call is built with NDEBUG only, which the host models do not define.
#pragma once
#include "example_api.h"
bool UARTRetargetEnable(UART &Dev, int FileNo);
