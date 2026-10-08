/* TEST ONLY: GPIO effects are recorded, never written to hardware. */
#pragma once
#include "example_api.h"
static inline void IOPinToggle(int p,int n) {ExampleToggle(p,n);}
