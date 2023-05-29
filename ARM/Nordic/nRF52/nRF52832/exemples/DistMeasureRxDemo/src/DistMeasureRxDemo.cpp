//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2023, I-SYST inc. All rights reserved.
// Description : Hello World in C++
//============================================================================

#include "nrf_dm.h"

nrf_dm_config_t s_DmCfg = NRF_DM_DEFAULT_CONFIG;
nrf_dm_ppi_config_t s_PpiCfg = {

};

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{

	nrf_dm_init(&s_PpiCfg, );

	return 0;
}
