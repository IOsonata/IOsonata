/**-------------------------------------------------------------------------
@file	bt_pds_sdc.h

@brief	SDC bond persistence entry point.

		The store runs on an Nvm, so nothing about the medium or the radio is
		described here. Who may touch the memory and when is registered with
		the memory interface by whoever owns the radio; the store and this
		glue never learn what a timeslot is.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __BT_PDS_SDC_H__
#define __BT_PDS_SDC_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// SDC bond persistence init. Called internally by the SDC app init when a
// secure SecType is configured (not by the application). Initializes the NVM
// implementation, loads stored bonds into the SMP bond table, and pulls in the strong
// BtSmpBondSave/Load/Erase overrides that persist bonds. Returns 0 on success.
int BtSmpBondSdcInit(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_SDC_H__
