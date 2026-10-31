// STM32WBA bond cache sizing. MIT License, (c) 2026 I-SYST inc.
#ifndef __BT_WBA_BOND_H__
#define __BT_WBA_BOND_H__

// Two complete images plus PDS record and collection overhead must fit one
// erase sector. BtPdsWbaInit checks this against the mounted NVM geometry.
#ifndef CFG_BLE_NVM_MAX_SIZE
#define CFG_BLE_NVM_MAX_SIZE 384
#endif

static_assert(CFG_BLE_NVM_MAX_SIZE > 1 && CFG_BLE_NVM_MAX_SIZE <= 1024,
	"WBA bond cache word count must fit the host interface");

#endif
