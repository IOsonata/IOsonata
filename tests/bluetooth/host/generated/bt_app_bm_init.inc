// Generated from the production nRF54 BM port. Do not edit.
static void SecurePendingReset() {}
static void BtSmpOobDataClearInternal() {}

bool BtAppInit(const BtAppCfg_t *pCfg)
{
	uint32_t err_code;

	// Initialize application event handler
	if (AppEvtHandlerInit(pCfg->pEvtHandlerQueMem, pCfg->EvtHandlerQueMemSize) == false)
	{
		return false;
	}

	SecurePendingReset();
	BtSmpOobDataClearInternal();

	// Populate internal app data from config
	g_BtAppData.AppDevice.Conn.Role = pCfg->Role;
	g_BtAppData.AdvHdl = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
	if (!BtPeerInit(pCfg->pPeerPoolMem, pCfg->PeerPoolMemSize))
	{
		return false;
	}

	// Split the long-write reassembly pool across the peer slots so each
	// link gets its own buffer (per Conn.pLongWrBuff).
	BtPeerLongWrInit(pCfg->pLongWrPoolMem, pCfg->LongWrPoolMemSize);

	// Connection pool removed: the peer manager (BtPeerInit above) owns
	// the single connection table now.
	// bExtAdv is no longer a config field; BtAdvEncode determines extended vs
	// legacy from the encoded payload and sets g_BtAppData.bExtAdv during
	// BtAppAdvInit.
	g_BtAppData.ConnLedPort = pCfg->ConnLedPort;
	g_BtAppData.ConnLedPin = pCfg->ConnLedPin;
	g_BtAppData.ConnLedActLevel = pCfg->ConnLedActLevel;
	g_BtAppData.bScan = false;
	g_BtAppData.AppDevice.VendorId = pCfg->VendorId;
	g_BtAppData.AppDevice.ProductId = pCfg->ProductId;
	g_BtAppData.AppDevice.ProductVer = pCfg->ProductVer;
	g_BtAppData.AppDevice.Appearance = pCfg->Appearance;

	g_BtAppData.AppDevice.Conn.MaxMtu = GATT_MTU_SIZE_DEFAULT;
	if (pCfg->MaxMtu > GATT_MTU_SIZE_DEFAULT)
		g_BtAppData.AppDevice.Conn.MaxMtu = pCfg->MaxMtu;

	// Setup connection LED
	if (pCfg->ConnLedPort != -1 && pCfg->ConnLedPin != -1)
	{
		IOPinConfig(pCfg->ConnLedPort, pCfg->ConnLedPin, 0,
					IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		BtAppConnLedOff();
	}

	// Initialize SoftDevice + BLE stack
	if (BtAppStackInit(pCfg) == false)
	{
		DEBUG_PRINTF("BtAppStackInit failed\r\n");
		return false;
	}


	// Set GAP appearance
	err_code = sd_ble_gap_appearance_set(pCfg->Appearance);
	(void)err_code;

	// Initialize GAP parameters
	BtGapCfg_t gapcfg = {
		.Role = static_cast<uint8_t>(pCfg->Role),
		.SecType = pCfg->SecType,
		.AdvInterval = pCfg->AdvInterval,
		.AdvTimeout = pCfg->AdvTimeout,
		.ConnIntervalMin = pCfg->ConnIntervalMin,
		.ConnIntervalMax = pCfg->ConnIntervalMax,
		.SlaveLatency = BT_GAP_CONN_SLAVE_LATENCY,
		.SupTimeout = BT_GAP_CONN_SUP_TIMEOUT
	};

	BtGapInit(&gapcfg);

	if (pCfg->pDevName != NULL)
	{
		BtGapSetDevName(pCfg->pDevName);
	}

	// Register connection parameters event handler
	ble_conn_params_evt_handler_set(on_conn_params_evt);

	// Initialize user services
	DEBUG_PRINTF("BtAppInit: Role=0x%x PERIPHERAL=%d\r\n",
		pCfg->Role, (pCfg->Role & BTAPP_ROLE_PERIPHERAL) ? 1 : 0);
	if (pCfg->Role & BTAPP_ROLE_PERIPHERAL)
	{
		DEBUG_PRINTF("BtAppInit: calling BtAppInitUserServices\r\n");
		BtAppInitUserServices();
	}

	// Register the IOsonata Device Information Service through the
	// target BtGattSrvcAdd/BtGattCharSetValue implementation.
	if (pCfg->pDevInfo != NULL && BtDisInit(pCfg) == false)
	{
		return false;
	}

	// Initialize user data
	BtAppInitUserData();

	g_BtAppData.AppDevice.bSecure = pCfg->SecType != BTGAP_SECTYPE_NONE;

	// Initialize Secure Connections (peer_manager + LESC) when the app
	// requests security. Bonds persist through the IOsonata bt_pds store.
	if (g_BtAppData.AppDevice.bSecure)
	{
		// No erase-bond flag in BtAppCfg_t; bonds are preserved across init.
		// A dedicated clear (pm_peers_delete / BtSmpBondClearAll) can be added
		// as a separate API if forced re-bonding is needed.
		if (BtAppPeerMngrInit(pCfg->SecType, pCfg->SecExchg, false) != NRF_SUCCESS)
		{
			DEBUG_PRINTF("BtAppPeerMngrInit failed\r\n");
			return false;
		}
	}

	// Initialize advertising
	if (g_BtAppData.AppDevice.Conn.Role & (BTAPP_ROLE_PERIPHERAL | BTAPP_ROLE_BROADCASTER))
	{
		if (BtAppAdvInit(pCfg) == false)
		{
			DEBUG_PRINTF("BtAppAdvInit failed\r\n");
			return false;
		}

		err_code = sd_ble_gap_tx_power_set(
			BLE_GAP_TX_POWER_ROLE_ADV,
			g_BtAppData.AdvHdl,
			GetValidTxPower(pCfg->TxPower));
		(void)err_code;
	}
	else
	{
		err_code = sd_ble_gap_tx_power_set(
			BLE_GAP_TX_POWER_ROLE_SCAN_INIT,
			0,
			GetValidTxPower(pCfg->TxPower));
		(void)err_code;
	}

	g_BtAppData.State = BTAPP_STATE_INITIALIZED;

	DEBUG_PRINTF("BtAppInit: success\r\n");

	return true;
}
