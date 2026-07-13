from pathlib import Path

path = Path('ARM/Nordic/nRF54/src/bt_app_bm.cpp')
text = path.read_text()

anchor = '''#define DEBUG_PRINTF(...)
#endif

/*******************************/
'''
insert = '''#define DEBUG_PRINTF(...)
#endif

extern "C" void softdevice_fault_handler(uint32_t Id, uint32_t Pc, uint32_t Info)
{
	DEBUG_PRINTF("SoftDevice fault: id=0x%08lx pc=0x%08lx info=0x%08lx\\r\\n",
				 (unsigned long)Id, (unsigned long)Pc, (unsigned long)Info);
	while (true)
	{
		__NOP();
	}
}

extern "C" void HardFault_Handler(void)
{
	DEBUG_PRINTF("HardFault: CFSR=0x%08lx HFSR=0x%08lx MMFAR=0x%08lx BFAR=0x%08lx\\r\\n",
				 (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR,
				 (unsigned long)SCB->MMFAR, (unsigned long)SCB->BFAR);
	while (true)
	{
		__NOP();
	}
}

/*******************************/
'''
if anchor not in text:
    raise SystemExit('debug anchor not found')
text = text.replace(anchor, insert, 1)

repls = [
('''\terr = nrf_sdh_enable_request();
\tif (err && err != -EALREADY)
''', '''\terr = nrf_sdh_enable_request();
\tDEBUG_PRINTF("BtAppStackInit: nrf_sdh_enable_request returned %d\\r\\n", err);
\tif (err && err != -EALREADY)
'''),
('''\terr = SDBleDefaultCfgSet(pCfg, BTAPP_CONN_CFG_TAG, ramstart);

\tif (err != NRF_SUCCESS)
''', '''\tDEBUG_PRINTF("BtAppStackInit: SDBleDefaultCfgSet\\r\\n");
\terr = SDBleDefaultCfgSet(pCfg, BTAPP_CONN_CFG_TAG, ramstart);
\tDEBUG_PRINTF("BtAppStackInit: SDBleDefaultCfgSet returned 0x%lx\\r\\n",
\t\t\t\t (unsigned long)err);

\tif (err != NRF_SUCCESS)
'''),
('''\t// Require before call to nrf_sdh_ble_enable
\tSDBleRandSeed(NRF_EVT_RAND_SEED_REQUEST, NULL);

\tDEBUG_PRINTF("BtAppStackInit: nrf_sdh_ble_enable\\r\\n");
''', '''\t// Required before calling sd_ble_enable.
\tDEBUG_PRINTF("BtAppStackInit: SDBleRandSeed\\r\\n");
\tSDBleRandSeed(NRF_EVT_RAND_SEED_REQUEST, NULL);
\tDEBUG_PRINTF("BtAppStackInit: SDBleRandSeed done\\r\\n");

\tDEBUG_PRINTF("BtAppStackInit: sd_ble_enable\\r\\n");
'''),
('''\terr = sd_ble_enable(&ramreq);

\tif (err != NRF_SUCCESS)
''', '''\terr = sd_ble_enable(&ramreq);
\tDEBUG_PRINTF("BtAppStackInit: sd_ble_enable returned 0x%lx ram=0x%08lx\\r\\n",
\t\t\t\t (unsigned long)err, (unsigned long)ramreq);

\tif (err != NRF_SUCCESS)
'''),
('''\tBtLescSetCryptoEngine(&s_LescEcdh);

\terr_code = pm_init();
''', '''\tBtLescSetCryptoEngine(&s_LescEcdh);

\tDEBUG_PRINTF("BtAppPeerMngrInit: pm_init\\r\\n");
\terr_code = pm_init();
\tDEBUG_PRINTF("BtAppPeerMngrInit: pm_init returned 0x%lx\\r\\n",
\t\t\t\t (unsigned long)err_code);
'''),
('''\tif (BtAppStackInit(pCfg) == false)
\t{
\t\tDEBUG_PRINTF("BtAppStackInit failed\\r\\n");
\t\treturn false;
\t}


\t// Set GAP appearance
''', '''\tif (BtAppStackInit(pCfg) == false)
\t{
\t\tDEBUG_PRINTF("BtAppStackInit failed\\r\\n");
\t\treturn false;
\t}
\tDEBUG_PRINTF("BtAppInit: stack initialized\\r\\n");

\t// Set GAP appearance
''')
]
for old, new in repls:
    if old not in text:
        raise SystemExit('replacement anchor not found: ' + old[:80])
    text = text.replace(old, new, 1)

path.write_text(text)
