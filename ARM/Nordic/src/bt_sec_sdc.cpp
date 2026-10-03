/**-------------------------------------------------------------------------
@file	bt_sec_sdc.cpp

@brief	SoftDevice Controller application security module.

		Extracted from bt_app_sdc.cpp. Owns the SMP start up: the P-256 ECDH
		engine, the SMP self-tests, the pairing configuration and the bond
		store, and secures each new link when a secure SecType is configured.

		This object is linked only when the application calls BtAppSecInit,
		normally from BtAppInitUserData. An application that does not use
		security never references it, so SMP, the ECDH engine and the bond
		store stay out of the image.

@author	Hoang Nguyen Hoan
@date	Oct. 1, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_pds.h"		// BtSmpBondNvmInit (bond persistence on an Nvm)

#include "crypto/crypto_uecc.h"
#include "crypto_rng_nrf.h"
#if defined(NRF54L15_XXAA) || defined(NRF54H20_XXAA)
#include "cracen_intrf.h"
#include "crypto/ba414ep.h"
#elif defined(NRF52840_XXAA)
#include "crypto_cc3xx.h"
#endif

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

// Which persistence the build uses. Deliberately independent of DEBUG_ENABLE
// and of NDEBUG: the release build is where a bond has to survive a reset, and
// this one line says whether the store is even in the picture.
#define STORE_TRACE

#ifdef STORE_TRACE
#include "syslog.h"
#define STORE_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define STORE_PRINTF(...)
#endif

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// Defined in bt_app_sdc.cpp: gives the port timer event the periodic work of
// this module.
void BtAppSdcSecPollSet(void (*Poll)(void));

// Set when the bond store is in use, see BtAppSecInit.
static bool s_bBtSecSdcBondStore = false;

// Periodic work of the security module, called once per second from the
// timer event of the port.
static void BtSecSdcPoll(void)
{
	// Pairing timeout (Core Vol 3 Part H 3.4). A busy crypto engine is
	// retried from its own queued event. Cheap no-op when no pairing is in
	// progress.
	BtSmpTimeoutCheck();

	if (s_bBtSecSdcBondStore)
	{
		BtSmpBondNvmPoll();
	}
}

// Connection callback of the port (BtAppConnected), saved when BtAppSecInit
// hooks the HCI device connection callback.
static void (*s_BtSecSdcConnected)(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6]) = nullptr;

// Surface a secured link (fresh pairing or bonded reconnect) to the application.
// The generic SMP engine calls this on every successful encryption; translate it
// to the port-neutral BtAppEvtSecured hook the example waits on before discovery.
// Declared in bt_smp.h, so no linkage specifier is needed here.
void BtSmpPairingComplete(uint16_t ConnHdl, bool Success,
						  const BtSmpKeys_t *pKeys)
{
	(void)pKeys;
	if (Success)
	{
		BtAppEvtSecured(ConnHdl);
	}
}

// Connection callback, installed in place of the port one by BtAppSecInit.
// The port callback runs first: it creates the peer record SMP works on.
static void BtSecSdcConnected(uint16_t ConnHdl, uint8_t Role, uint8_t AddrType, uint8_t PeerAddr[6])
{
	if (s_BtSecSdcConnected != nullptr)
	{
		s_BtSecSdcConnected(ConnHdl, Role, AddrType, PeerAddr);
	}

	// A link the peer pool rejected is not exposed to security.
	if (BtPeerFindByHdl(ConnHdl) == nullptr)
	{
		return;
	}

	// If a secure SecType was configured, secure the link. As the central we
	// initiate pairing (or re-encrypt from a bond); as the peripheral we send a
	// Security Request. Host-driven SMP, internal so the application stays
	// SDK-neutral - it does not call any stack-specific function.
	// Which procedure to run is decided by this link's role, not by the
	// device's configured role bitmask: a device built for both roles has
	// both bits set, so the bitmask cannot say what this connection is. The
	// central starts pairing; the peripheral asks the central to secure the
	// link with a Security Request (Vol 3 Part H 2.4.6, 3.5.1).
	if (g_BtAppData.AppDevice.bSecure)
	{
		switch (Role)
		{
			case BT_CONN_ROLE_CENTRAL:
				BtSmpStartPairing(ConnHdl);
				break;

			case BT_CONN_ROLE_PERIPHERAL:
				BtSmpRequestSecurity(ConnHdl);
				break;

			default:
				// Role not reported: starting the wrong procedure would be
				// rejected by the peer, so start neither.
				break;
		}
	}
}

/**
 * @brief	Start the security module.
 *
 * Called by the application, normally from BtAppInitUserData. The call is
 * what links this module: SMP, its ECDH engine and the bond store. Uses the
 * SecType and SecExchg given to BtAppInit.
 *
 * @return	true - security started
 */
bool BtAppSecInit(void)
{
	if (g_BtAppData.bSecInit)
	{
		// Already started
		return true;
	}

	BtHciDevice_t *pDev = (BtHciDevice_t*)g_BtAppData.AppDevice.pHciDev;
	if (pDev == nullptr)
	{
		// BtAppInit has not set up the HCI host yet
		return false;
	}

	// Security works on a link. Start connection support first so that the
	// connection callback hooked below is the one of the port.
	if (BtAppConnInit() == false)
	{
		return false;
	}

	// The SDC path owns its SMP crypto: P-256 ECDH and the BLE controller
	// (HCI LE Encrypt) for AES. These are internal to this path - the
	// application does not supply or see them; it only requests security via
	// SecType. Randomness comes from the Nordic hardware RNG.
	//
	// The SDC runs on several families with different P-256 hardware, selected
	// at compile time:
	//   nRF54L15 / nRF54H20 : CRACEN            -> Ba414ep (hardware)
	//   nRF52840            : CryptoCell CC310  -> CryptoCc3xx (hardware)
	//   nRF52832            : no accelerator    -> CryptoUecc (software)
	//   nRF5340 (net core)  : CC312 is on the app core secure domain, not
	//                         reachable from the network core -> CryptoUecc
	// The controller supplies AES-128 ECB through the HCI LE Encrypt path
	// (CryptoCtlrSdc). SMP composes the ECDH engine and the AES engine.
	KeyAgreeEngine *pEcdh = nullptr;
#if defined(NRF54L15_XXAA) || defined(NRF54H20_XXAA)
	static Ba414ep s_Ecdh;								// CRACEN engine object
	if (s_Ecdh.Init(CracenIntrfInstance(), CryptoRngNrfInstance()))
	{
		pEcdh = &s_Ecdh;
		DEBUG_PRINTF("Crypto ECDH engine: Ba414ep (CRACEN hardware P-256)\r\n");
	}
#elif defined(NRF52840_XXAA)
	alignas(CryptoCc3xx) static uint8_t s_CryptoEcdhMem[CRYPTO_CC3XX_MEMSIZE];
	pEcdh = CryptoCc3xxCreate(s_CryptoEcdhMem, sizeof(s_CryptoEcdhMem),
							 CryptoRngNrfInstance());
	if (pEcdh != nullptr)
	{
		DEBUG_PRINTF("Crypto ECDH engine: CryptoCc3xx (CC310 hardware P-256)\r\n");
	}
#else
	alignas(uint64_t) static uint8_t s_CryptoEcdhMem[CRYPTO_UECC_MEMSIZE];
	pEcdh = CryptoUeccCreate(s_CryptoEcdhMem, sizeof(s_CryptoEcdhMem),
							 CryptoRngNrfInstance());
	if (pEcdh != nullptr)
	{
		DEBUG_PRINTF("Crypto ECDH engine: CryptoUecc (software P-256)\r\n");
	}
#endif
	if (pEcdh == nullptr)
	{
		// No P-256 engine came up. LE Secure Connections pairing cannot run:
		// SmpLocalKeyGen fails and SMP answers every pairing with
		// BT_SMP_ERR_UNSPECIFIED. Say so here rather than at the first pairing.
		DEBUG_PRINTF("Crypto ECDH engine MISSING, LESC pairing will fail\r\n");
	}

	CipherEngine *pAes = BtCryptoCtlrSdcInit();
	if (!BtSmpInit(pEcdh, pAes, CryptoRngNrfInstance()))
	{
		// A mandatory crypto provider (ECDH, AES or secure RNG) is missing or
		// unsuitable. Secure Connections pairing cannot run; fail init here
		// rather than at the first pairing attempt.
		DEBUG_PRINTF("BtAppInit: BtSmpInit rejected a crypto provider\r\n");
		return false;
	}

	// Verify the SMP crypto toolbox against the specification sample data
	// before any pairing can run. These exercise the AES-CMAC based SC
	// functions (f4) and the legacy c1/s1 confirm functions, plus the RPA and
	// signing helpers, catching a byte-order or AES engine fault at startup
	// rather than as a confirm mismatch against a peer. A failure here means
	// the composed AES engine is wrong; refuse to continue.
	if (BtSmpF4SelfTest() != 0 || BtSmpC1S1SelfTest() != 0 ||
		BtSmpRpaSelfTest() != 0 || BtSmpSignSelfTest() != 0)
	{
		DEBUG_PRINTF("BtAppInit: SMP crypto self-test FAILED\r\n");
		return false;
	}

	// Translate the application security configuration into the SMP IO
	// capability, authentication requirements and association-model callbacks.
	// SecExchg selects the IO capability; SecType selects bonding and MITM. The
	// Secure Connections bit is forced inside BtSmpAuthConfig.
	uint8_t smpIoCaps;
	if ((g_BtAppData.SecExchg & BTAPP_SECEXCHG_KEYBOARD) &&
		(g_BtAppData.SecExchg & BTAPP_SECEXCHG_DISPLAY))
	{
		smpIoCaps = BT_SMP_IOCAPS_KEYBOARD_DISPLAY;
	}
	else if ((g_BtAppData.SecExchg & BTAPP_SECEXCHG_DISPLAY) &&
			 (g_BtAppData.SecExchg & BTAPP_SECEXCHG_YESNO))
	{
		smpIoCaps = BT_SMP_IOCAPS_DISPLAY_YESNO;
	}
	else if (g_BtAppData.SecExchg & BTAPP_SECEXCHG_KEYBOARD)
	{
		smpIoCaps = BT_SMP_IOCAPS_KEYBOARD_ONLY;
	}
	else if (g_BtAppData.SecExchg & BTAPP_SECEXCHG_DISPLAY)
	{
		smpIoCaps = BT_SMP_IOCAPS_DISPLAY_ONLY;
	}
	else
	{
		smpIoCaps = BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT;
	}

	uint8_t smpAuthReq = 0;
	if (g_BtAppData.SecType != BTGAP_SECTYPE_NONE)
	{
		smpAuthReq |= BT_SMP_AUTHREQ_BONDING_FLAG_BONDING;
	}
	if (g_BtAppData.SecType == BTGAP_SECTYPE_STATICKEY_MITM ||
		g_BtAppData.SecType == BTGAP_SECTYPE_LESC_MITM ||
		g_BtAppData.SecType == BTGAP_SECTYPE_SIGNED_MITM)
	{
		smpAuthReq |= BT_SMP_AUTHREQ_MITM;
	}

	BtSmpAuthConfig(smpIoCaps, smpAuthReq);

	// Bring up flash-backed bond persistence when security is enabled. This is
	// internal to this path: it loads any stored bonds into the SMP bond table and
	// links the strong BtSmpBondSave/Load/Erase overrides. The application does
	// not call it - persistence follows from the configured SecType.
	if (g_BtAppData.SecType != BTGAP_SECTYPE_NONE)
	{
		// This path stores bonds through bt_pds on Nvm, not through the nRF5
		// SDK peer_manager and fstorage. Printed so a persistence problem is
		// not chased in the wrong layer.
		STORE_PRINTF("STORE: bt_pds -> Nvm (SDC, no fstorage)\r\n");
		int storeRes = BtSmpBondNvmInit();
		if (storeRes < 0)
		{
			STORE_PRINTF("STORE: bt_pds init failed: %d\r\n", storeRes);
			return false;
		}
		s_bBtSecSdcBondStore = true;
	}
	else
	{
		STORE_PRINTF("STORE: none, bSecure is 0 so bonds stay in RAM\r\n");
	}

	// Pairing timeout and bond save retry are driven from the port timer.
	BtAppSdcSecPollSet(BtSecSdcPoll);

	// Secure each new link: take the connection callback, the port one is
	// called first.
	s_BtSecSdcConnected = pDev->Connected;
	pDev->Connected = BtSecSdcConnected;

	g_BtAppData.bSecInit = true;

	return true;
}
