/**-------------------------------------------------------------------------
@file	bt_smp.h

@brief	Bluetooth Security Manager Protocol (SMP)

Generic implementation & definitions of Bluetooth Security Manager Protocol

L2CAP channel 6

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#ifndef __BT_SMP_H__
#define __BT_SMP_H__

#include <inttypes.h>
#include <stddef.h>

#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_hci.h"
#include "crypto/icrypto.h"

// SMP-internal crypto return codes used by the state machine. Mapped from the
// generic CRYPTO_STATUS by the dispatch wrappers in bt_smp.cpp. PENDING parks
// the state machine for a controller/secure async completion.
#define BT_SMP_CRYPTO_OK		0
#define BT_SMP_CRYPTO_PENDING	1
#define BT_SMP_CRYPTO_FAIL		(-1)

/** @addtogroup Bluetooth
  * @{
  */

#define BT_SMP_CODE_PAIRING_REQ						1
#define BT_SMP_CODE_PAIRING_RSP						2
#define BT_SMP_CODE_PAIRING_CONFIRM					3
#define BT_SMP_CODE_PAIRING_RANDOM					4
#define BT_SMP_CODE_PAIRING_FAILED					5
#define BT_SMP_CODE_PAIRING_ENCRYP_INFO				6
#define BT_SMP_CODE_PAIRING_CENTRAL_ID				7
#define BT_SMP_CODE_PAIRING_ID_INFO					8
#define BT_SMP_CODE_PAIRING_ID_ADDR_INFO			9
#define BT_SMP_CODE_PAIRING_SIGNING_INFO			0xA
#define BT_SMP_CODE_PAIRING_SECURITY_REQ			0xB
#define BT_SMP_CODE_PAIRING_PUBLIC_KEY				0xC
#define BT_SMP_CODE_PAIRING_DHKEY_CHECK				0xD
#define BT_SMP_CODE_PAIRING_KEYPRESS_NOTIF			0xE

// IO capability values (Vol 3, Part H, 3.5.1).
#define BT_SMP_IOCAPS_DISPLAY_ONLY					0
#define BT_SMP_IOCAPS_DISPLAY_YESNO					1
#define BT_SMP_IOCAPS_KEYBOARD_ONLY					2
#define BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT			3
#define BT_SMP_IOCAPS_KEYBOARD_DISPLAY				4

#define BT_SMP_OOB_AUTH_NOT_PRESENT					0	//!< OOB Authentication data not present
#define BT_SMP_OOB_AUTH_PRESENT 					1	//!< OOB Authentication data from remote device present

// Association models selected from the IO capability mapping (Core spec
// Vol 3 Part H, 2.3.5.1, Table 2.8, LE Secure Connections). JUST_WORKS is
// unauthenticated. NUMERIC_COMPARISON, PASSKEY_ENTRY and OOB are MITM
// protected and raise the link to an authenticated level on success.
#define BT_SMP_MODEL_JUST_WORKS						0
#define BT_SMP_MODEL_NUMERIC_COMPARISON				1
#define BT_SMP_MODEL_PASSKEY_ENTRY					2
#define BT_SMP_MODEL_OOB							3

// Passkey value passed to BtSmpPasskeyReply to reject / cancel entry. Any value
// above the 6 digit range is treated as a cancel.
#define BT_SMP_PASSKEY_INVALID						0xFFFFFFFFu

// AuthReq flags (Vol 3, Part H, 3.5.1, Figure 3.3).
#define BT_SMP_AUTHREQ_BONDING_FLAG_MASK			(3<<0)
#define BT_SMP_AUTHREQ_BONDING_FLAG_NO_BONDING		(0<<0)	//!< No bonding
#define BT_SMP_AUTHREQ_BONDING_FLAG_BONDING			(1<<0)	//!< Bonding requested
#define BT_SMP_AUTHREQ_MITM							(1<<2)	//!< MITM protection requested
#define BT_SMP_AUTHREQ_SC							(1<<3)	//!< LE Secure Connections supported
#define BT_SMP_AUTHREQ_KEYPRESS						(1<<4)	//!< Keypress notifications
#define BT_SMP_AUTHREQ_CT2							(1<<5)	//!< h7 key derivation supported
#define BT_SMP_AUTHREQ_RFU_MASK						(3<<6)

// Key distribution flags (Vol 3, Part H, 3.6.1).
#define BT_SMP_KEYDIST_ENCKEY						(1<<0)	//!< Distribute LTK (legacy) / not used in SC
#define BT_SMP_KEYDIST_IDKEY						(1<<1)	//!< Distribute IRK + identity address
#define BT_SMP_KEYDIST_SIGNKEY						(1<<2)	//!< Distribute CSRK
#define BT_SMP_KEYDIST_LINKKEY						(1<<3)	//!< Derive BR/EDR link key

// Pairing Failed reason codes (Vol 3, Part H, 3.5.5, Table 3.7).
#define BT_SMP_ERR_PASSKEY_ENTRY_FAILED				0x01
#define BT_SMP_ERR_OOB_NOT_AVAILABLE				0x02
#define BT_SMP_ERR_AUTHEN_REQUIREMENTS				0x03
#define BT_SMP_ERR_CONFIRM_VALUE_FAILED				0x04
#define BT_SMP_ERR_PAIRING_NOT_SUPPORTED			0x05
#define BT_SMP_ERR_ENC_KEY_SIZE						0x06
#define BT_SMP_ERR_CMD_NOT_SUPPORTED				0x07
#define BT_SMP_ERR_UNSPECIFIED						0x08
#define BT_SMP_ERR_REPEATED_ATTEMPTS				0x09
#define BT_SMP_ERR_INVALID_PARAMS					0x0A
#define BT_SMP_ERR_DHKEY_CHECK_FAILED				0x0B
#define BT_SMP_ERR_NUMERIC_COMPARISON_FAILED		0x0C
#define BT_SMP_ERR_BREDR_PAIRING_IN_PROGRESS		0x0D
#define BT_SMP_ERR_CROSS_TRANSP_NOT_ALLOWED			0x0E
#define BT_SMP_ERR_KEY_REJECTED						0x0F

#define BT_SMP_MAX_ENC_KEY_SIZE						16
#define BT_SMP_MIN_ENC_KEY_SIZE						7	//!< Spec absolute floor (Core Vol 3 Part H 2.3.4)

/// Locally-enforced minimum encryption key size. Defaults to the spec floor (7)
/// for interoperability; a security-sensitive build raises it (16 for full LE
/// Secure Connections entropy) to refuse a KNOB-style key-size downgrade, where
/// an active MITM rewrites the MaxKeySize octet in the Pairing Request/Response
/// to force both sides down to 56-bit keys. Must be within 7..16.
#ifndef BT_SMP_CFG_MIN_ENC_KEY_SIZE
#define BT_SMP_CFG_MIN_ENC_KEY_SIZE					BT_SMP_MIN_ENC_KEY_SIZE
#endif

#if (BT_SMP_CFG_MIN_ENC_KEY_SIZE < BT_SMP_MIN_ENC_KEY_SIZE) || (BT_SMP_CFG_MIN_ENC_KEY_SIZE > BT_SMP_MAX_ENC_KEY_SIZE)
#error "BT_SMP_CFG_MIN_ENC_KEY_SIZE must be between BT_SMP_MIN_ENC_KEY_SIZE (7) and BT_SMP_MAX_ENC_KEY_SIZE (16)"
#endif

/// SMP pairing timeout in milliseconds (Core Vol 3 Part H 3.4). If a pairing
/// makes no progress within this window it is failed and no further SMP is
/// accepted on the link until it disconnects.
#ifndef BT_SMP_TIMEOUT_MS
#define BT_SMP_TIMEOUT_MS							30000
#endif

/// Lock a link after this many failed pairing attempts within one connection
/// (Core Vol 3 Part H 2.3.6, repeated attempts). Further attempts are refused
/// until the link disconnects. Cross-connection exponential back-off needs
/// persistent state and is left to the platform.
#ifndef BT_SMP_MAX_PAIR_ATTEMPTS
#define BT_SMP_MAX_PAIR_ATTEMPTS					3
#endif

/// SMP pairing phase. Drives the per-link state machine in bt_smp.cpp.
/// The responder (peripheral) path is implemented end to end; the
/// initiator (central) path reuses the same states with the roles of the
/// confirm/random exchange swapped.
typedef enum __Bt_Smp_State {
	BT_SMP_STATE_IDLE = 0,			//!< No pairing in progress
	BT_SMP_STATE_PAIR_RSP_SENT,		//!< Sent Pairing Response, waiting for peer
	BT_SMP_STATE_PUBKEY_WAIT,		//!< SC: waiting for peer public key
	BT_SMP_STATE_PUBKEY_LOCAL_WAIT,	//!< SC: waiting for local P-256 key from controller
	BT_SMP_STATE_DHKEY_WAIT,		//!< SC: waiting for DHKey from controller
	BT_SMP_STATE_CONFIRM_WAIT,		//!< Waiting for peer Pairing Confirm
	BT_SMP_STATE_RANDOM_WAIT,		//!< Waiting for peer Pairing Random
	BT_SMP_STATE_DHKEY_CHECK_WAIT,	//!< SC: waiting for peer DHKey Check
	BT_SMP_STATE_NUMERIC_WAIT,		//!< SC: waiting for user numeric comparison confirm
	BT_SMP_STATE_PASSKEY_WAIT,		//!< SC: input side waiting for user passkey entry
	BT_SMP_STATE_LTK_WAIT,			//!< Waiting for controller LTK request (enc start)
	BT_SMP_STATE_KEYDIST,			//!< Distributing / receiving transport keys
	BT_SMP_STATE_DONE				//!< Pairing complete, link encrypted
} BtSmpState_t;

/// Per-link security key material. For bonded peers it is the record the
/// application persists and reloads on reconnect.
typedef struct __Bt_Smp_Keys {
	uint8_t  Ltk[16];				//!< Long Term Key
	uint8_t  Irk[16];				//!< Identity Resolving Key (peer)
	uint8_t  Csrk[16];				//!< Connection Signature Resolving Key (peer)
	uint64_t Rand;					//!< LTK Rand (legacy; 0 for SC)
	uint16_t Ediv;					//!< LTK EDIV (legacy; 0 for SC)
	uint8_t  EncKeySize;			//!< Negotiated encryption key size, bytes
	uint8_t  IdAddrType;			//!< Peer identity address type
	uint8_t  IdAddr[6];				//!< Peer identity address
	bool     bAuthenticated;		//!< true if MITM-protected (passkey/numeric/OOB)
	bool     bSc;					//!< true if produced by LE Secure Connections
	bool     bValid;				//!< true once the key set is populated
} BtSmpKeys_t;

/// Per-link SMP context. Held inside the peer record; allocated/freed with
/// the link. Holds transient pairing state that must not leak across
/// connections (the pre-refactor globals did exactly that).
typedef struct __Bt_Smp_Ctx {
	BtSmpState_t State;				//!< Current pairing state
	uint8_t  PReq[7];				//!< Cached Pairing Request PDU (for confirm calc)
	uint8_t  PRsp[7];				//!< Cached Pairing Response PDU (for confirm calc)
	uint8_t  IoCaps;				//!< Local IO capability in use
	uint8_t  AuthReq;				//!< Negotiated AuthReq
	uint8_t  PeerAuthReq;			//!< Peer-requested AuthReq
	uint8_t  Model;					//!< Selected association model (BT_SMP_MODEL_*)
	bool     bSc;					//!< true if SC negotiated for this pairing
	bool     bInitiator;			//!< true if local device is the SMP initiator (central)
	uint8_t  Tk[16];				//!< Temporary Key (legacy) / 0 for Just Works
	uint8_t  LocalRand[16];			//!< Local random (Mrand / Srand)
	uint8_t  PeerRand[16];			//!< Peer random
	uint8_t  LocalConfirm[16];		//!< Local confirm value
	uint8_t  PeerConfirm[16];		//!< Peer confirm value
	uint8_t  LocalPubKey[64];		//!< SC: local P-256 public key (X||Y)
	uint8_t  PeerPubKey[64];		//!< SC: peer P-256 public key (X||Y)
	uint8_t  DhKey[32];				//!< SC: computed DHKey
	uint8_t  EcdhKeyCtx[64];		//!< SC: ECDH private-key context, KeyGen to Agree (per-link, wiped after)
	uint8_t  Mackey[16];			//!< SC: MacKey from f5
	uint8_t  Ltk[16];				//!< Derived/working LTK
	uint32_t Passkey;				//!< Passkey Entry: 6 digit value 0..999999
	uint8_t  PkRound;				//!< Passkey Entry: current round 0..19
	bool     bPkDisplay;			//!< Passkey Entry: true if local displays, false if local inputs
	bool     bPkReady;				//!< Passkey Entry: true once Passkey is known
	bool     bPkPeerCommit;			//!< Passkey Entry: peer Confirm buffered before Passkey was entered
	bool     bOobPeerData;			//!< OOB: peer OOB data was provided for this pairing
	uint8_t  OobLocalRand[16];		//!< OOB: local random distributed out of band (SMP order)
	uint8_t  OobPeerRand[16];		//!< OOB: peer random received out of band (SMP order)
	uint8_t  OobPeerConfirm[16];	//!< OOB: peer confirm received out of band (SMP order)
	uint8_t  KeyDistExp;			//!< Phase-3 peer key-distribution bits still expected (BT_SMP_KEYDIST_*)
	uint32_t TmrStart;				//!< BtSmpMsTick() when the current pairing started (SMP timeout anchor)
	uint8_t  FailCount;				//!< Failed pairing attempts on this link (repeated-attempts guard)
	bool     bLocked;				//!< Link locked after timeout / repeated attempts; reject all SMP until disconnect
} BtSmpCtx_t;

#pragma pack(push, 1)

typedef struct __Bt_Smp_Packet {
	uint8_t Code;
	uint8_t Data[64];			//!< Max data size is 65 byte secure or 23 non secure
} BtSmpPacket_t;

typedef struct __Bt_Smp_Paring_Req {
	uint8_t Code;				//!< SMP code
	uint8_t IOCaps;				//!< IO capability
	uint8_t OOBFlag;			//!< OOB data flag
	uint8_t AuthReq;			//!< Authentication requirements
	uint8_t MaxKeySize;			//!< Max encryption key size in bytes
	uint8_t InitiatorKeyDist;	//!< Initiator key distribution
	uint8_t ResponderKeyDist;	//!< Responder key distribution
} BtSmpPairingReq_t;

typedef BtSmpPairingReq_t BtSmpPairingRsp_t;

typedef struct __Bt_Smp_Pairing_Confirm {
	uint8_t Code;				//!< SMP code
	uint8_t Value[16];
} BtSmpPairingConfirm_t;

typedef struct __Bt_Smp_Pairing_Random {
	uint8_t Code;				//!< SMP code
	uint8_t Value[16];
} BtSmpPairingRandom_t;

typedef struct __Bt_Smp_Pairing_Failed {
	uint8_t Code;				//!< SMP code
	uint8_t Reason;				//!< Pairing Failed reason
} BtSmpPairingFailed_t;

typedef struct __Bt_Smp_Public_Key {
	uint8_t Code;				//!< SMP code
	uint8_t KeyX[32];			//!< P-256 public key X coordinate
	uint8_t KeyY[32];			//!< P-256 public key Y coordinate
} BtSmpPublicKey_t;

typedef struct __Bt_Smp_DHKey_Check {
	uint8_t Code;				//!< SMP code
	uint8_t Value[16];			//!< Ea / Eb
} BtSmpDhKeyCheck_t;

typedef struct __Bt_Smp_Encrypt_Info {
	uint8_t Code;				//!< SMP code
	uint8_t Ltk[16];			//!< Long Term Key (legacy distribution)
} BtSmpEncryptInfo_t;

typedef struct __Bt_Smp_Central_Id {
	uint8_t  Code;				//!< SMP code
	uint16_t Ediv;				//!< EDIV
	uint64_t Rand;				//!< Rand
} BtSmpCentralId_t;

typedef struct __Bt_Smp_Id_Info {
	uint8_t Code;				//!< SMP code
	uint8_t Irk[16];			//!< Identity Resolving Key
} BtSmpIdInfo_t;

typedef struct __Bt_Smp_Id_Addr_Info {
	uint8_t Code;				//!< SMP code
	uint8_t AddrType;			//!< Identity address type
	uint8_t Addr[6];			//!< Identity address
} BtSmpIdAddrInfo_t;

typedef struct __Bt_Smp_Signing_Info {
	uint8_t Code;				//!< SMP code
	uint8_t Csrk[16];			//!< Connection Signature Resolving Key
} BtSmpSigningInfo_t;

typedef struct __Bt_Smp_Security_Req {
	uint8_t Code;				//!< SMP code
	uint8_t AuthReq;			//!< Authentication requirements
} BtSmpSecurityReq_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Process an inbound SMP PDU (L2CAP CID 6).
 *
 * Now takes the connection handle: every reply this function emits, and the
 * LTK reply it ultimately triggers, are addressed by handle. The previous
 * signature dropped it, which made any response impossible on a multi-link
 * stack.
 *
 * @param	pDev		HCI device the link belongs to.
 * @param	ConnHdl		Connection handle the PDU arrived on.
 * @param	pSmp		SMP payload (points at L2CAP Smp field, not the header).
 * @param	Len			SMP payload length in bytes (L2CAP Hdr.Len).
 */
void BtProcessSmpData(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					  BtL2CapSmp_t * const pSmp, size_t Len);

/**
 * @brief	Controller LE Long Term Key Request handler.
 *
 * Called from the HCI LE event path on BT_HCI_EVT_LE_LONGTERM_KEY_RQST.
 * Looks up the peer's stored/just-derived LTK and issues the positive or
 * negative LTK reply. This is the step that actually starts link
 * encryption; without it the peripheral never encrypts and the central
 * times out.
 */
void BtSmpProcessLtkRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint64_t Rand, uint16_t Ediv);

/**
 * @brief	Peripheral-initiated security. Sends an SMP Security Request on the
 *			given connection to prompt the central to encrypt (with an existing
 *			bond) or start pairing. Call from the connection-established hook
 *			when the peripheral wants a secure/bonded link.
 */
void BtSmpRequestSecurity(uint16_t ConnHdl);

/**
 * @brief	Central-initiated security (SMP initiator role). Call from the
 *			connection-established hook when the local device is the central and
 *			wants a secure/bonded link. If a Secure Connections bond already
 *			exists for the peer address it re-encrypts from the stored LTK;
 *			otherwise it sends a Pairing Request and runs the SC initiator
 *			exchange. This build is LE Secure Connections, Just Works only.
 */
void BtSmpStartPairing(uint16_t ConnHdl);

/**
 * @brief	Release the SMP link state for a connection. Call from the
 *			disconnection hook to free the per-connection pairing context.
 */
void BtSmpDisconnected(uint16_t ConnHdl);

/**
 * @brief	Millisecond tick for the SMP pairing timeout (Core Vol 3 Part H 3.4).
 *
 * Weak default returns 0, so the timeout is inert on ports that do not supply a
 * clock: elapsed time is always 0 and a pairing never times out. An app with a
 * running millisecond counter overrides this to enable the timeout. The value
 * only needs to advance in milliseconds; wrap-around is handled.
 */
uint32_t BtSmpMsTick(void);

/**
 * @brief	Fail and lock any pairing that has exceeded BT_SMP_TIMEOUT_MS.
 *
 * Call periodically from the application main loop (or a timer). A stalled
 * pairing produces no further SMP PDUs, so this external tick fires the timeout
 * for a peer that connects, starts pairing, then stops sending. Has no effect
 * until BtSmpMsTick() is overridden with a running millisecond counter.
 */
void BtSmpTimeoutCheck(void);

/**
 * @brief	Initialise the SMP layer and compose its crypto from engines.
 *
 * SMP needs two primitives - ECDH (P-256) and AES-128 ECB - each supplied by an
 * OO crypto engine (crypto/icrypto.h): a KeyAgreeEngine and a CipherEngine.
 * The application constructs the engine that fills each slot from whatever its
 * target provides: hardware P-256 (Ba414ep on CRACEN, CryptoCc3xx on CC310) or
 * software (CryptoUecc), and AES from the BLE controller (CryptoCtlrSdc). A null
 * or incapable engine leaves that slot disabled and the corresponding operation
 * fails loud. The same engine pointer may be passed for both slots if one engine
 * implements both facets.
 *
 * Randomness is NOT a slot: it comes from the target RngGet driver
 * (crypto/icrypto.h), backed by the MCU RNG peripheral, which SMP calls directly.
 * A target without an RNG peripheral does not link.
 *
 * @param	pEcdh	KeyAgreeEngine providing P-256 ECDH.
 * @param	pAes	CipherEngine providing AES-128 ECB.
 */
void BtSmpInit(KeyAgreeEngine *pEcdh, CipherEngine *pAes);

/// Configure the local IO capability and authentication requirements. Call
/// after BtSmpInit. When never called the defaults are NoInputNoOutput /
/// bonding + Secure Connections, which resolves to Just Works. The Secure
/// Connections bit is forced set; this build does not pair with legacy.
void BtSmpAuthConfig(uint8_t IoCaps, uint8_t AuthReq);

/// Numeric Comparison user interaction. The SMP core calls this when a pairing
/// selects the Numeric Comparison model: display Value (a 6 digit number) and
/// ask the user whether it matches the value shown on the peer, then resume by
/// calling BtSmpNumericComparisonReply. The library provides a weak default
/// that rejects the pairing; an application advertising DisplayYesNo or
/// KeyboardDisplay overrides it with a strong definition.
void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value);

/// Resume a Numeric Comparison pairing from the application. Call in response
/// to BtSmpNumericComparison: Confirm true when the user reports the values on
/// both devices match, false otherwise. On a false reply pairing aborts; the
/// link is left unencrypted.
void BtSmpNumericComparisonReply(uint16_t ConnHdl, bool Confirm);

/// Passkey Entry display. The SMP core calls this on the device that displays
/// the passkey when a pairing selects the Passkey Entry model: show Passkey (a
/// 6 digit number) to the user. The peer device inputs the same value. The
/// library provides a weak default that does nothing.
void BtSmpPasskeyDisplay(uint16_t ConnHdl, uint32_t Passkey);

/// Passkey Entry request. The SMP core calls this on the device that inputs the
/// passkey: prompt the user for the 6 digit number shown on the peer, then
/// resume by calling BtSmpPasskeyReply. The library provides a weak default
/// that rejects the pairing.
void BtSmpPasskeyRequest(uint16_t ConnHdl);

/// Resume a Passkey Entry pairing from the application. Call in response to
/// BtSmpPasskeyRequest with the value the user entered (0..999999). Pass
/// BT_SMP_PASSKEY_INVALID (or any value above the 6 digit range) to cancel; the
/// pairing then aborts and the link is left unencrypted.
void BtSmpPasskeyReply(uint16_t ConnHdl, uint32_t Passkey);

/**
 * @brief	Generate the local LE Secure Connections OOB data set.
 *
 * Generates the P-256 key pair the next pairing will use, a 16 octet random r
 * and the confirm C = f4(PKx, PKx, r, 0). The application transfers r and C
 * (with the local address) to the peer over the out of band channel. The
 * crypto provider retains the private key; the next pairing reuses this key
 * pair so the confirm stays valid. All values are SMP byte order, low octet
 * first.
 *
 * @param	pDev	HCI device used by the crypto provider.
 * @param	pRand	Filled with the 16 octet random r.
 * @param	pConf	Filled with the 16 octet confirm C.
 *
 * @return	0 on success, negative on crypto failure or a pending asynchronous
 *			key generation, which this call does not support.
 */
int BtSmpOobLocalDataGen(BtHciDevice_t * const pDev, uint8_t * const pRand, uint8_t * const pConf);

/**
 * @brief	Provide the peer LE Secure Connections OOB data set.
 *
 * Stores the random and confirm received from the peer over the out of band
 * channel. The next pairing advertises OOB data present, selects the OOB
 * association model and verifies the peer public key against this confirm.
 * Values are SMP byte order, low octet first.
 *
 * @param	pRand	Peer 16 octet random r.
 * @param	pConf	Peer 16 octet confirm C.
 */
void BtSmpOobPeerDataSet(const uint8_t * const pRand, const uint8_t * const pConf);

/**
 * @brief	Discard any pending local and peer OOB data.
 */
void BtSmpOobDataClear(void);

/**
 * @brief	Bring up the Bluetooth-owned controller AES engine (SDC).
 *
 * Returns a CipherEngine whose AES-128 ECB is served by the BLE SoftDevice
 * Controller's HCI LE Encrypt. This is a Bluetooth helper, NOT a generic crypto
 * engine - it requires a running BLE controller and is only valid for the SMP
 * AES slot. On a part with no CryptoCell, compose it with a software ECDH engine
 * (randomness comes from the RngGet utility):
 *
 *   static uint8_t ecdhMem[CRYPTO_UECC_MEMSIZE];
 *   CryptoUecc *ecdh = CryptoUeccCreate(ecdhMem, sizeof(ecdhMem),
 *                                       CryptoRngNrfInstance());
 *   CipherEngine *aes = BtCryptoCtlrSdcInit();   // controller AES
 *   BtSmpInit(ecdh, aes);
 *
 * @return	The controller AES engine, or nullptr if the SDC HCI is not present.
 */
CipherEngine *BtCryptoCtlrSdcInit(void);

// SMP crypto entry points used by the state machine (dispatch to the composed
// engines). Kept as functions so the toolbox (c1/f4/f5/f6, AES-CMAC) calls a
// stable signature regardless of which engine backs each slot.
void BtSmpCryptoAes128(BtHciDevice_t * const pDev,
					   const uint8_t Key[16], const uint8_t In[16], uint8_t Out[16]);
int  BtSmpCryptoP256KeyGen(BtHciDevice_t * const pDev, uint8_t pPubKey[64]);
int  BtSmpCryptoEcdh(BtHciDevice_t * const pDev,
					 const uint8_t pPeerPubKey[64], uint8_t pDhKey[32]);
void BtSmpCryptoRand(uint8_t *pBuf, size_t Len);
int  BtSmpCryptoSelfTest(void);

/**
 * @brief	Platform hooks to answer the controller LE Long Term Key Request.
 *			These send HCI COMMANDS (not ACL data); the port implementation
 *			overrides them to use its own HCI command channel (e.g. the SDC
 *			sdc_hci_cmd_le_long_term_key_request_reply function).
 */
void BtSmpHciLtkReply(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					  const uint8_t Ltk[16]);
void BtSmpHciLtkNegReply(BtHciDevice_t * const pDev, uint16_t ConnHdl);

/**
 * @brief	Platform hook to start link encryption as the central via HCI LE
 *			Enable Encryption. Sends an HCI COMMAND (not ACL data); the port
 *			implementation overrides it to use its own HCI command channel (e.g. the
 *			SDC sdc_hci_cmd_le_enable_encryption function). For SC, Rand and Ediv
 *			are zero.
 */
void BtSmpHciEnableEncryption(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							  uint64_t Rand, uint16_t Ediv, const uint8_t Ltk[16]);

/**
 * @brief	Notify SMP that the controller produced the local P-256 key.
 *			Forwarded from BT_HCI_EVT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE.
 */
void BtSmpLocalPubKeyReady(BtHciDevice_t * const pDev, uint8_t Status,
						   const uint8_t *pKeyX, const uint8_t *pKeyY);

/**
 * @brief	Notify SMP that the controller produced the DHKey.
 *			Forwarded from BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE.
 */
void BtSmpDhKeyReady(BtHciDevice_t * const pDev, uint8_t Status,
					 const uint8_t *pDhKey);

/**
 * @brief	Notify SMP that the link encryption state changed.
 *			Forwarded from BT_HCI_EVT_ENCRYPTION_CHANGE_V1/V2. Marks the peer
 *			bSecure and, on a fresh pairing, advances to key distribution.
 */
void BtSmpEncryptionChanged(BtHciDevice_t * const pDev, uint16_t ConnHdl,
							uint8_t Status, uint8_t Enabled);

/**
 * @brief	Weak application hook: pairing completed on a link.
 *			Override to persist BtSmpKeys_t for bonding. Default does nothing.
 */
void BtSmpPairingComplete(uint16_t ConnHdl, bool Success, const BtSmpKeys_t *pKeys);

/**
 * @brief	Bond store. BtSmpBondAdd persists a key set after a successful
 *			pairing; the SMP core calls it automatically. BtSmpBondLtkLookup is
 *			used by the LTK request handler on reconnection. BtSmpBondClearAll
 *			removes all stored bonds. The default store is RAM-backed; a
 *			flash-backed port overrides BtSmpBondSave / BtSmpBondLoad /
 *			BtSmpBondErase and uses the access functions below to move records
 *			between its non-volatile storage and the generic RAM table.
 */
void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys);
bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv, uint8_t Ltk[16]);
void BtSmpBondClearAll(void);

// True if a stored bond exists for the peer on ConnHdl (matched on link address).
bool BtSmpBonded(uint16_t ConnHdl);

/**
 * @brief	Persist (BtSmpBondCccdSave) or fetch (BtSmpBondCccdGet) the CCCD set
 *			for the bonded peer on ConnHdl. Save is a no-op for unbonded peers,
 *			so CCCD stays volatile unless the client is bonded (Core spec Vol 3
 *			Part G 3.3.3.3). Get returns the entry count via handle/value arrays.
 */
void    BtSmpBondCccdSave(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value);
uint8_t BtSmpBondCccdGet(uint16_t ConnHdl, uint16_t *pHdl, uint16_t *pValue, uint8_t Max);

/**
 * @brief	Platform persistence seam (weak, RAM-only by default).
 *
 * BtSmpBondSave   : generic layer calls this when slot Slot changes; the
 *					 platform writes the Len-byte blob at pBond to NVM.
 * BtSmpBondLoad   : generic layer calls this once at BtSmpInit; the platform
 *					 reads NVM and calls BtSmpBondRestore for each saved slot.
 * BtSmpBondErase  : generic layer calls this from BtSmpBondClearAll; the
 *					 platform wipes the NVM copy.
 */
void BtSmpBondSave(int Slot, const void *pBond, size_t Len);
void BtSmpBondLoad(void);
void BtSmpBondErase(void);

/**
 * @brief	Access functions for a platform persistence store.
 *
 * BtSmpBondSlotCount  : number of slots in the table (NVM region sizing).
 * BtSmpBondRecordSize : size of one saved/loaded blob, in bytes.
 * BtSmpBondRestore    : write one loaded blob into table slot Slot. Called by a
 *					     BtSmpBondLoad override; records with bValid false are
 *					     ignored. The platform treats the blob as opaque and need
 *					     not know the record layout.
 */
int    BtSmpBondSlotCount(void);
size_t BtSmpBondRecordSize(void);
void   BtSmpBondRestore(int Slot, const void *pBond, size_t Len);

/**
 * @brief	Get the local device address and type used on air.
 *
 * The SMP toolbox (c1 for legacy, f5/f6 for SC) needs the responder's own
 * address exactly as the peer sees it, including the address TYPE
 * (0 = public, 1 = random). A wrong address or type makes every confirm /
 * check value mismatch and pairing fails with CONFIRM_VALUE / DHKEY_CHECK.
 *
 * Weak default returns public, all-zero - which is almost never correct, so
 * the active implementation MUST override this. The SDC implementation reports the random
 * static address it set at init.
 *
 * @param	pType	out: address type (0 public, 1 random).
 * @param	pAddr	out: 6-byte address, little-endian as on air.
 */
void BtSmpLocalAddrGet(uint8_t *pType, uint8_t pAddr[6]);

/**
 * @brief	f4 self-test against the BLE spec worked example. Returns 0 on PASS.
 *			For bring-up verification that the on-target f4/AES-CMAC are correct.
 */
int BtSmpF4SelfTest(void);

// Resolve a resolvable private address (6-byte BD_ADDR) against a peer IRK.
bool BtSmpRpaResolve(const uint8_t Irk[16], const uint8_t Rpa[6]);

// ah / RPA-resolution self-test against the spec sample. Returns 0 on PASS.
int BtSmpRpaSelfTest(void);

// Compute the 8-byte signed-write MAC over pMsg (signed data || SignCounter, wire
// order) with the little-endian Csrk. Mac is written little-endian.
void BtSmpSignMac(const uint8_t Csrk[16], const uint8_t *pMsg, size_t Len, uint8_t Mac[8]);

// Verify a signed-write signature for the peer on ConnHdl. pMsg is the signed
// message (opcode || handle || value || SignCounter) in wire order; pSig is the
// 12-byte signature. Advances the stored SignCounter on success.
bool BtSmpSignVerify(uint16_t ConnHdl, const uint8_t *pMsg, size_t MsgLen, const uint8_t *pSig);

// Signed-write MAC self-test (byte-order check). Returns 0 on PASS.
int BtSmpSignSelfTest(void);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_SMP_H__
