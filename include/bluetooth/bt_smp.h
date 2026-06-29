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
#include "crypto/crypto.h"

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
#define BT_SMP_MIN_ENC_KEY_SIZE						7

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
	uint8_t  Mackey[16];			//!< SC: MacKey from f5
	uint8_t  Ltk[16];				//!< Derived/working LTK
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
 * @brief	Initialise the SMP layer and compose its crypto from engines.
 *
 * SMP needs two composed primitives - ECDH (P-256) and AES-128 ECB - each
 * supplied by a CryptoDev_t engine (crypto/crypto.h).
 * The application passes the engine that fills each slot from whatever its
 * target provides: one engine may fill both (mbedtls, CC310), or they compose
 * from two (e.g. uECC for ECDH, the BLE controller for AES, on a part with no
 * CryptoCell). Each slot is validated against the required capability; an
 * engine lacking it leaves that slot disabled and the corresponding operation
 * fails loud. The same engine pointer may be passed for both slots.
 *
 * Randomness is NOT a slot: it comes from the target RngGet utility
 * (coredev/rng.h) - hardware where the MCU has an RNG peripheral, weak software
 * default otherwise - which SMP calls directly.
 *
 * @param	pEcdh	Engine providing CRYPTO_CAP_ECDH_P256.
 * @param	pAes	Engine providing CRYPTO_CAP_AES128_ECB.
 */
void BtSmpInit(CryptoDev_t *pEcdh, CryptoDev_t *pAes);

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

/**
 * @brief	Bring up the Bluetooth-owned controller CryptoDev_t (SDC).
 *
 * Populates a caller-owned CryptoDev_t whose AES-128 ECB is served by the BLE
 * SoftDevice Controller's HCI LE Encrypt. This is a Bluetooth helper, NOT a
 * generic crypto engine - it requires a running BLE controller and is only
 * valid for the SMP AES slot. On a part with no CryptoCell, compose it with a
 * software ECDH engine (randomness comes from the RngGet utility):
 *
 *   CryptoDev_t ecdh, aes;
 *   CryptoUeccInit(&ecdh);
 *   BtCryptoCtlrSdcInit(&aes);   // controller AES
 *   BtSmpInit(&ecdh, &aes);
 *
 * @return	true on success, false if the SDC HCI is not present in this build.
 */
bool BtCryptoCtlrSdcInit(CryptoDev_t * const pDev);

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
 * @brief	Backend hooks to answer the controller LE Long Term Key Request.
 *			These send HCI COMMANDS (not ACL data); the active backend
 *			overrides them to use its real HCI command channel (e.g. the SDC
 *			sdc_hci_cmd_le_long_term_key_request_reply function).
 */
void BtSmpHciLtkReply(BtHciDevice_t * const pDev, uint16_t ConnHdl,
					  const uint8_t Ltk[16]);
void BtSmpHciLtkNegReply(BtHciDevice_t * const pDev, uint16_t ConnHdl);

/**
 * @brief	Backend hook to start link encryption as the central via HCI LE
 *			Enable Encryption. Sends an HCI COMMAND (not ACL data); the active
 *			backend overrides it to use its real HCI command channel (e.g. the
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
 * @brief	Access functions for a platform persistence backend.
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
 * the active backend MUST override this. The SDC backend reports the random
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
