/**-------------------------------------------------------------------------
@file	bt_dev.h

@brief	Bluetooth device representation. Same struct for local stack
		identity and for tracked remote peers. Operations differ by who
		drives them, factored into separate function namespaces
		(BtApp*() for app driven, BtDevice*() for shared queries).

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BT_DEV_H__
#define __BT_DEV_H__

#include <stdint.h>
#include <stdbool.h>

// "device.h" is included for compatibility - the old bt_dev.h pulled it in,
// and several ports (notably the BM nRF54 port) rely on the transitive
// inclusion of coredev/timer.h, coredev/iopincfg.h, and device_intrf.h
// that comes from this header. Removing it would force every dependent
// port to add explicit includes.
#include "device.h"

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"

/** @addtogroup Bluetooth
  * @{
  */

#define BT_DEV_NAME_MAXLEN			30
#define BT_DEV_SERVICE_MAXCNT		10
#define BT_DEV_TXPEND_MAX			8		//!< Depth of the per-link notify/indicate TX-complete ring

/// Per-peer discovery state. Used by the central while walking the peer's
/// GATT table. Previously held as file-scope globals (g_CurIdx, g_UuidType)
/// in bt_attrsp.cpp / bt_attreq.cpp, which broke multi-link discovery: the
/// cursor for one connection's response would clobber another connection's
/// in-flight state.
///
/// Hdl is uint16_t because ATT handles are 16-bit; the prior CurParseInf_t
/// typedef stored it as uint8_t and silently truncated peer handles >= 256.
typedef struct __Bt_Dev_Disc_State {
	uint8_t			SrvIdx;			//!< Index into Services[] currently being parsed
	uint8_t			CharIdx;		//!< Index into the current service's char array
	uint16_t		Hdl;			//!< Current ATT handle being read/queried
	BtUuid_t		UuidType;		//!< UUID type the state machine is scanning for
} BtDevDiscState_t;

/// Bluetooth device representation. One instance for the local stack's
/// own identity, one instance per tracked remote peer. Same structure
/// in both cases. Role asymmetry lives in the operations (BtApp*() for
/// app driven, BtDevice*() for shared queries), not in the type.
///
/// Field occupancy by role:
///   local  : Name = own GAP name, Conn.PeerAddr = own BD_ADDR, Conn.Role =
///            bitmask of GAP roles this stack takes, Services = exposed GATT
///            DB, Conn.Hdl = unused, pHciDev = the controller in use.
///   remote : Name = peer's reported name, Conn.PeerAddr = peer's BD_ADDR,
///            Conn.Role = LL role peer plays on the active link,
///            Services = discovered GATT DB, Conn.Hdl = active link handle,
///            pHciDev = the local controller managing this link.
typedef struct __Bt_Device {
	BtGapConnection_t Conn;						//!< Per-link state (base). bt_dev owns it; &Conn is passed down to GATT/GAP. Holds Hdl/Role/PeerAddr/MaxMtu/long-write.
	char			Name[BT_DEV_NAME_MAXLEN];	//!< Device name
	uint16_t		Appearance;					//!< GAP appearance value
	uint16_t		VendorId;					//!< PnP vendor ID
	uint16_t		ProductId;					//!< PnP product ID
	uint16_t		ProductVer;					//!< PnP product version
	bool			bIsLocal;					//!< true for the local stack instance, false for tracked remotes
	bool			bSecure;					//!< true if link is encrypted or device is bonded
	BtHciDevice_t	*pHciDev;					//!< Associated HCI device
	int				NbSrvc;						//!< Number of services in the Services array
	BtGattDBSrvc_t	Services[BT_DEV_SERVICE_MAXCNT];	//!< Services: exposed if local, discovered if remote
	BtDevDiscState_t Discovery;					//!< Per-peer discovery cursor (remote role only)
	void			*TxPendCh[BT_DEV_TXPEND_MAX];//!< Ring of chars with a notification/indication in flight (native-host TX-complete attribution)
	uint8_t			TxPendHead;					//!< Ring read index
	uint8_t			TxPendCount;				//!< Ring occupancy
} BtDevice_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Weak callback invoked when GATT discovery on a peer completes.
 *
 * The discovery flow populates pDev->Services with the discovered
 * GATT DB and then calls this. Applications override to handle the
 * completion (typically by calling BtDeviceFindService and
 * BtDeviceFindCharacteristic to navigate the result).
 *
 * The pointer references stack managed memory. Applications that need
 * to retain values across return should copy what they need.
 *
 * @param	pDev	Pointer to the BtDevice_t whose discovery completed.
 */
void BtDeviceDiscovered(BtDevice_t *pDev);

/**
 * @brief	Find a service in a device's Services array by 16 bit UUID.
 *
 * Works on any BtDevice_t (the Services array may be locally exposed
 * or remotely discovered, the walk is the same).
 *
 * @param	pDev	Pointer to the BtDevice_t.
 * @param	Uuid	16 bit service UUID.
 *
 * @return	Index of the matching service in pDev->Services,
 *			or -1 if not found.
 */
int BtDeviceFindService(BtDevice_t * const pDev, uint16_t Uuid);

/**
 * @brief	Find a characteristic within a service by 16 bit UUID.
 *
 * @param	pDev		Pointer to the BtDevice_t.
 * @param	SrvcIdx		Service index in pDev->Services (from BtDeviceFindService).
 * @param	Uuid		16 bit characteristic UUID.
 *
 * @return	Index of the matching characteristic, or -1 if not found.
 */
int BtDeviceFindCharacteristic(BtDevice_t * const pDev, int SrvcIdx, uint16_t Uuid);

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BT_DEV_H__
