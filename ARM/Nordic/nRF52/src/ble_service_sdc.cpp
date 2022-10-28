/**-------------------------------------------------------------------------
@file	ble_service_sdc.c

@brief	Implement Bluetooth LE service and characteristic

Implementation allow the creation of generic custom Bluetooth Smart service
with multiple user defined characteristics.

This implementation is to be used with Nordic nrfxlib Softdevice Controller

@author	Hoang Nguyen Hoan
@date	Oct. 15, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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


#include "bluetooth/ble_srvc.h"
#include "bluetooth/bt_gatt.h"
#if 0
/** @def BT_GATT_ATTRIBUTE
 *  @brief Attribute Declaration Macro.
 *
 *  Helper macro to declare an attribute.
 *
 *  @param _uuid Attribute uuid.
 *  @param _perm Attribute access permissions,
 *               a bitmap of @ref bt_gatt_perm values.
 *  @param _read Attribute read callback (@ref bt_gatt_attr_read_func_t).
 *  @param _write Attribute write callback (@ref bt_gatt_attr_write_func_t).
 *  @param _user_data Attribute user data.
 */
#define BT_GATT_ATTRIBUTE(_uuid, _perm, _read, _write, _user_data)	\
{									\
	.uuid = _uuid,							\
	.read = _read,							\
	.write = _write,						\
	.user_data = _user_data,					\
	.handle = 0,							\
	.perm = _perm,							\
}

/** @def BT_GATT_SERVICE_DEFINE
 *  @brief Statically define and register a service.
 *
 *  Helper macro to statically define and register a service.
 *
 *  @param _name Service name.
 */
#define BT_GATT_SERVICE_DEFINE(_name, ...)				\
	const struct bt_gatt_attr attr_##_name[] = { __VA_ARGS__ };	\
	const STRUCT_SECTION_ITERABLE(bt_gatt_service_static, _name) =	\
					BT_GATT_SERVICE(attr_##_name)

#define _BT_GATT_ATTRS_ARRAY_DEFINE(n, _instances, _attrs_def)	\
	static struct bt_gatt_attr attrs_##n[] = _attrs_def(_instances[n])

#define _BT_GATT_SERVICE_ARRAY_ITEM(_n, _) BT_GATT_SERVICE(attrs_##_n)

/** @def BT_GATT_CHARACTERISTIC
 *  @brief Characteristic and Value Declaration Macro.
 *
 *  Helper macro to declare a characteristic attribute along with its
 *  attribute value.
 *
 *  @param _uuid Characteristic attribute uuid.
 *  @param _props Characteristic attribute properties,
 *                a bitmap of BT_GATT_CHRC_* macros.
 *  @param _perm Characteristic Attribute access permissions,
 *               a bitmap of @ref bt_gatt_perm values.
 *  @param _read Characteristic Attribute read callback
 *               (@ref bt_gatt_attr_read_func_t).
 *  @param _write Characteristic Attribute write callback
 *                (@ref bt_gatt_attr_write_func_t).
 *  @param _user_data Characteristic Attribute user data.
 */
#define BT_GATT_CHARACTERISTIC(_uuid, _props, _perm, _read, _write, _user_data) \
	BT_GATT_ATTRIBUTE(BT_UUID_GATT_CHRC, BT_GATT_PERM_READ,                 \
			  bt_gatt_attr_read_chrc, NULL,                         \
			  ((struct bt_gatt_chrc[]) {                            \
				BT_GATT_CHRC_INIT(_uuid, 0U, _props),           \
						   })),                         \
	BT_GATT_ATTRIBUTE(_uuid, _perm, _read, _write, _user_data)

#if defined(CONFIG_BT_SETTINGS_CCC_LAZY_LOADING)
	#define BT_GATT_CCC_MAX (CONFIG_BT_MAX_CONN)
#elif defined(CONFIG_BT_CONN)
	#define BT_GATT_CCC_MAX (CONFIG_BT_MAX_PAIRED + CONFIG_BT_MAX_CONN)
#else
	#define BT_GATT_CCC_MAX 0
#endif

/** @brief GATT CCC configuration entry. */
struct bt_gatt_ccc_cfg {
	/** Local identity, BT_ID_DEFAULT in most cases. */
	uint8_t id;
	/** Remote peer address. */
	bt_addr_le_t peer;
	/** Configuration value. */
	uint16_t value;
};


BT_GATT_SERVICE_DEFINE(nus_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX,
			       BT_GATT_CHRC_WRITE |
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       NULL, on_receive, NULL),
);
#endif

uint32_t BleSrvcCharNotify(BleSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	return 0;
}

uint32_t BleSrvcAddChar(BleSrvc_t *pSrvc, BleSrvcChar_t *pChar, uint32_t SecType)
{
	return 0;
}

uint32_t BleSrvcInit(BleSrvc_t *pSrvc, const BleSrvcCfg_t *pCfg)
{
	uint32_t   err;
	uint8_t baseidx = 0;
	//ble_uuid_t ble_uuid;

	// Initialize service structure
	pSrvc->ConnHdl  = -1;
	pSrvc->UuidSvc = pCfg->UuidSvc;

	// Add base UUID to softdevice's internal list.
	//for (int i = 0; i < pCfg->NbUuidBase; i++)
	//{
	if (pCfg->bCustom)
	{
		baseidx = BtUuidAddBase(pCfg->UuidBase);
	}
	//}

//	BtGattSrvc_t sv = {
//		0xFFFF,
//		{ 1, BT_UUID_TYPE_16, pCfg->UuidSvc},
//	};

	BtUuid16_t uid16 = { baseidx, BT_UUID_TYPE_16, pCfg->UuidSvc};

	BtUuid16_t TypeUuid = { 0, BT_UUID_TYPE_16, BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE };

	pSrvc->Hdl = BtGattRegister(&TypeUuid, &uid16);

	pSrvc->NbChar = pCfg->NbChar;
    pSrvc->pCharArray = pCfg->pCharArray;

    for (int i = 0; i < pCfg->NbChar; i++)
    {
    	uid16.Uuid = pSrvc->pCharArray[i].Uuid;
    	pSrvc->pCharArray[i].BaseUuidIdx = baseidx;

    	BtGattCharDeclar_t gatt = {(uint8_t)pSrvc->pCharArray[i].Property, 0, {pSrvc->pCharArray[i].BaseUuidIdx, BT_UUID_TYPE_16, pSrvc->pCharArray[i].Uuid}};

    	TypeUuid.Uuid = BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC;
    	pSrvc->pCharArray[i].Hdl = BtGattRegister(&TypeUuid, &gatt);
    	pSrvc->pCharArray[i].ValHdl = gatt.ValHdl;

        pSrvc->pCharArray[i].bNotify = false;
        if (pSrvc->pCharArray[i].Property & BLESRVC_CHAR_PROP_NOTIFY)
        {
        	TypeUuid.Uuid = BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
			pSrvc->pCharArray[i].CccdHdl = BtGattRegister(&TypeUuid, &uid16);
        }
    }

/*
	    ble_uuid.type = pSrvc->UuidType[0];
	    ble_uuid.uuid = pCfg->UuidSvc;

	    err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &pSrvc->SrvcHdl);
	    if (err != NRF_SUCCESS)
	    {
	        return err;
	    }

	    pSrvc->NbChar = pCfg->NbChar;

	    pSrvc->pCharArray = pCfg->pCharArray;

	    for (int i = 0; i < pCfg->NbChar; i++)
	    {
	    	err = BlueIOBleSrvcCharAdd(pSrvc, &pSrvc->pCharArray[i],
									   pCfg->SecType);
	        if (err != NRF_SUCCESS)
	        {
	            return err;
	        }
	        pSrvc->pCharArray[i].bNotify = false;
	    }

	    pSrvc->pLongWrBuff = pCfg->pLongWrBuff;
	    pSrvc->LongWrBuffSize = pCfg->LongWrBuffSize;
	    pSrvc->AuthReqCB = pCfg->AuthReqCB;

	    return NRF_SUCCESS;*/

	return 0;

}
