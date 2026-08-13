#ifndef __BT_TEST_NRF_ERROR_H__
#define __BT_TEST_NRF_ERROR_H__

#define NRF_SUCCESS			0U
#define NRF_ERROR_INTERNAL		3U
#define NRF_ERROR_NO_MEM		4U
#define NRF_ERROR_NOT_FOUND		5U
#define NRF_ERROR_INVALID_PARAM		7U
#define NRF_ERROR_INVALID_STATE	8U
#define NRF_ERROR_TIMEOUT		13U
#define NRF_ERROR_NULL			14U
#define NRF_ERROR_FORBIDDEN		15U
#define NRF_ERROR_BUSY		17U

// FDS reports a full flash through the peer data layer with this code.
#define NRF_ERROR_STORAGE_FULL		0x8000U

#endif
