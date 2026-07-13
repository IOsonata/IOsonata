/*
 * Copyright (c) 2015-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrf_error.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>
#include <modules/peer_manager_internal.h>
#include <modules/peer_id.h>
#include <modules/peer_data_storage.h>

#include "bluetooth/bt_pds.h"

extern int BtPdsBmInit(void);

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

#define PDS_EVENT_HANDLERS_CNT ARRAY_SIZE(evt_handlers)
#define ENTRY_ID_PEER_ID_OFFSET_BITS 16U
#define ENTRY_ID_DATA_ID_MASK ((1UL << ENTRY_ID_PEER_ID_OFFSET_BITS) - 1UL)

extern void pdb_pds_evt_handler(struct pm_evt *evt);

static const pm_evt_handler_internal_t evt_handlers[] = {
	pdb_pds_evt_handler,
};

static bool module_initialized;

static void pds_evt_send(struct pm_evt *event)
{
	event->conn_handle = BLE_CONN_HANDLE_INVALID;

	for (uint32_t i = 0; i < PDS_EVENT_HANDLERS_CNT; i++) {
		evt_handlers[i](event);
	}
}

static uint32_t peer_record_id(uint16_t peer_id, enum pm_peer_data_id data_id)
{
	return ((uint32_t)peer_id << ENTRY_ID_PEER_ID_OFFSET_BITS) |
		((uint32_t)data_id & ENTRY_ID_DATA_ID_MASK);
}

static bool peer_data_id_is_valid(enum pm_peer_data_id data_id)
{
	return data_id == PM_PEER_DATA_ID_BONDING ||
		data_id == PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING ||
		data_id == PM_PEER_DATA_ID_GATT_LOCAL ||
		data_id == PM_PEER_DATA_ID_GATT_REMOTE ||
		data_id == PM_PEER_DATA_ID_PEER_RANK ||
		data_id == PM_PEER_DATA_ID_CENTRAL_ADDR_RES ||
		data_id == PM_PEER_DATA_ID_APPLICATION;
}

static void peer_ids_restore(void)
{
	uint8_t data[PM_PEER_DATA_MAX_SIZE];

	peer_id_init();

	for (uint16_t peer_id = 0; peer_id < PM_PEER_ID_N_AVAILABLE_IDS; peer_id++) {
		ssize_t len = BtPdsRead(peer_record_id(peer_id, PM_PEER_DATA_ID_BONDING),
			data, sizeof(data));

		if (len > 0) {
			(void)peer_id_allocate(peer_id);
		} else if (len < 0 && len != -ENOENT) {
			LOG_ERR("Could not restore peer id %u. BtPdsRead() returned %d.",
				peer_id, (int)len);
		}
	}
}

static uint32_t peer_delete_all(uint16_t peer_id)
{
	for (enum pm_peer_data_id data_id = 0;
		data_id < PM_PEER_DATA_ID_LAST;
		data_id++) {
		if (!peer_data_id_is_valid(data_id)) {
			continue;
		}

		int err = BtPdsDelete(peer_record_id(peer_id, data_id));
		if (err < 0) {
			LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d. "
				"peer_id: %u, data_id: %u.", err, peer_id, data_id);
			return NRF_ERROR_INTERNAL;
		}
	}

	peer_id_free(peer_id);
	return NRF_SUCCESS;
}

void pds_peer_data_iterate_prepare(uint16_t *peer_id_iter)
{
	__ASSERT_NO_MSG(peer_id_iter != NULL);
	*peer_id_iter = 0;
}

bool pds_peer_data_iterate(enum pm_peer_data_id data_id,
				   uint16_t *const peer_id,
				   struct pm_peer_data_const *const data,
				   uint16_t *peer_id_iter)
{
	uint8_t temp_buf[PM_PEER_DATA_MAX_SIZE];
	ssize_t ret;

	__ASSERT_NO_MSG(peer_id != NULL);
	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(data->all_data != NULL);
	__ASSERT_NO_MSG(peer_id_iter != NULL);

	if (!peer_data_id_is_valid(data_id) ||
		*peer_id_iter >= PM_PEER_ID_N_AVAILABLE_IDS) {
		return false;
	}

	do {
		uint16_t current = *peer_id_iter;
		ret = BtPdsRead(peer_record_id(current, data_id),
			temp_buf, sizeof(temp_buf));
		(*peer_id_iter)++;

		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Could not iterate peer data. BtPdsRead() returned %d. "
				"peer_id: %u, data_id: %u.", (int)ret, current, data_id);
			return false;
		}
	} while (ret == -ENOENT && *peer_id_iter < PM_PEER_ID_N_AVAILABLE_IDS);

	if (ret == -ENOENT) {
		return false;
	}

	*peer_id = (uint16_t)(*peer_id_iter - 1U);
	memcpy((void *)data->all_data, temp_buf, (size_t)ret);
	return true;
}

uint32_t pds_init(void)
{
	__ASSERT_NO_MSG(!module_initialized);

	int err = BtPdsBmInit();
	if (err != 0) {
		LOG_ERR("Could not initialize NVM storage. BtPdsBmInit() returned %d.", err);
		return NRF_ERROR_RESOURCES;
	}

	peer_ids_restore();
	module_initialized = true;
	return NRF_SUCCESS;
}

uint32_t pds_peer_data_read(uint16_t peer_id,
				    enum pm_peer_data_id data_id,
				    struct pm_peer_data *const data,
				    const uint32_t *const buf_len)
{
	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(buf_len != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_data_id_is_valid(data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	ssize_t ret = BtPdsRead(peer_record_id(peer_id, data_id),
		data->all_data, *buf_len);

	if (ret == -ENOENT) {
		return NRF_ERROR_NOT_FOUND;
	}
	if (ret < 0) {
		LOG_ERR("Could not read peer data. BtPdsRead() returned %d. "
			"peer_id: %u, data_id: %u.", (int)ret, peer_id, data_id);
		return NRF_ERROR_INTERNAL;
	}
	if ((uint32_t)ret > *buf_len) {
		return NRF_ERROR_DATA_SIZE;
	}

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_store(uint16_t peer_id,
				     const struct pm_peer_data_const *peer_data,
				     uint32_t *store_token)
{
	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(peer_data != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_data_id_is_valid(peer_data->data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t entry_id = peer_record_id(peer_id, peer_data->data_id);
	ssize_t ret = BtPdsWrite(entry_id, peer_data->all_data, peer_data->length);

	if (ret < 0) {
		LOG_ERR("Could not store peer data. BtPdsWrite() returned %d. "
			"peer_id: %u, data_id: %u.",
			(int)ret, peer_id, peer_data->data_id);
		return ret == -ENOMEM ? NRF_ERROR_RESOURCES : NRF_ERROR_INTERNAL;
	}

	if (store_token != NULL) {
		*store_token = entry_id;
	}

	struct pm_evt event = {
		.evt_id = PM_EVT_PEER_DATA_UPDATE_SUCCEEDED,
		.peer_id = peer_id,
	};
	event.peer_data_update_succeeded.data_id = peer_data->data_id;
	event.peer_data_update_succeeded.action = PM_PEER_DATA_OP_UPDATE;
	event.peer_data_update_succeeded.token = entry_id;
	event.peer_data_update_succeeded.flash_changed = true;
	pds_evt_send(&event);

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_delete(uint16_t peer_id, enum pm_peer_data_id data_id)
{
	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_data_id_is_valid(data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t entry_id = peer_record_id(peer_id, data_id);
	int err = BtPdsDelete(entry_id);

	if (err < 0) {
		LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d. "
			"peer_id: %u, data_id: %u.", err, peer_id, data_id);
		return err == -ENOMEM ? NRF_ERROR_RESOURCES : NRF_ERROR_INTERNAL;
	}

	struct pm_evt event = {
		.evt_id = PM_EVT_PEER_DATA_UPDATE_SUCCEEDED,
		.peer_id = peer_id,
	};
	event.peer_data_update_succeeded.data_id = data_id;
	event.peer_data_update_succeeded.action = PM_PEER_DATA_OP_DELETE;
	event.peer_data_update_succeeded.token = entry_id;
	event.peer_data_update_succeeded.flash_changed = true;
	pds_evt_send(&event);

	return NRF_SUCCESS;
}

/*
 * Temporary ABI compatibility for the stock peer_database module.
 * New IOsonata code must call peer_id_* directly. These wrappers disappear
 * when peer_database.c is replaced.
 */
uint16_t pds_peer_id_allocate(void)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_allocate(PM_PEER_ID_INVALID);
}

uint32_t pds_peer_id_free(uint16_t peer_id)
{
	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_id_is_allocated(peer_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t status = peer_delete_all(peer_id);
	struct pm_evt event = {
		.evt_id = status == NRF_SUCCESS ?
			PM_EVT_PEER_DELETE_SUCCEEDED : PM_EVT_PEER_DELETE_FAILED,
		.peer_id = peer_id,
	};

	if (status != NRF_SUCCESS) {
		event.peer_delete_failed.error = status;
	}
	pds_evt_send(&event);
	return status;
}

bool pds_peer_id_is_allocated(uint16_t peer_id)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_is_allocated(peer_id);
}

bool pds_peer_id_is_deleted(uint16_t peer_id)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_is_deleted(peer_id);
}

uint16_t pds_next_peer_id_get(uint16_t prev_peer_id)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_get_next_used(prev_peer_id);
}

uint16_t pds_next_deleted_peer_id_get(uint16_t prev_peer_id)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_get_next_deleted(prev_peer_id);
}

uint32_t pds_peer_count_get(void)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_n_ids();
}
