/*
 * Copyright (c) 2015-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrf_error.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/__assert.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>
#include <modules/peer_manager_internal.h>
#include <modules/peer_id.h>
#include <modules/peer_data_storage.h>

// IOsonata persistent data store replaces the bm_zms filesystem. The store
// owns its region constants in its platform backend (BtPdsBmInit), so no
// devicetree partition macros are needed here.
#include "bluetooth/bt_pds.h"

// Platform backend init (e.g. BtPdsBmInit for the bm RRAM backend). Declared
// here; defined by the target's bt_pds_nvm_*.c.
extern int BtPdsBmInit(void);

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

/* The number of registered event handlers. */
#define PDS_EVENT_HANDLERS_CNT ARRAY_SIZE(evt_handlers)

/* Peer Data Storage event handler in Peer Database. */
extern void pdb_pds_evt_handler(struct pm_evt *evt);

/* Peer Data Storage events' handlers.
 * The number of elements in this array is PDS_EVENT_HANDLERS_CNT.
 */
static const pm_evt_handler_internal_t evt_handlers[] = {
	pdb_pds_evt_handler,
};

static bool module_initialized;
static volatile bool peer_delete_deferred;

/* Keeps track of the number of peers currently under delete processing. */
static atomic_t delete_counter;

/* Function for dispatching events to all registered event handlers. */
static void pds_evt_send(struct pm_evt *event)
{
	event->conn_handle = BLE_CONN_HANDLE_INVALID;

	for (uint32_t i = 0; i < PDS_EVENT_HANDLERS_CNT; i++) {
		evt_handlers[i](event);
	}
}

#define ENTRY_ID_PEER_ID_OFFSET_BITS 16
#define ENTRY_ID_DATA_ID_MASK        ((1 << ENTRY_ID_PEER_ID_OFFSET_BITS) - 1)

/**
 * @brief Pack the given peer_id and data_id into a single 32 bit entry_id.
 *
 * @p peer_id is stored in the most significant 16 bits.
 * @p data_id is stored in the least significant 16 bits.
 */
static uint32_t peer_id_peer_data_id_to_entry_id(uint16_t peer_id, enum pm_peer_data_id data_id)
{
	return (peer_id << ENTRY_ID_PEER_ID_OFFSET_BITS) | (data_id & ENTRY_ID_DATA_ID_MASK);
}

static bool peer_data_id_is_valid(enum pm_peer_data_id data_id)
{
	return ((data_id == PM_PEER_DATA_ID_BONDING) ||
		(data_id == PM_PEER_DATA_ID_SERVICE_CHANGED_PENDING) ||
		(data_id == PM_PEER_DATA_ID_GATT_LOCAL) ||
		(data_id == PM_PEER_DATA_ID_GATT_REMOTE) ||
		(data_id == PM_PEER_DATA_ID_PEER_RANK) ||
		(data_id == PM_PEER_DATA_ID_CENTRAL_ADDR_RES) ||
		(data_id == PM_PEER_DATA_ID_APPLICATION));
}

/* Returns the next data entry or a negative errno. */
static uint32_t find_next_data_entry_in_peer(uint16_t peer_id, uint32_t *next_entry_id)
{
	ssize_t ret;
	uint8_t temp_buf[PM_PEER_DATA_MAX_SIZE] = { 0 };

	for (enum pm_peer_data_id i = 0; i < PM_PEER_DATA_ID_LAST; i++) {
		uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, i);

		ret = BtPdsRead(entry_id, temp_buf, sizeof(temp_buf));
		/* Unexpected error. */
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Could not read entry %d from NVM. bm_zms_read() returned %d. "
				"peer_id: %d, data_id: %d", entry_id, ret, peer_id, i);
			return NRF_ERROR_INTERNAL;
		}

		/* Some peer data has been found. */
		if (ret > 0) {
			*next_entry_id = entry_id;
			return NRF_SUCCESS;
		}
	}

	/* Every data read for the peer has returned `-ENOENT`. */
	return NRF_ERROR_NOT_FOUND;
}

/* Function for deleting all data belonging to deleted peers.
 *
 * With the synchronous IOsonata store each delete completes immediately, so the
 * whole-peer delete sequence that the old bm_zms path drove through its async
 * completion callback is performed here in a straight loop: for each deleted
 * peer, delete every data entry, then free the peer id and emit
 * PM_EVT_PEER_DELETE_SUCCEEDED.
 */
static void peer_data_delete_process(void)
{
	int err;
	uint16_t peer_id;
	uint32_t entry_id;

	peer_delete_deferred = false;

	peer_id = peer_id_get_next_deleted(PM_PEER_ID_INVALID);

	while (peer_id != PM_PEER_ID_INVALID) {

		/* Delete every remaining data entry for this peer. */
		while (find_next_data_entry_in_peer(peer_id, &entry_id) == NRF_SUCCESS) {
			err = BtPdsDelete(entry_id);
			if (err == -ENOMEM) {
				/* Store full mid-delete: defer and retry later. */
				peer_delete_deferred = true;
				return;
			} else if (err < 0) {
				LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d "
					"for peer_id: %d", err, peer_id);
				atomic_dec(&delete_counter);
				struct pm_evt fail_evt = {
					.evt_id = PM_EVT_PEER_DELETE_FAILED,
					.peer_id = peer_id,
				};
				fail_evt.peer_delete_failed.error = NRF_ERROR_INTERNAL;
				pds_evt_send(&fail_evt);
				break;
			}
		}

		/* All entries gone (or aborted on error): free the id and report. */
		atomic_dec(&delete_counter);

		struct pm_evt done_evt = {
			.evt_id = PM_EVT_PEER_DELETE_SUCCEEDED,
			.peer_id = peer_id,
		};
		peer_id_free(peer_id);
		pds_evt_send(&done_evt);

		peer_id = peer_id_get_next_deleted(peer_id);
	}
}

static void peer_ids_load(void)
{
	uint16_t peer_id;
	uint16_t peer_id_iter;
	struct pm_peer_data_const peer_data = { 0 };
	uint8_t peer_data_buffer[PM_PEER_DATA_MAX_SIZE] = { 0 };

	peer_data.all_data = peer_data_buffer;

	/* Search through existing bonds to look for a duplicate. */
	pds_peer_data_iterate_prepare(&peer_id_iter);

	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peer_id, &peer_data,
		&peer_id_iter)) {
		(void)peer_id_allocate(peer_id);
	}
}

void pds_peer_data_iterate_prepare(uint16_t *peer_id_iter)
{
	*peer_id_iter = 0;
}

bool pds_peer_data_iterate(enum pm_peer_data_id data_id, uint16_t *const peer_id,
			   struct pm_peer_data_const *const data, uint16_t *peer_id_iter)
{
	ssize_t ret;
	uint8_t temp_buf[PM_PEER_DATA_MAX_SIZE] = { 0 };

	if (*peer_id_iter >= PM_PEER_ID_N_AVAILABLE_IDS) {
		return false;
	}

	/* Exits the loop when `ret > 0` (it found data), it reached the end of the available peers,
	 * or the read had a catastrophical failure.
	 */
	do {
		uint32_t entry_id = peer_id_peer_data_id_to_entry_id(*peer_id_iter, data_id);

		ret = BtPdsRead(entry_id, temp_buf, sizeof(temp_buf));
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Could not read data from NVM. bm_zms_read() returned %d. "
				"peer_id: %d",
				ret, *peer_id_iter);
			return false;
		}

		(*peer_id_iter)++;
	} while ((ret == -ENOENT) && (*peer_id_iter < PM_PEER_ID_N_AVAILABLE_IDS));

	if ((ret == -ENOENT) && (*peer_id_iter == PM_PEER_ID_N_AVAILABLE_IDS)) {
		return false;
	}

	/* We found a suitable Peer ID. */

	/* `p_peer_id_iter` counts the iterations, so the Peer ID is iterations minus one. */
	*peer_id = (*peer_id_iter) - 1;

	/* `ret` is equal the exact amount of data contained in the entry, so copy that amount
	 * safely.
	 */
	memcpy((void *)data->all_data, temp_buf, ret);

	return true;
}

uint32_t pds_init(void)
{
	int err;

	/* Check for re-initialization if debugging. */
	__ASSERT_NO_MSG(!module_initialized);

	/* Initialize the IOsonata persistent data store. The platform backend
	 * (BtPdsBmInit on the bm port) owns the NVM region constants. This is
	 * synchronous: on return the store is mounted and scanned.
	 */
	err = BtPdsBmInit();
	if (err) {
		LOG_ERR("Could not initialize NVM storage. BtPdsBmInit() returned %d.", err);
#ifdef BT_PDS_TRACE
		{
			extern void BtPdsTraceOut(const char *pStr);
			char b[64];
			snprintf(b, sizeof(b), "pds_init: BtPdsBmInit failed %d\r\n", err);
			BtPdsTraceOut(b);
		}
#endif
		return NRF_ERROR_RESOURCES;
	}

	peer_id_init();
	peer_ids_load();

	module_initialized = true;

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_read(uint16_t peer_id, enum pm_peer_data_id data_id,
			    struct pm_peer_data *const data, const uint32_t *const buf_len)
{
	ssize_t ret;

	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(buf_len != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS || !peer_data_id_is_valid(data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, data_id);

	ret = BtPdsRead(entry_id, data->all_data, *buf_len);
	if (ret == -ENOENT) {
		LOG_DBG("Could not read entry %d. bm_zms_read() returned %d. "
			"peer_id: %d, data_id: %d", entry_id,
			ret, peer_id, data_id);
		return NRF_ERROR_NOT_FOUND;
	} else if (ret < 0) {
		LOG_ERR("Could not read data from NVM. bm_zms_read() returned %d. "
			"peer_id: %d",
			ret, peer_id);
		return NRF_ERROR_INTERNAL;
	}

	if (*buf_len < ret) {
		return NRF_ERROR_DATA_SIZE;
	}

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_store(uint16_t peer_id, const struct pm_peer_data_const *peer_data,
			     uint32_t *store_token)
{
	ssize_t ret;

	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(peer_data != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS || !peer_data_id_is_valid(peer_data->data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, peer_data->data_id);

	ret = BtPdsWrite(entry_id, peer_data->all_data, peer_data->length);
#ifdef BT_PDS_TRACE
	{
		extern void BtPdsTraceOut(const char *pStr);
		char b[80];
		snprintf(b, sizeof(b), "pds_store: entry=0x%lx len=%u BtPdsWrite=%d\r\n",
				 (unsigned long)entry_id, (unsigned)peer_data->length, ret);
		BtPdsTraceOut(b);
	}
#endif
	if (ret < 0) {
		LOG_ERR("Could not write data to NVM. BtPdsWrite() returned %d. "
			"peer_id: %d",
			ret, peer_id);
		return NRF_ERROR_INTERNAL;
	}

	if (store_token != NULL) {
		/* Update the store token. */
		*store_token = entry_id;
	}

	/* The store is synchronous: the write is committed now, so emit the
	 * peer-manager completion event inline (the old bm_zms path delivered this
	 * asynchronously through bm_zms_evt_handler).
	 */
	{
		struct pm_evt pds_evt = {
			.evt_id = PM_EVT_PEER_DATA_UPDATE_SUCCEEDED,
			.peer_id = peer_id,
		};
		pds_evt.peer_data_update_succeeded.data_id = peer_data->data_id;
		pds_evt.peer_data_update_succeeded.action = PM_PEER_DATA_OP_UPDATE;
		pds_evt.peer_data_update_succeeded.token = entry_id;
		pds_evt.peer_data_update_succeeded.flash_changed = true;
		pds_evt_send(&pds_evt);
	}

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_delete(uint16_t peer_id, enum pm_peer_data_id data_id)
{
	int err;

	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS || !peer_data_id_is_valid(data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, data_id);

	err = BtPdsDelete(entry_id);
	if (err) {
		LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d. peer_id: %d, "
			"data_id: %d.",
			err, peer_id, data_id);
		return NRF_ERROR_INTERNAL;
	}

	/* Synchronous store: emit the completion event inline. Mirror the old
	 * bm_zms_evt_handler DELETE branch, which suppressed the per-entry event
	 * while a whole-peer delete is in progress (peer_id_is_deleted).
	 */
	if (!peer_id_is_deleted(peer_id)) {
		struct pm_evt pds_evt = {
			.evt_id = PM_EVT_PEER_DATA_UPDATE_SUCCEEDED,
			.peer_id = peer_id,
		};
		pds_evt.peer_data_update_succeeded.data_id = data_id;
		pds_evt.peer_data_update_succeeded.action = PM_PEER_DATA_OP_DELETE;
		pds_evt.peer_data_update_succeeded.token = entry_id;
		pds_evt.peer_data_update_succeeded.flash_changed = true;
		pds_evt_send(&pds_evt);
	}

	return NRF_SUCCESS;
}

uint16_t pds_peer_id_allocate(void)
{
	__ASSERT_NO_MSG(module_initialized);
	return peer_id_allocate(PM_PEER_ID_INVALID);
}

uint32_t pds_peer_id_free(uint16_t peer_id)
{
	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS) {
		return NRF_ERROR_INVALID_PARAM;
	}

	if (!peer_id_delete(peer_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	/* Only start processing on the first delete request.
	 * `peer_data_delete_process` will iteratively take care of processing all the peers marked
	 * for deletion.
	 */
	if (atomic_inc(&delete_counter) == 0) {
		peer_data_delete_process();
	}

	return NRF_SUCCESS;
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
