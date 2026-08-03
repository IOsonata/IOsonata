/*
 * Copyright (c) 2015-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrf_error.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <bm/bluetooth/peer_manager/peer_manager_types.h>

#include "app_evt_handler.h"

#define PDS_LOAD_TRACE

#ifdef PDS_LOAD_TRACE
#include "syslog.h"
#define PDS_LOAD_PRINTF(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define PDS_LOAD_PRINTF(...)
#endif
#include <modules/peer_manager_internal.h>
#include <modules/peer_id.h>
#include <modules/peer_data_storage.h>

#include "bluetooth/bt_pds.h"

extern int BtPdsBmInit(void);

#define PDS_QUE_DEPTH			4

struct pds_pending {
	bool				busy;
	bool				is_delete;
	bool				del_peer;
	uint16_t			peer_id;
	enum pm_peer_data_id	data_id;
	uint32_t		entry_id;
	uint32_t		length;
	uint8_t			data[BT_PDS_RECORD_DATA_MAX];
};

static struct pds_pending pds_que[PDS_QUE_DEPTH];
static bool del_peer_running;

LOG_MODULE_DECLARE(peer_manager, CONFIG_PEER_MANAGER_LOG_LEVEL);

#define PDS_EVENT_HANDLERS_CNT ARRAY_SIZE(evt_handlers)

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

#define ENTRY_ID_PEER_ID_OFFSET_BITS 16
#define ENTRY_ID_DATA_ID_MASK        ((1 << ENTRY_ID_PEER_ID_OFFSET_BITS) - 1)

static uint32_t peer_id_peer_data_id_to_entry_id(uint16_t peer_id,
													 enum pm_peer_data_id data_id)
{
	return (peer_id << ENTRY_ID_PEER_ID_OFFSET_BITS) |
		(data_id & ENTRY_ID_DATA_ID_MASK);
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

static uint32_t find_next_data_entry_in_peer(uint16_t peer_id,
											 uint32_t *next_entry_id)
{
	ssize_t ret;
	uint8_t temp_buf[PM_PEER_DATA_MAX_SIZE] = { 0 };

	for (enum pm_peer_data_id i = 0; i < PM_PEER_DATA_ID_LAST; i++) {
		uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, i);

		ret = BtPdsRead(entry_id, temp_buf, sizeof(temp_buf));
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Could not read entry %d from NVM. bm_zms_read() returned %d. "
				"peer_id: %d, data_id: %d", entry_id, ret, peer_id, i);
			return NRF_ERROR_INTERNAL;
		}

		if (ret >= 0) {
			*next_entry_id = entry_id;
			return NRF_SUCCESS;
		}
	}

	return NRF_ERROR_NOT_FOUND;
}

static bool peer_delete_step(struct pds_pending *p)
{
	uint32_t entry_id;

	if (p->peer_id == PM_PEER_ID_INVALID) {
		p->peer_id = peer_id_get_next_deleted(PM_PEER_ID_INVALID);

		if (p->peer_id == PM_PEER_ID_INVALID) {
			return false;
		}
	}

	uint32_t look = find_next_data_entry_in_peer(p->peer_id, &entry_id);

	if (look == NRF_ERROR_NOT_FOUND) {
		struct pm_evt done_evt = {
			.evt_id = PM_EVT_PEER_DELETE_SUCCEEDED,
			.peer_id = p->peer_id,
		};

		peer_id_free(p->peer_id);
		PDS_LOAD_PRINTF("PDS: peer %u deleted\r\n", (unsigned)p->peer_id);
		pds_evt_send(&done_evt);

		p->peer_id = peer_id_get_next_deleted(PM_PEER_ID_INVALID);
		return p->peer_id != PM_PEER_ID_INVALID;
	}

	if (look != NRF_SUCCESS) {
		struct pm_evt fail_evt = {
			.evt_id = PM_EVT_PEER_DELETE_FAILED,
			.peer_id = p->peer_id,
		};

		fail_evt.peer_delete_failed.error = NRF_ERROR_INTERNAL;
		PDS_LOAD_PRINTF("PDS: peer %u delete failed, id kept\r\n",
						(unsigned)p->peer_id);
		pds_evt_send(&fail_evt);
		return false;
	}

	int err = BtPdsDelete(entry_id);

	if (err == -ENOMEM) {
		PDS_LOAD_PRINTF("PDS: peer %u delete out of room, still marked\r\n",
						(unsigned)p->peer_id);
		return false;
	}

	if (err < 0) {
		struct pm_evt fail_evt = {
			.evt_id = PM_EVT_PEER_DELETE_FAILED,
			.peer_id = p->peer_id,
		};

		LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d "
			"for peer_id: %d", err, p->peer_id);

		fail_evt.peer_delete_failed.error = NRF_ERROR_INTERNAL;
		PDS_LOAD_PRINTF("PDS: peer %u delete failed, id kept\r\n",
						(unsigned)p->peer_id);
		pds_evt_send(&fail_evt);
		return false;
	}

	return true;
}

static void peer_ids_load(void)
{
	uint16_t peer_id;
	uint16_t peer_id_iter;
	struct pm_peer_data_const peer_data = { 0 };
	uint8_t peer_data_buffer[PM_PEER_DATA_MAX_SIZE] = { 0 };

	peer_data.all_data = peer_data_buffer;
	pds_peer_data_iterate_prepare(&peer_id_iter);

	unsigned n = 0;

	while (pds_peer_data_iterate(PM_PEER_DATA_ID_BONDING, &peer_id, &peer_data,
		&peer_id_iter)) {
		if (peer_id_allocate(peer_id) == peer_id) {
			PDS_LOAD_PRINTF("PDS: peer %u restored\r\n", (unsigned)peer_id);
			n++;
		} else {
			PDS_LOAD_PRINTF("PDS: peer %u found but not allocatable\r\n",
							(unsigned)peer_id);
		}
	}

	PDS_LOAD_PRINTF("PDS: load done, %u peer%s restored\r\n", n,
					n == 1 ? "" : "s");
}

void pds_peer_data_iterate_prepare(uint16_t *peer_id_iter)
{
	*peer_id_iter = 0;
}

bool pds_peer_data_iterate(enum pm_peer_data_id data_id,
			   uint16_t *const peer_id,
			   struct pm_peer_data_const *const data,
			   uint16_t *peer_id_iter)
{
	ssize_t ret;
	uint8_t temp_buf[PM_PEER_DATA_MAX_SIZE] = { 0 };

	if (*peer_id_iter >= PM_PEER_ID_N_AVAILABLE_IDS) {
		return false;
	}

	do {
		uint32_t entry_id =
			peer_id_peer_data_id_to_entry_id(*peer_id_iter, data_id);

		ret = BtPdsRead(entry_id, temp_buf, sizeof(temp_buf));
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Could not read data from NVM. bm_zms_read() returned %d. "
				"peer_id: %d", ret, *peer_id_iter);
			return false;
		}

		(*peer_id_iter)++;
	} while ((ret == -ENOENT) &&
		 (*peer_id_iter < PM_PEER_ID_N_AVAILABLE_IDS));

	if ((ret == -ENOENT) &&
		(*peer_id_iter == PM_PEER_ID_N_AVAILABLE_IDS)) {
		return false;
	}

	*peer_id = (*peer_id_iter) - 1;
	memcpy((void *)data->all_data, temp_buf, ret);
	return true;
}

static void pds_work_handler(uint32_t evt, void *ctx);
static struct pds_pending *pds_pending_claim(void);
static void peer_delete_kick(void);

static void pds_idle_pump(void)
{
	peer_delete_kick();
}

uint32_t pds_init(void)
{
	int err;

	__ASSERT_NO_MSG(!module_initialized);

	err = BtPdsBmInit();
	if (err) {
		LOG_ERR("Could not initialize NVM storage. BtPdsBmInit() returned %d.", err);
		return NRF_ERROR_RESOURCES;
	}

	if (!AppEvtHandlerIdleRegister(pds_idle_pump)) {
		LOG_ERR("Could not register PDS application idle pump.");
		return NRF_ERROR_RESOURCES;
	}

	peer_id_init();
	peer_ids_load();

	module_initialized = true;
	return NRF_SUCCESS;
}

uint32_t pds_peer_data_read(uint16_t peer_id,
			    enum pm_peer_data_id data_id,
			    struct pm_peer_data *const data,
			    const uint32_t *const buf_len)
{
	ssize_t ret;

	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(data != NULL);
	__ASSERT_NO_MSG(buf_len != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_data_id_is_valid(data_id)) {
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
			"peer_id: %d", ret, peer_id);
		return NRF_ERROR_INTERNAL;
	}

	if (*buf_len < ret) {
		return NRF_ERROR_DATA_SIZE;
	}

	return NRF_SUCCESS;
}

static void pds_pending_report(struct pds_pending *p, bool ok)
{
	struct pm_evt evt = {
		.peer_id = p->peer_id,
	};

	if (ok) {
		evt.evt_id = PM_EVT_PEER_DATA_UPDATE_SUCCEEDED;
		evt.peer_data_update_succeeded.data_id = p->data_id;
		evt.peer_data_update_succeeded.action =
			p->is_delete ? PM_PEER_DATA_OP_DELETE : PM_PEER_DATA_OP_UPDATE;
		evt.peer_data_update_succeeded.token = p->entry_id;
		evt.peer_data_update_succeeded.flash_changed = true;
	} else {
		evt.evt_id = PM_EVT_PEER_DATA_UPDATE_FAILED;
		evt.peer_data_update_failed.data_id = p->data_id;
		evt.peer_data_update_failed.action =
			p->is_delete ? PM_PEER_DATA_OP_DELETE : PM_PEER_DATA_OP_UPDATE;
		evt.peer_data_update_failed.token = p->entry_id;
		evt.peer_data_update_failed.error = NRF_ERROR_INTERNAL;
	}

	p->busy = false;
	pds_evt_send(&evt);
}

static void peer_delete_kick(void)
{
	if (del_peer_running) {
		return;
	}
	if (peer_id_get_next_deleted(PM_PEER_ID_INVALID) == PM_PEER_ID_INVALID) {
		return;
	}

	struct pds_pending *p = pds_pending_claim();

	if (p == NULL) {
		return;
	}

	p->del_peer = true;
	p->is_delete = true;
	p->peer_id = PM_PEER_ID_INVALID;
	p->data_id = PM_PEER_DATA_ID_BONDING;
	p->entry_id = 0;
	p->length = 0;

	if (!AppEvtHandlerQue(0, p, pds_work_handler)) {
		p->del_peer = false;
		p->busy = false;
		return;
	}

	del_peer_running = true;
}

static void pds_work_handler(uint32_t evt, void *ctx)
{
	struct pds_pending *p = (struct pds_pending *)ctx;

	(void)evt;

	if (p == NULL || !p->busy) {
		return;
	}

	if (p->del_peer) {
		if (peer_delete_step(p) &&
			AppEvtHandlerQue(0, p, pds_work_handler)) {
			return;
		}

		/* The peer remains marked when the queue cannot take the next step.
		 * AppEvtHandlerExec invokes pds_idle_pump after releasing queue slots,
		 * which starts a new walk without waiting for unrelated storage work.
		 */
		del_peer_running = false;
		p->busy = false;
		return;
	}

	if (p->is_delete) {
		int err = BtPdsDelete(p->entry_id);

		if (err < 0) {
			LOG_ERR("Could not delete peer data. BtPdsDelete() returned %d. "
				"peer_id: %d", err, p->peer_id);
			pds_pending_report(p, false);
		} else if (peer_id_is_deleted(p->peer_id)) {
			p->busy = false;
		} else {
			pds_pending_report(p, true);
		}
	} else {
		ssize_t ret = BtPdsWrite(p->entry_id, p->data, p->length);

		if (ret < 0) {
			LOG_ERR("Could not write data to NVM. BtPdsWrite() returned %d. "
				"peer_id: %d", (int)ret, p->peer_id);
			pds_pending_report(p, false);
		} else {
			pds_pending_report(p, true);
		}
	}

	peer_delete_kick();
}

static struct pds_pending *pds_pending_claim(void)
{
	for (uint32_t i = 0; i < PDS_QUE_DEPTH; i++) {
		if (!pds_que[i].busy) {
			pds_que[i].busy = true;
			pds_que[i].del_peer = false;
			return &pds_que[i];
		}
	}

	return NULL;
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
	if (peer_data->length > sizeof(pds_que[0].data)) {
		LOG_ERR("Peer data %u exceeds BT_PDS_RECORD_DATA_MAX %u. peer_id: %d",
			(unsigned)peer_data->length,
			(unsigned)sizeof(pds_que[0].data), peer_id);
		return NRF_ERROR_DATA_SIZE;
	}

	struct pds_pending *p = pds_pending_claim();

	if (p == NULL) {
		return NRF_ERROR_BUSY;
	}

	uint32_t entry_id =
		peer_id_peer_data_id_to_entry_id(peer_id, peer_data->data_id);

	memcpy(p->data, peer_data->all_data, peer_data->length);
	p->length = peer_data->length;
	p->is_delete = false;
	p->peer_id = peer_id;
	p->data_id = peer_data->data_id;
	p->entry_id = entry_id;

	if (!AppEvtHandlerQue(0, p, pds_work_handler)) {
		p->busy = false;
		return NRF_ERROR_BUSY;
	}

	if (store_token != NULL) {
		*store_token = entry_id;
	}

	return NRF_SUCCESS;
}

uint32_t pds_peer_data_delete(uint16_t peer_id,
			     enum pm_peer_data_id data_id)
{
	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS ||
		!peer_data_id_is_valid(data_id)) {
		return NRF_ERROR_INVALID_PARAM;
	}

	struct pds_pending *p = pds_pending_claim();

	if (p == NULL) {
		return NRF_ERROR_BUSY;
	}

	p->is_delete = true;
	p->peer_id = peer_id;
	p->data_id = data_id;
	p->entry_id = peer_id_peer_data_id_to_entry_id(peer_id, data_id);
	p->length = 0;

	if (!AppEvtHandlerQue(0, p, pds_work_handler)) {
		p->busy = false;
		return NRF_ERROR_BUSY;
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

	peer_delete_kick();
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
