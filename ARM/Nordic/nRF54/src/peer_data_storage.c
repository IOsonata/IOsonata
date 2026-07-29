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

/* Store trace, on the same output as the PDS trace in bt_sec_bm.cpp rather
 * than the peer_manager LOG module, so what the store did and what came back
 * out of it read as one sequence. Independent of NDEBUG for the same reason
 * that one is: a release build is where peer data has to survive a reset.
 */
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

// IOsonata persistent data store replaces the bm_zms filesystem. The region
// is set where the memory is brought up, so no devicetree partition macros
// are needed here.
#include "bluetooth/bt_pds.h"

// Mounts the store on an Nvm. C entry point defined in bt_sec_bm.cpp, because
// Nvm is a C++ class and this is C. Where the store lives comes from the
// linker script, not from a constant here or there.
extern int BtPdsBmInit(void);

/* A store or a delete arrives from inside the peer manager's BLE event
 * dispatch. The memory is arbitrated with an MPSL timeslot, and the wait for
 * that timeslot cannot make progress from the context the dispatch runs in, so
 * writing there ends the slot abnormally. The work is queued and runs from the
 * application event handler instead.
 *
 * This is also what the API already says happens: a store is asynchronous and
 * the caller expects PM_EVT_PEER_DATA_UPDATE_SUCCEEDED or _FAILED to follow.
 * Sending the success event inline, as this used to, reported a write that had
 * not happened yet.
 */
#define PDS_QUE_DEPTH			4

struct pds_pending {
	bool				busy;
	bool				is_delete;
	/* Delete every entry belonging to peer_id, then the peer itself, one
	 * entry per handler invocation. Reports once for the peer rather than
	 * once per entry, so pds_pending_report is not used for it.
	 */
	bool				del_peer;
	uint16_t			peer_id;
	enum pm_peer_data_id	data_id;
	uint32_t		entry_id;
	uint32_t		length;
	uint8_t			data[BT_PDS_RECORD_DATA_MAX];
};

static struct pds_pending pds_que[PDS_QUE_DEPTH];

/* One whole-peer delete is in flight at a time. It walks every peer marked
 * deleted before it releases its slot, so pm_peers_delete over more peers
 * than the queue is deep cannot run out of slots, and a request arriving
 * while one runs needs nothing more than the peer being marked.
 */
static bool del_peer_running;

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

		/* Some peer data has been found. BtPdsWrite accepts a zero length
		 * value, so BtPdsRead answering 0 means present and empty. Absent
		 * is -ENOENT, which the test above has already let through.
		 */
		if (ret >= 0) {
			*next_entry_id = entry_id;
			return NRF_SUCCESS;
		}
	}

	/* Every data read for the peer has returned `-ENOENT`. */
	return NRF_ERROR_NOT_FOUND;
}

/* One step of the whole-peer delete.
 *
 * The stock bm_zms module is a state machine that deletes a single entry per
 * invocation and is re-entered by the store's completion callback. The
 * IOsonata store finishes inside the call, so there is no such callback and an
 * earlier version of this file collapsed the machine into a straight loop.
 * That put a read and delete of every record of every deleted peer inside the
 * Peer Manager BLE dispatch, where the rest of this module deliberately does
 * not work, and it let a whole-peer delete run ahead of a store already
 * sitting in the queue for that same peer: the delete scanned the memory, did
 * not see the queued record, freed the peer id, and the store then wrote under
 * an id that had been handed back.
 *
 * So the machine is back, re-entered from the application event queue instead
 * of a store callback. Being in the same queue as the stores is what puts it
 * behind them.
 *
 * Returns true when there is more to do and the caller should queue it again.
 */
static bool peer_delete_step(struct pds_pending *p)
{
	uint32_t entry_id;

	if (p->peer_id == PM_PEER_ID_INVALID) {
		p->peer_id = peer_id_get_next_deleted(PM_PEER_ID_INVALID);

		if (p->peer_id == PM_PEER_ID_INVALID) {
			return false;		/* nothing marked, nothing to do */
		}
	}

	/* NOT_FOUND is the end of this peer; anything else is a read that
	 * failed, and taking that for the end freed the peer id with its
	 * records still there.
	 */
	uint32_t look = find_next_data_entry_in_peer(p->peer_id, &entry_id);

	if (look == NRF_ERROR_NOT_FOUND) {
		struct pm_evt done_evt = {
			.evt_id = PM_EVT_PEER_DELETE_SUCCEEDED,
			.peer_id = p->peer_id,
		};

		peer_id_free(p->peer_id);
		PDS_LOAD_PRINTF("PDS: peer %u deleted\r\n", (unsigned)p->peer_id);
		pds_evt_send(&done_evt);

		/* On to whatever else is marked. Asking from the start is right
		 * because the id just freed is no longer marked.
		 */
		p->peer_id = peer_id_get_next_deleted(PM_PEER_ID_INVALID);

		return p->peer_id != PM_PEER_ID_INVALID;
	}

	if (look != NRF_SUCCESS) {
		/* The peer keeps its id and stays marked deleted, so its data is
		 * still reachable and a later delete request walks it again.
		 * Freeing the id here handed it back for reuse with records still
		 * stored under it. Stopping rather than moving on, because the
		 * peer stays marked and would be picked up again immediately.
		 */
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
		/* Out of room part way through. The peer keeps its id and stays
		 * marked, so a later delete request walks it again once something
		 * has freed space. Nothing here retries: a store with no room and
		 * no other traffic would have this requeueing against itself.
		 */
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

	return true;			/* one entry gone, come back for the next */
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

	/* Bring up the memory and mount the IOsonata store on it. Where the store
	 * lives comes from the linker script, so nothing here states an address.
	 * Mounting is synchronous: on return the store has been scanned. Storing
	 * and deleting are not, and report through pds_evt_send when they finish.
	 */
	err = BtPdsBmInit();
	if (err) {
		LOG_ERR("Could not initialize NVM storage. BtPdsBmInit() returned %d.", err);
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

/* Report one queued operation, then release its slot. */
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

/* Runs from the application event handler, out of the BLE event dispatch. */
static void pds_work_handler(uint32_t evt, void *ctx)
{
	struct pds_pending *p = (struct pds_pending *)ctx;

	(void)evt;

	if (p == NULL || !p->busy) {
		return;
	}

	if (p->del_peer) {
		if (peer_delete_step(p) && AppEvtHandlerQue(0, p, pds_work_handler)) {
			return;
		}

		/* Finished, stopped on an error, or the queue would not take it
		 * back. Anything left is still marked deleted and a later
		 * pds_peer_id_free walks it.
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
			return;
		}

		/* A whole peer delete reports once for the peer, not per entry. */
		if (peer_id_is_deleted(p->peer_id)) {
			p->busy = false;
			return;
		}

		pds_pending_report(p, true);

		return;
	}

	ssize_t ret = BtPdsWrite(p->entry_id, p->data, p->length);

	if (ret < 0) {
		LOG_ERR("Could not write data to NVM. BtPdsWrite() returned %d. "
			"peer_id: %d", (int)ret, p->peer_id);
		pds_pending_report(p, false);

		return;
	}

	pds_pending_report(p, true);
}

/* Claim a free slot, or NULL when every one is still in flight. */
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

uint32_t pds_peer_data_store(uint16_t peer_id, const struct pm_peer_data_const *peer_data,
			     uint32_t *store_token)
{
	__ASSERT_NO_MSG(module_initialized);
	__ASSERT_NO_MSG(peer_data != NULL);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS || !peer_data_id_is_valid(peer_data->data_id)) {
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

	uint32_t entry_id = peer_id_peer_data_id_to_entry_id(peer_id, peer_data->data_id);

	/* The caller's buffer does not outlive this call. */
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

uint32_t pds_peer_data_delete(uint16_t peer_id, enum pm_peer_data_id data_id)
{
	__ASSERT_NO_MSG(module_initialized);

	if (peer_id >= PM_PEER_ID_N_AVAILABLE_IDS || !peer_data_id_is_valid(data_id)) {
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

	/* Marked, and that is what the delete works from. One walk is in flight
	 * at a time and it takes every marked peer in turn, so a request that
	 * arrives while one runs needs nothing further.
	 */
	if (del_peer_running) {
		return NRF_SUCCESS;
	}

	struct pds_pending *p = pds_pending_claim();

	if (p == NULL) {
		return NRF_ERROR_BUSY;
	}

	p->del_peer = true;
	p->is_delete = true;
	p->peer_id = PM_PEER_ID_INVALID;	/* start from the first marked */
	p->data_id = PM_PEER_DATA_ID_BONDING;
	p->entry_id = 0;
	p->length = 0;

	if (!AppEvtHandlerQue(0, p, pds_work_handler)) {
		p->del_peer = false;
		p->busy = false;

		return NRF_ERROR_BUSY;
	}

	del_peer_running = true;

	/* PM_EVT_PEER_DELETE_SUCCEEDED follows from the handler. peer_manager
	 * expects that: pm_peers_delete returns after requesting every peer and
	 * raises PM_EVT_PEERS_DELETE_SUCCEEDED later, from the per peer event,
	 * once nothing allocated or marked is left.
	 */
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
