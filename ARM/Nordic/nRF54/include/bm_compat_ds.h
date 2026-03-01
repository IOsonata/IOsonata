/**-------------------------------------------------------------------------
@file	bm_compat_ds.h

@brief	Zephyr kernel data structure replacements for sdk-nrf-bm

Lightweight bare-metal implementations of:
  - k_mem_slab (fixed-size block allocator)
  - k_heap     (variable-size allocator over static buffer)
  - ring_buf   (byte-level circular buffer)

Note: sys_slist / sys_snode are defined in bm_compat.h (slist section).

Designed for Cortex-M bare metal.  No RTOS dependencies.

@author	IOsonata
@date	2025

@license MIT
----------------------------------------------------------------------------*/

#ifndef BM_COMPAT_DS_H__
#define BM_COMPAT_DS_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ======================================================================
 * Section: Memory slab  —  replaces <zephyr/kernel.h> k_mem_slab
 *
 * Fixed-size block allocator.  Free list threaded through the blocks
 * themselves (each free block stores a pointer to the next free block).
 * ====================================================================== */

/** @brief Timeout value: non-blocking. */
#ifndef K_NO_WAIT
#define K_NO_WAIT  0
#endif

struct k_mem_slab {
	void      *free_list;    /**< Head of the free-block chain.     */
	uint8_t   *buffer;       /**< Backing buffer.                   */
	size_t     block_size;   /**< Size of each block (aligned).     */
	uint32_t   num_blocks;   /**< Total number of blocks.           */
	uint32_t   num_used;     /**< Currently allocated blocks.       */
};

/** Round block size up to pointer alignment. */
#define _SLAB_BLOCK_SIZE(bsize) \
	(((bsize) + sizeof(void *) - 1) & ~(sizeof(void *) - 1))

/**
 * @brief Internal: define a memory slab with given storage class prefix.
 *
 * @param _sc          Storage class tokens (e.g. "static" or empty).
 * @param name         Symbol name.
 * @param bsize        Block size in bytes (rounded up to pointer alignment).
 * @param nblocks      Number of blocks.
 * @param balign       Minimum alignment (unused; pointer-aligned).
 */
#define _K_MEM_SLAB_DEFINE(_sc, name, bsize, nblocks, balign)        \
	_sc uint8_t __aligned(sizeof(void *))                        \
		_slab_buf_##name[(nblocks) * _SLAB_BLOCK_SIZE(bsize)]; \
	_sc struct k_mem_slab name = {                               \
		.free_list  = NULL,                                  \
		.buffer     = _slab_buf_##name,                      \
		.block_size = _SLAB_BLOCK_SIZE(bsize),               \
		.num_blocks = (nblocks),                             \
		.num_used   = 0,                                     \
	}

/** Static slab definition. */
#define K_MEM_SLAB_DEFINE_STATIC(name, bsize, nblocks, balign) \
	_K_MEM_SLAB_DEFINE(static, name, bsize, nblocks, balign)

/** Non-static slab definition — caller provides storage class. */
#define K_MEM_SLAB_DEFINE(name, bsize, nblocks, balign) \
	_K_MEM_SLAB_DEFINE(/*non-static*/, name, bsize, nblocks, balign)

/**
 * @brief Initialize a memory slab at runtime (builds free list).
 *
 * Called automatically on first alloc if free_list is NULL.
 */
static inline void k_mem_slab_init_lazy(struct k_mem_slab *slab)
{
	if (slab->free_list != NULL || slab->num_used > 0) {
		return;  /* Already initialized or in use. */
	}
	uint8_t *p = slab->buffer;
	void *prev = NULL;

	for (uint32_t i = 0; i < slab->num_blocks; i++) {
		*(void **)p = prev;
		prev = p;
		p += slab->block_size;
	}
	slab->free_list = prev;
}

/**
 * @brief Allocate a block from the slab.
 *
 * @param slab    Pointer to the memory slab.
 * @param mem     Output: pointer to allocated block.
 * @param timeout Ignored (always non-blocking on bare metal).
 *
 * @retval 0       Success.
 * @retval -ENOMEM No free blocks.
 */
static inline int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem,
                                   int timeout)
{
	(void)timeout;
	k_mem_slab_init_lazy(slab);

	if (slab->free_list == NULL) {
		return -ENOMEM;
	}
	void *block = slab->free_list;
	slab->free_list = *(void **)block;
	slab->num_used++;
	*mem = block;
	return 0;
}

/**
 * @brief Free a block back to the slab.
 */
static inline void k_mem_slab_free(struct k_mem_slab *slab, void *mem)
{
	*(void **)mem = slab->free_list;
	slab->free_list = mem;
	slab->num_used--;
}


/* ======================================================================
 * Section: Heap  —  replaces <zephyr/kernel.h> k_heap / K_HEAP_DEFINE
 *
 * Simple first-fit allocator over a static buffer.  Each allocated
 * block has a 4-byte header storing its size.  Free blocks are
 * chained through an embedded next pointer + size.  Adjacent free
 * blocks are coalesced on free.
 *
 * Designed for small allocations (BLE GATT payloads <= 247 bytes).
 * ====================================================================== */

/** @brief Free block header, embedded in free space. */
struct k_heap_free_blk {
	struct k_heap_free_blk *next;
	size_t size;  /**< Usable size (excludes the alloc header). */
};

/** @brief Allocation header prepended to every returned block. */
struct k_heap_alloc_hdr {
	size_t size;  /**< Usable size of this allocation. */
};

#define K_HEAP_ALLOC_HDR_SIZE  sizeof(struct k_heap_alloc_hdr)

struct k_heap {
	uint8_t *buffer;
	size_t   size;
	struct k_heap_free_blk *free_list;
	bool     initialized;
};

/**
 * @brief Define a heap with backing buffer.
 *
 * Does NOT include storage class — caller provides it:
 *   static K_HEAP_DEFINE(name, size);
 */
#define K_HEAP_DEFINE(name, heap_size)                                           \
	uint8_t __aligned(sizeof(void *)) _heap_buf_##name[heap_size];          \
	struct k_heap name = {                                                  \
		.buffer      = _heap_buf_##name,                                \
		.size        = (heap_size),                                     \
		.free_list   = NULL,                                            \
		.initialized = false,                                           \
	}

static inline void k_heap_init_lazy(struct k_heap *h)
{
	if (h->initialized) {
		return;
	}
	/* One big free block spanning the entire buffer. */
	struct k_heap_free_blk *blk = (struct k_heap_free_blk *)h->buffer;

	blk->next = NULL;
	blk->size = h->size - K_HEAP_ALLOC_HDR_SIZE;
	h->free_list = blk;
	h->initialized = true;
}

/**
 * @brief Aligned allocation from heap.  First-fit.
 *
 * @param h       Heap.
 * @param align   Required alignment (must be power of 2).
 * @param bytes   Requested size.
 * @param timeout Ignored.
 *
 * @return Pointer to allocated memory, or NULL.
 */
static inline void *k_heap_aligned_alloc(struct k_heap *h, size_t align,
                                         size_t bytes, int timeout)
{
	(void)timeout;
	k_heap_init_lazy(h);

	if (align < sizeof(void *)) {
		align = sizeof(void *);
	}

	/* Minimum block: must hold free_blk header when freed. */
	size_t min_payload = sizeof(struct k_heap_free_blk) - K_HEAP_ALLOC_HDR_SIZE;
	if (bytes < min_payload) {
		bytes = min_payload;
	}

	struct k_heap_free_blk **prev = &h->free_list;
	struct k_heap_free_blk *cur = h->free_list;

	while (cur) {
		/* Compute aligned payload address after alloc header. */
		uintptr_t raw = (uintptr_t)cur + K_HEAP_ALLOC_HDR_SIZE;
		uintptr_t aligned = (raw + align - 1) & ~(align - 1);
		size_t pad = aligned - raw;

		if (cur->size >= bytes + pad) {
			/* Split off remainder if large enough. */
			size_t total_used = pad + bytes;
			size_t remain = cur->size - total_used;

			if (remain >= sizeof(struct k_heap_free_blk)) {
				/* Create a new free block after this allocation. */
				struct k_heap_free_blk *nblk =
					(struct k_heap_free_blk *)
					((uint8_t *)cur + K_HEAP_ALLOC_HDR_SIZE + total_used);
				nblk->next = cur->next;
				nblk->size = remain - K_HEAP_ALLOC_HDR_SIZE;
				*prev = nblk;
			} else {
				/* Use the whole block (include remainder in alloc). */
				bytes = cur->size - pad;
				*prev = cur->next;
			}

			/* Write alloc header just before the aligned payload. */
			struct k_heap_alloc_hdr *hdr =
				(struct k_heap_alloc_hdr *)(aligned - K_HEAP_ALLOC_HDR_SIZE);
			hdr->size = bytes + pad;

			return (void *)aligned;
		}

		prev = &cur->next;
		cur = cur->next;
	}

	return NULL;  /* Out of memory. */
}

/**
 * @brief Free a heap allocation.
 *
 * Inserts the block back into the free list in address order and
 * coalesces with adjacent free blocks.
 */
static inline void k_heap_free(struct k_heap *h, void *mem)
{
	if (mem == NULL) {
		return;
	}

	struct k_heap_alloc_hdr *hdr =
		(struct k_heap_alloc_hdr *)((uint8_t *)mem - K_HEAP_ALLOC_HDR_SIZE);

	/* Build a free block at the alloc header position. */
	struct k_heap_free_blk *freed = (struct k_heap_free_blk *)hdr;

	freed->size = hdr->size;

	/* Insert in address-sorted free list. */
	struct k_heap_free_blk **prev = &h->free_list;
	struct k_heap_free_blk *cur = h->free_list;

	while (cur && cur < freed) {
		prev = &cur->next;
		cur = cur->next;
	}

	freed->next = cur;
	*prev = freed;

	/* Coalesce with next block. */
	if (cur &&
	    ((uint8_t *)freed + K_HEAP_ALLOC_HDR_SIZE + freed->size == (uint8_t *)cur)) {
		freed->size += K_HEAP_ALLOC_HDR_SIZE + cur->size;
		freed->next = cur->next;
	}

	/* Coalesce with previous block. */
	if (prev != &h->free_list) {
		struct k_heap_free_blk *prev_blk =
			(struct k_heap_free_blk *)((uintptr_t)prev -
			offsetof(struct k_heap_free_blk, next));
		if ((uint8_t *)prev_blk + K_HEAP_ALLOC_HDR_SIZE + prev_blk->size ==
		    (uint8_t *)freed) {
			prev_blk->size += K_HEAP_ALLOC_HDR_SIZE + freed->size;
			prev_blk->next = freed->next;
		}
	}
}

static inline void *k_heap_alloc(struct k_heap *h, size_t bytes, int timeout)
{
	return k_heap_aligned_alloc(h, sizeof(void *), bytes, timeout);
}


/* ======================================================================
 * Section: Ring Buffer  —  replaces <zephyr/sys/ring_buffer.h>
 *
 * Byte-level circular buffer.  Used by BLE samples for FIFO queues.
 * ====================================================================== */

struct ring_buf {
	uint8_t  *buffer;
	uint32_t  size;
	uint32_t  head;   /**< Next write position.  */
	uint32_t  tail;   /**< Next read position.   */
};

#define RING_BUF_DECLARE(name, buf_size)                          \
	static uint8_t _ring_data_##name[buf_size];               \
	static struct ring_buf name = {                            \
		.buffer = _ring_data_##name,                       \
		.size   = (buf_size),                              \
		.head   = 0,                                       \
		.tail   = 0,                                       \
	}

static inline void ring_buf_reset(struct ring_buf *rb)
{
	rb->head = 0;
	rb->tail = 0;
}

static inline bool ring_buf_is_empty(const struct ring_buf *rb)
{
	return rb->head == rb->tail;
}

static inline uint32_t ring_buf_space_get(const struct ring_buf *rb)
{
	return rb->size - (rb->head - rb->tail);
}

static inline uint32_t ring_buf_put(struct ring_buf *rb,
                                    const uint8_t *data, uint32_t len)
{
	uint32_t space = ring_buf_space_get(rb);

	if (len > space) {
		len = space;
	}
	for (uint32_t i = 0; i < len; i++) {
		rb->buffer[rb->head % rb->size] = data[i];
		rb->head++;
	}
	return len;
}

static inline uint32_t ring_buf_get(struct ring_buf *rb,
                                    uint8_t *data, uint32_t len)
{
	uint32_t avail = rb->head - rb->tail;

	if (len > avail) {
		len = avail;
	}
	for (uint32_t i = 0; i < len; i++) {
		data[i] = rb->buffer[rb->tail % rb->size];
		rb->tail++;
	}
	return len;
}


#ifdef __cplusplus
}
#endif

#endif /* BM_COMPAT_DS_H__ */
