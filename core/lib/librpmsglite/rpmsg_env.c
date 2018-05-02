/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * Copyright (c) 2015 Xilinx, Inc.
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * Copyright (c) Microsoft
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 * FILE NAME
 *
 *       bm_env.c
 *
 *
 * DESCRIPTION
 *
 *       This file is Bare Metal Implementation of env layer for OpenAMP.
 *
 *
 **************************************************************************/

#include <rpmsglite/rpmsg_lite.h>
#include <rpmsglite/rpmsg_env.h>
#include <rpmsglite/rpmsg_platform.h>
#include <rpmsglite/virtqueue.h>
#include <rpmsglite/rpmsg_compiler.h>
#include <mm/core_memprot.h>
#include <kernel/tee_time.h>
#include <kernel/spinlock.h>
#include <rpmsglite/rpmsg_queue.h>

#include <stdlib.h>
#include <string.h>
#include <string_ext.h>

#define DEBUG_ENV_QUEUE 0

static int env_init_counter = 0;

/* Max supported ISR counts which equals to the max number of supported queues */
#define ISR_COUNT 4

/*!
 * Structure to keep track of registered ISR's.
 */
struct isr_info {
	void *data;
};
static struct isr_info isr_table[ISR_COUNT];

struct semaphore {
	unsigned int count;
	unsigned int lock;
};

struct concurrent_bounded_queue {
	size_t capacity;
	size_t count;
	size_t elem_size;
	size_t next_in;
	size_t next_out;
	void *buffer;
	struct semaphore can_produce_sem;
	struct semaphore can_consume_sem;
	unsigned int lock;
};

#if DEBUG_ENV_QUEUE
static void dump(uint8_t *buff, size_t len)
{
	while (len > 0) {
		DMSG("%x", *buff);
		buff++;
		len--;
	}
}
#endif

static void sem_init(struct semaphore *sem, int count)
{
	assert(sem);

	memset(sem, 0x0, sizeof(*sem));
	sem->count = count;
	sem->lock = SPINLOCK_UNLOCK;
}

static void sem_deinit(struct semaphore *sem)
{
	assert(sem);
	assert(sem->lock != SPINLOCK_UNLOCK);
}

static void sem_wait(struct semaphore *sem)
{
	uint32_t exceptions;
	assert(sem);

	for (;;) {
		exceptions = cpu_spin_lock_xsave(&sem->lock);
		if (sem->count != 0) {
			sem->count--;
			cpu_spin_unlock_xrestore(&sem->lock, exceptions);
			return;
		}
		cpu_spin_unlock_xrestore(&sem->lock, exceptions);
	}
}

static void sem_signal(struct semaphore *sem)
{
	uint32_t exceptions;
	assert(sem);

	exceptions = cpu_spin_lock_xsave(&sem->lock);
	sem->count++;
	cpu_spin_unlock_xrestore(&sem->lock, exceptions);
}

static bool queue_init(struct concurrent_bounded_queue *q, size_t capacity,
		       size_t elem_size)
{
	assert(q);

	memset(q, 0x00, sizeof(*q));

	q->capacity = capacity;
	q->count = 0;
	q->elem_size = elem_size;
	q->buffer = calloc(capacity, elem_size);
	if (!q->buffer) {
		return false;
	}

	q->next_in = 0;
	q->next_out = 0;
	q->lock = SPINLOCK_UNLOCK;
	sem_init(&q->can_produce_sem, capacity);
	sem_init(&q->can_consume_sem, 0);

	return true;
}

static void queue_deinit(struct concurrent_bounded_queue *q)
{
	assert(q);
	assert(q->buffer);
	free(q->buffer);
	sem_deinit(&q->can_produce_sem);
	sem_deinit(&q->can_consume_sem);
	memset(q, 0x00, sizeof(*q));
}

static size_t queue_get_size(struct concurrent_bounded_queue *q)
{
	assert(q);
	return q->count;
}

static void queue_enqueue(struct concurrent_bounded_queue *q, const void *src)
{
	void *dst;
	uint32_t exceptions;
	assert(q);
	assert(src);

	sem_wait(&q->can_produce_sem);

	exceptions = cpu_spin_lock_xsave(&q->lock);
	dst = (char *)q->buffer + (q->next_in * q->elem_size);
	memcpy(dst, src, q->elem_size);
	q->next_in = (q->next_in + 1) % q->capacity;
	q->count++;
	DMSG("+enqueue count:%u", q->count);
	cpu_spin_unlock_xrestore(&q->lock, exceptions);

	sem_signal(&q->can_consume_sem);
}

static void queue_dequeue(struct concurrent_bounded_queue *q, void *dst)
{
	void *src;
	uint32_t exceptions;
	assert(q);
	assert(dst);

	sem_wait(&q->can_consume_sem);

	exceptions = cpu_spin_lock_xsave(&q->lock);
	src = (char *)q->buffer + (q->next_out * q->elem_size);
	memcpy(dst, src, q->elem_size);
	q->next_out = (q->next_out + 1) % q->capacity;
	q->count--;
	DMSG("-dequeue count:%u", q->count);
	cpu_spin_unlock_xrestore(&q->lock, exceptions);

	sem_signal(&q->can_produce_sem);
}


/*!
 * env_init
 *
 * Initializes OS/BM environment.
 *
 */
int env_init(void)
{
	DMSG("RPMSG env init");
	// verify 'env_init_counter'
	RL_ASSERT(env_init_counter >= 0);
	if (env_init_counter < 0) {
		return -1;
	}
	env_init_counter++;
	// multiple call of 'env_init' - return ok
	if (1 < env_init_counter) {
		return 0;
	}
	// first call
	memset(isr_table, 0, sizeof(isr_table));
	return platform_init();
}

/*!
 * env_deinit
 *
 * Uninitializes OS/BM environment.
 *
 * @returns Execution status
 */
int env_deinit(void)
{
	DMSG("RPMSG env deinit");
	// verify 'env_init_counter'
	RL_ASSERT(env_init_counter > 0);
	if (env_init_counter <= 0) {
		return -1;
	}
	// counter on zero - call platform deinit
	env_init_counter--;
	// multiple call of 'env_deinit' - return ok
	if (0 < env_init_counter) {
		return 0;
	}
	// last call
	return platform_deinit();
}

/*!
 * env_allocate_memory - implementation
 *
 * @param size
 */
void *env_allocate_memory(unsigned int size)
{
	return malloc(size);
}

/*!
 * env_free_memory - implementation
 *
 * @param ptr
 */
void env_free_memory(void *ptr)
{
	if (ptr != NULL) {
		free(ptr);
	}
}

/*!
 *
 * env_memset - implementation
 *
 * @param ptr
 * @param value
 * @param size
 */
void env_memset(void *ptr, int value, unsigned long size)
{
	memset(ptr, value, size);
}

/*!
 *
 * env_memcpy - implementation
 *
 * @param dst
 * @param src
 * @param len
 */
void env_memcpy(void *dst, void const *src, unsigned long len)
{
	memcpy(dst, src, len);
}

/*!
 *
 * env_strcmp - implementation
 *
 * @param dst
 * @param src
 */

int env_strcmp(const char *dst, const char *src)
{
	return strcmp(dst, src);
}

/*!
 *
 * env_strncpy - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
void env_strncpy(char *dest, const char *src, unsigned long len)
{
	strlcpy(dest, src, len);
}

/*!
 *
 * env_strncmp - implementation
 *
 * @param dest
 * @param src
 * @param len
 */
int env_strncmp(char *dest, const char *src, unsigned long len)
{
	return strncmp(dest, src, len);
}

/*!
 *
 * env_mb - implementation
 *
 */
void env_mb(void)
{
	MEM_BARRIER();
}

/*!
 * env_rmb - implementation
 */
void env_rmb(void)
{
	MEM_BARRIER();
}

/*!
 * env_wmb - implementation
 */
void env_wmb(void)
{
	MEM_BARRIER();
}

/*!
 * env_map_vatopa - implementation
 *
 * @param address
 */
unsigned long env_map_vatopa(void *address)
{
	assert(address);
	return (unsigned long)virt_to_phys(address);
}

/*!
 * env_map_patova - implementation
 *
 * @param address
 */
void *env_map_patova(unsigned long address)
{
	assert(address);
	return (void *)phys_to_virt((paddr_t)address, MEM_AREA_IO_SEC);
}

/*!
 * env_create_mutex
 *
 * Creates a mutex with the given initial count.
 *
 */
int env_create_mutex(void **lock, int count)
{
	struct semaphore *m;

	if (!lock || (count > 1) || (count < 0)) {
		return -1;
	}

	m = malloc(sizeof(struct semaphore));
	if (!m) {
		return -1;
	}

	sem_init(m, 1);

	if (count == 0) {
		sem_wait(m);
	}

	DMSG("created %p count %d", (void *)m, count);

	*lock = m;
	return 0;
}

/*!
 * env_delete_mutex
 *
 * Deletes the given lock
 *
 */
void env_delete_mutex(void *lock)
{
	assert(lock);

	DMSG("destroy %p", lock);

	sem_deinit((struct semaphore *)lock);
	free(lock);
}

/*!
 * env_lock_mutex
 *
 * Tries to acquire the lock, if lock is not available then call to
 * this function will suspend.
 */
void env_lock_mutex(void *lock)
{
	assert(lock);
	DMSG("+lock %p", lock);
	sem_wait((struct semaphore *)lock);
}

/*!
 * env_unlock_mutex
 *
 * Releases the given lock.
 */
void env_unlock_mutex(void *lock)
{
	assert(lock);
	DMSG("-unlock %p", lock);
	sem_signal((struct semaphore *)lock);
}

/*!
 * env_sleep_msec
 *
 * Suspends the calling thread for given time , in msecs.
 */
void env_sleep_msec(int num_msec)
{
	/* implementing sleep is not possible without an RPC call to REE */
}

/*!
 * env_register_isr
 *
 * Registers interrupt handler data for the given interrupt vector.
 *
 * @param vector_id - virtual interrupt vector number
 * @param data      - interrupt handler data (virtqueue)
 */
void env_register_isr(int vector_id, void *data)
{
	RL_ASSERT(vector_id < ISR_COUNT);
	if (vector_id < ISR_COUNT) {
		DMSG("register channel%d interrupt", vector_id);
		isr_table[vector_id].data = data;
	}
}

/*!
 * env_unregister_isr
 *
 * Unregisters interrupt handler data for the given interrupt vector.
 *
 * @param vector_id - virtual interrupt vector number
 */
void env_unregister_isr(int vector_id)
{
	RL_ASSERT(vector_id < ISR_COUNT);
	if (vector_id < ISR_COUNT) {
		DMSG("unregister channel%d interrupt", vector_id);
		isr_table[vector_id].data = NULL;
	}
}

/*!
 * env_enable_interrupt
 *
 * Enables the given interrupt
 *
 * @param vector_id   - virtual interrupt vector number
 */

void env_enable_interrupt(unsigned int vector_id)
{
	DMSG("enable channel%d interrupt", vector_id);
	platform_interrupt_enable(vector_id);
}

/*!
 * env_disable_interrupt
 *
 * Disables the given interrupt
 *
 * @param vector_id   - virtual interrupt vector number
 */

void env_disable_interrupt(unsigned int vector_id)
{
	DMSG("disable channel%d interrupt", vector_id);
	platform_interrupt_disable(vector_id);
}


/*!
 * env_create_queue
 *
 * Creates a message queue.
 *
 * @param queue -  pointer to created queue
 * @param length -  maximum number of elements in the queue
 * @param item_size - queue element size in bytes
 *
 * @return - status of function execution
 */
int env_create_queue(void **queue, int length, int element_size)
{
	struct concurrent_bounded_queue *q;

	assert(queue);
	assert(length > 0);
	assert(element_size > 0);

	DMSG("create queue element size %d len %d", element_size, length);

	q = malloc(sizeof(*q));
	if (!q) {
		return -1;
	}

	if (!queue_init(q, length, (size_t)element_size)) {
		return -1;
	}

	*queue = q;

	return 0;
}

/*!
 * env_delete_queue
 *
 * Deletes the message queue.
 *
 * @param queue - queue to delete
 */

void env_delete_queue(void *queue)
{
	assert(queue);
	queue_deinit(queue);
}

struct rpmsg_queue_rx_cb_data {
	unsigned long src;
	void *data;
	short int len;
};

/*!
 * env_put_queue
 *
 * Put an element in a queue.
 *
 * @param queue - queue to put element in
 * @param msg - pointer to the message to be put into the queue
 * @param timeout_ms - timeout in ms
 *
 * @return - status of function execution
 */

int env_put_queue(void *queue, void *msg, int timeout_ms)
{
	struct concurrent_bounded_queue *q;
	uint32_t exceptions;
	assert(queue);
	assert(msg);

	/* only no-timeout (non-blocking) or infinite timeout (blocking) are
	 * supported */
	assert((timeout_ms == 0) || (timeout_ms == (int)RL_BLOCK));

	q = (struct concurrent_bounded_queue *)queue;
	exceptions = cpu_spin_lock_xsave(&q->lock);

#if DEBUG_ENV_QUEUE
	IMSG("+put");
	dump(m->data, m->len);
#endif

	if ((q->count == q->capacity) && (timeout_ms == 0)) {
		EMSG("queue is full");
		cpu_spin_unlock_xrestore(&q->lock, exceptions);
		return 0;
	} else {
		cpu_spin_unlock_xrestore(&q->lock, exceptions);
		/* potential race condition here!!
		 * the whole env_put_queue should be atomic, by releasing the lock here
		 * 2 threads may end up enqueing, one of them will get block and the other
		 * will not if the queue wasn't full during the queue-full check.
		 */
		queue_enqueue((struct concurrent_bounded_queue *)queue, msg);
		return 1;
	}
}

/*!
 * env_get_queue
 *
 * Get an element out of a queue.
 *
 * @param queue - queue to get element from
 * @param msg - pointer to a memory to save the message
 * @param timeout_ms - timeout in ms
 *
 * @return - status of function execution
 */

int env_get_queue(void *queue, void *msg, int timeout_ms)
{
	struct concurrent_bounded_queue *q;
	uint32_t exceptions;
	assert(queue);
	assert(msg);

	q = (struct concurrent_bounded_queue *)queue;
	assert(timeout_ms == (int)RL_BLOCK);
	queue_dequeue(q, msg);

	exceptions = cpu_spin_lock_xsave(&q->lock);

#if DEBUG_ENV_QUEUE
	IMSG("+get");
	dump((uint8_t *)m->data, m->len);
#endif
	cpu_spin_unlock_xrestore(&q->lock, exceptions);

	return 1;
}

/*!
 * env_get_current_queue_size
 *
 * Get current queue size.
 *
 * @param queue - queue pointer
 *
 * @return - Number of queued items in the queue
 */

int env_get_current_queue_size(void *queue)
{
	assert(queue);
	return queue_get_size((struct concurrent_bounded_queue *)queue);
}


/*========================================================= */
/* Util data / functions for BM */

void env_isr(int vector)
{
	struct isr_info *info;
	RL_ASSERT(vector < ISR_COUNT);
	if (vector < ISR_COUNT) {
		info = &isr_table[vector];
		virtqueue_notification((struct virtqueue *)info->data);
	}
}
