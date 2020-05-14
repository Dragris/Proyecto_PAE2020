/*
 * b_queue.c
 *
 *  Created on: 22 mar. 2020
 *      Author: droma
 *
 *  Simple thread safe circular FIFO queue
 */

#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "b_queue.h"

// Avoid mallocs. Since we known we only need one queue, we will declared it static
// instead of the usual init returning a queue object

/**
 * Initialize the previous queue
 */
void init_queue(Queue_t *q) {
    //Initialization of the queue pop and push indexes
    q->idx_pop = 0;
    q->idx_push = 0;
    //Initialization of the variable counting the elements pending
    // to be read from the queue
    q->data_pending = 0;
    //Initialization of mutex object for the protect code of the queue
    pthread_mutex_init(&q->mutex, NULL);
}

/**
 * Push a new data byte to the queue
 *
 * @param[in] data Byte to be added to the queue
 * @return Return queue status: full or success
 */
QUEUE_RET queue_push(uint8_t data, Queue_t *q) {
    // Check if the queue is full
    if (q->data_pending >= sizeof(q->data)) {
        return QUEUE_FULL;
    }
    // Start the mutex protected code
    pthread_mutex_lock(&q->mutex);
    // Add the data to the latest free position
    q->data[q->idx_push] = data;
    // Increment the counter of the read pending data
    q->data_pending++;
    // Compute the next write position with modulo the FIFO length
    q->idx_push = (q->idx_push + 1) % sizeof(q->data);
    // Release the mutex lock
    pthread_mutex_unlock(&q->mutex);

    return QUEUE_OK;
}

/**
 * Pop a byte from the queue
 *
 * @param[out] data Byte removed from the queue
 * @return Return queue status: empty or success
 */
QUEUE_RET queue_pop(uint8_t *data, Queue_t *q) {
    // Check if the queue is empty
    if (q->data_pending == 0) {
        return QUEUE_EMPTY;
    }
    // Start the mutex protected code
    pthread_mutex_lock(&q->mutex);
    // Copy the read data to the pointer
    *data = q->data[q->idx_pop];
    // Decrement the counter of the read pending data
    q->data_pending--;
    // Compute the next read position with modulo the FIFO length
    q->idx_pop = (q->idx_pop + 1) % sizeof(q->data);
    // Release the mutex lock
    pthread_mutex_unlock(&q->mutex);

    return QUEUE_OK;
}

/**
 * Returns true in case queue is empty
 */
bool queue_is_empty(Queue_t *q) {
    if (q->data_pending == 0) {
        return true;
    } else {
        return false;
    }
}
/**
 * Returns true in case queue is full
 */
bool queue_is_full(Queue_t *q) {
    if (q->data_pending >= sizeof(q->data)) {
        return true;
    } else {
        return false;
    }
}
