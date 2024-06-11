#include "main.h"
#include "Queue.h"

void initQueue(StateQueue_t* q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
}

int isQueueFull(StateQueue_t* q) {
    return q->size == MAX_QUEUE_SIZE;
}

int isQueueEmpty(StateQueue_t* q) {
    return q->size == 0;
}

void enqueue(StateQueue_t* q, State_t state) {
    if (!isQueueFull(q)) {
        q->queue[q->rear] = state;
        q->rear = (q->rear + 1) % MAX_QUEUE_SIZE;
        q->size++;
    }
}

State_t dequeue(StateQueue_t* q) {
    State_t state = STATE_IDLE;
    if (!isQueueEmpty(q)) {
        state = q->queue[q->front];
        q->front = (q->front + 1) % MAX_QUEUE_SIZE;
        q->size--;
    }
    return state;
}
