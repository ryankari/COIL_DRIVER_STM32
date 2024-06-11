
#ifndef QUEUE_H
	#define QUEUE_H
	#define MAX_QUEUE_SIZE 10

	typedef enum {
		STATE_IDLE = 0,
		STATE_TIM2,
		STATE_TIM3,
		STATE_USB_RECEIVED
	} State_t;

	typedef struct {
		State_t queue[MAX_QUEUE_SIZE];
		uint8_t front;
		uint8_t rear;
		uint8_t size;
	} StateQueue_t;


	void initQueue(StateQueue_t* q);
	int isQueueFull(StateQueue_t* q) ;
	int isQueueEmpty(StateQueue_t* q);
	void enqueue(StateQueue_t* q, State_t state);
	State_t dequeue(StateQueue_t* q);


#endif
