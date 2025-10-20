#ifndef RING_BUFFER_H
#define RING_BUFFER_H
// a ring buffer basically just writes with modulo.
#include <stddef.h>


#define RB_COUNT_MAX 64
extern size_t RB_COUNT;

#define RB_SIZE 1024
#define RB_TYPE double

#define init_RingBuffer(name) RingBuffers[RB_COUNT++] = INIT_RINGBUFFER(name)

typedef struct RingBuffer {
	RB_TYPE data[RB_SIZE];
	size_t write_head; // next pos to be written to
	size_t read_head;  // next pos to be read from
	size_t count;

	char *name;
	size_t capacity;
} RingBuffer_t;
RingBuffer_t RingBuffers[RB_COUNT_MAX];

RingBuffer_t INIT_RINGBUFFER(char* name);
void destroy_RingBuffer(RingBuffer_t rb);

bool RingBuffer_is_empty(RingBuffer_t rb);
bool RingBuffer_is_full(RingBuffer_t rb);
void RingBuffer_put(RingBuffer_t* rb, RB_TYPE data);
void RingBuffer_get(RingBuffer_t* rb, RB_TYPE* data);
void RingBuffer_log(RingBuffer_t rb, char* suffix);
RB_TYPE RingBuffer_avg(RingBuffer_t rb);
#endif // RING_BUFFER_H
