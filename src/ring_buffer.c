#include "ring_buffer.h"
#include "log.h"

size_t RB_COUNT;
RingBuffer_t RingBuffers[RB_COUNT_MAX];

RingBuffer_t INIT_RINGBUFFER(char* name) {
	return (RingBuffer_t) {
		.capacity = RB_SIZE,
		.name = name,
	};
}

RB_TYPE RingBuffer_avg(RingBuffer_t rb) {
	rb.read_head = 0;
	RB_TYPE sum = 0;
	for (int i = 0 ; i < RB_SIZE; i++) {
		RB_TYPE val;
		RingBuffer_get(&rb, &val);
		sum += val;
	}
	return sum / rb.count;
}
void RingBuffer_log(RingBuffer_t rb, char* suffix) {
	rb.read_head = 0;
	log("\tN  \t%-20s\n", rb.name);
	for (int i = 0 ; i < RB_SIZE; i++) {
		RB_TYPE val;
		RingBuffer_get(&rb, &val);
		log("\t%03d\t%0.2lf%s\n", i, val, suffix);
	}
}

bool RingBuffer_is_empty(RingBuffer_t rb) {
	return rb.count > 0;
}

bool RingBuffer_is_full(RingBuffer_t rb) {
	return rb.count == rb.capacity;
}


#define min(a,b) (a<=b? a : b)
void RingBuffer_put(RingBuffer_t* rb, RB_TYPE data) {
	rb->write_head = (rb->write_head + 1) % rb->capacity;
	rb->data[rb->write_head] = data;
	rb->count = min(rb->count + 1, rb->capacity);
}

void RingBuffer_get(RingBuffer_t* rb, RB_TYPE* data) {
	rb->read_head = (rb->read_head + 1) % rb->capacity;
	*data = rb->data[rb->read_head];
}

