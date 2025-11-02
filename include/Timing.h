#ifndef TIMING_H
#define TIMING_H

#include <sys/time.h>

#define SECS_PER_MIN 60
#define MSECS_PER_SEC 1000
#define USECS_PER_MSEC 1000

#define RecordTime(t) gettimeofday(&t, NULL)

#define BenchmarkFunction(span, f, ...) \
	{	timeval before, after;				\
		RecordTime(before);				\
		f(__VA_ARGS__);					\
		RecordTime(after);				\
		span= GetTimespan(before, after);}		\

typedef struct timeval wc_timeval; // im lazy

typedef struct timespan {
	time_t m;
	time_t s;
	time_t ms;
	time_t us;
} Timespan;

Timespan GetTimespan(wc_timeval before, wc_timeval after);
void PrintTimespan(Timespan t);
#endif //TIMING_H
