#ifndef __TIME_UTILS_H_
#define __TIME_UTILS_H_

#include <time.h>

#define FLL_NANOSECONDS_IN_MILISECOND 		1000000
#define FLL_NANOSECONDS_IN_SECOND 		1000000000
#define FLL_MICROSECONDS_IN_MILISECOND 		1000
#define FLL_MICROSECONDS_IN_SECOND 		1000000
#define FLL_NANOSECONDS_IN_MICROSECOND 		1000
#define FLL_MILISECONDS_IN_SECOND 		1000

static inline void timeval_zero(struct timeval *r)
{
	r->tv_sec = 0;
	r->tv_usec = 0;
}

static inline int timeval_subtract (struct timeval *r,
				    const struct timeval *t1,
				    const struct timeval *t2)
{
	if (t1->tv_sec > t2->tv_sec) {
		r->tv_sec = t1->tv_sec - t2->tv_sec;
		r->tv_usec = t1->tv_usec - t2->tv_usec;

	}
	else {
		r->tv_sec = t2->tv_sec - t1->tv_sec;
		r->tv_usec = t2->tv_usec - t1->tv_usec;
	}
	
	if (r->tv_usec < 0) {
		r->tv_sec--;
		r->tv_usec += FLL_MICROSECONDS_IN_SECOND;
	}

	/* Return 1 if result is negative. */
	return r->tv_sec < 0;
}

static inline void timeval_add(struct timeval *r,
			       const struct timeval *t)
{
	r->tv_sec += t->tv_sec;
	r->tv_usec += t->tv_usec;
	if (r->tv_usec >= FLL_MICROSECONDS_IN_SECOND) {
		r->tv_sec++;
		r->tv_usec -= FLL_MICROSECONDS_IN_SECOND;
	}
}

static inline unsigned long timeval_usecs(struct timeval *t)
{
	return t->tv_sec * FLL_MICROSECONDS_IN_SECOND + t->tv_usec;
}

static inline unsigned long timeval_msecs(struct timeval *t)
{
	return (t->tv_sec * FLL_MILISECONDS_IN_SECOND +
		t->tv_usec / FLL_MICROSECONDS_IN_MILISECOND);
}


static inline void timespec_zero(struct timespec *r)
{
	r->tv_sec = 0;
	r->tv_nsec = 0;
}

static inline int timespec_substract(struct timespec *const r,
			       const struct timespec *const t1,
			       const struct timespec *const t2)
{
	if (t1->tv_sec >= t2->tv_sec) {
		r->tv_sec = t1->tv_sec - t2->tv_sec;
		r->tv_nsec = t1->tv_nsec - t2->tv_nsec;

	}
	else {
		r->tv_sec = t2->tv_sec - t1->tv_sec;
		r->tv_nsec = t2->tv_nsec - t1->tv_nsec;
	}
	
	if (r->tv_nsec < 0) {
		r->tv_sec--;
		r->tv_nsec += FLL_NANOSECONDS_IN_SECOND;
	}

	/* Return 1 if result is negative. */
	return r->tv_sec < 0;
}

static inline void timespec_add(struct timespec *r,
				const struct timespec *t)
{
	r->tv_sec += t->tv_sec;
	r->tv_nsec += t->tv_nsec;
	if (r->tv_nsec >= FLL_NANOSECONDS_IN_SECOND) {
		r->tv_sec++;
		r->tv_nsec -= FLL_NANOSECONDS_IN_SECOND;
	}
}


static inline unsigned long timespec_usecs(struct timespec *t)
{
	return (t->tv_sec * FLL_MICROSECONDS_IN_SECOND +
		t->tv_nsec / FLL_NANOSECONDS_IN_MICROSECOND);
}

static inline unsigned long timespec_msecs(struct timespec *t)
{
	return (t->tv_sec * FLL_MILISECONDS_IN_SECOND +
		t->tv_nsec / FLL_NANOSECONDS_IN_MILISECOND);
}


#endif
