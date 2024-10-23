#ifndef TIMER_H_
#define TIMER_H_

#include <stdio.h>
#include <sys/poll.h>
#include <sys/timerfd.h>

#define NANO_IN_SEC 1000000000
#define TIMER_UFDS_IDX 1

int pollTimer_config(struct pollfd * ufds);
int pollTimer_set(const long long tValueNs, const long long tIntervalNs, struct pollfd * ufds);


#endif /* TIMER_H_ */
