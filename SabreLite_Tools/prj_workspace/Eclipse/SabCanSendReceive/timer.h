#ifndef TIMER_H_
#define TIMER_H_

#include <stdio.h>
#include <sys/poll.h>
#include <sys/timerfd.h>

#define NANO_IN_SEC 	1000000000
#define NANO_IN_MS		1000000
#define TIMER_UFDS_IDX1 1
#define TIMER_UFDS_IDX2 3 // optionally define all 4 fds indices elsewhere

int pollTimer_config(struct pollfd * ufds, int ufds_idx);
int pollTimer_set(const long long tValueNs,
				  const long long tIntervalNs,
				  struct pollfd * ufds,
				  int ufds_idx);


#endif /* TIMER_H_ */
