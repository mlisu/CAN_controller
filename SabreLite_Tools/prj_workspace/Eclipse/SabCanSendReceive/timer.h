#ifndef TIMER_H_
#define TIMER_H_

#include <stdio.h>
#include <sys/poll.h>
#include <sys/timerfd.h>

#define NANO_IN_SEC 	1000000000
#define NANO_IN_MS		1000000

int pollTimer_config(struct pollfd* ufds, int ufds_idx);

int pollTimer_set(const long long tValueNs,
				  const long long tIntervalNs,
				  struct pollfd * ufds,
				  int ufds_idx);

void tryReadTimer(struct pollfd* ufd, long long int* expTmp);

#endif /* TIMER_H_ */
