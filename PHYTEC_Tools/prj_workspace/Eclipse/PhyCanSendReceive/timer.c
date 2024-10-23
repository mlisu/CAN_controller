#include "timer.h"


int pollTimer_config(struct pollfd * ufds)
{
        int tfd;

        if ((tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK )) < 0)
                perror("timerfd create error");

        ufds[TIMER_UFDS_IDX].fd = tfd;
        ufds[TIMER_UFDS_IDX].events = POLLIN;

        return 0;
}

int pollTimer_set(const long long tValueNs, const long long tIntervalNs, struct pollfd * ufds)
{
        struct itimerspec newValue;
        long tValue_sec, tValue_nsec, tInterval_sec, tInterval_nsec;

        tValue_sec = tValueNs/NANO_IN_SEC;
        tValue_nsec = tValueNs - tValue_sec * NANO_IN_SEC;
        newValue.it_value.tv_sec=tValue_sec;
        newValue.it_value.tv_nsec=tValue_nsec;

        tInterval_sec = tIntervalNs/NANO_IN_SEC;
        tInterval_nsec = tIntervalNs - tInterval_sec * NANO_IN_SEC;
        newValue.it_interval.tv_sec=tInterval_sec;
        newValue.it_interval.tv_nsec=tInterval_nsec;

        if( timerfd_settime(ufds[TIMER_UFDS_IDX].fd,0,&newValue,NULL) <0)
        {
                perror("settime error");
        }

        return 0;
}

