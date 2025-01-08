#ifndef TASKS_FUNCTIONS_H_
#define TASKS_FUNCTIONS_H_

#include "can_handler.h"

int sendNReceiveTime(CanHandler* ch, int it_cnt);

void sendPeriodically(CanHandler* ch);

void controlSuspension(CanHandler* ch);

void controlRiddle(CanHandler* ch);

#endif /* TASKS_FUNCTIONS_H_ */
