#ifndef TASKS_FUNCTIONS_H_
#define TASKS_FUNCTIONS_H_

#include "can_handler.h"

int echo4sendNReceiveTime(CanHandler* ch);

int readPeriodically(CanHandler* ch);

int runSimulation(CanHandler* ch);

int runRiddleSimulation(CanHandler* ch);

#endif /* TASKS_FUNCTIONS_H_ */
