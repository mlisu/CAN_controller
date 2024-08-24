#ifndef TASKS_FUNCTIONS_H_
#define TASKS_FUNCTIONS_H_

#include "can_handler_sab.h"

int echo4sendNReceiveTime(CanHandler* ch);

int readSeries4CapacityMeasurement(CanHandler* ch);

#endif /* TASKS_FUNCTIONS_H_ */
