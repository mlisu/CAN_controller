#ifndef TASKS_FUNCTIONS_H_
#define TASKS_FUNCTIONS_H_

#include <stdint.h> //byte type (e.g. int8_t)

#include "can_handler.h"

int sendNReceiveTime(CanHandler* ch, int32_t it_cnt);

void sendSeries4CapacityMeasurement(CanHandler* ch, int32_t it_cnt);

void controlInertia(CanHandler* ch);

#endif /* TASKS_FUNCTIONS_H_ */
