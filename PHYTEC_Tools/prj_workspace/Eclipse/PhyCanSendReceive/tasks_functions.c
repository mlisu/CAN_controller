
#include "tasks_functions.h"

#include <stdio.h>

#include "can_handler.h"
/*
	Podaje tutaj it_cnt jako int mimo że calcExecTime przyjmuje size_t
	Jest tak bo calcExecTime przyjmując size_t jest bardziej ogólna a tutaj
	jest wygodniej korzystać z inta bo wystarczy jedna funkcja zamieniająca
	liczbę na data i odwrotnie (int32).
*/
int sendNReceiveTime(CanHandler* ch, int32_t it_cnt)
{
	uint32_t execTime;

	sendInt32(ch, it_cnt + 1); //+1 for cache warm up

	execTime = calcExecTime(ch, sendNReceive, it_cnt);
	if (execTime == 0)
	{
		return -1;
	}

	printf("SendNReceive time: %u\n", execTime);
	return 0;
}

void sendSeries4CapacityMeasurement(CanHandler* ch, int32_t it_cnt)
{
	sendInt32(ch, it_cnt);
	sendSeries(ch, it_cnt);
}

void controlInertia(CanHandler* ch)
{
	ch->inOutCanFrame.can_dlc = 8;
	memset(ch->inOutCanFrame.data, 0, sizeof(ch->inOutCanFrame.data));

	poll(ch->ufds, 1, WAIT_MS);
	if (ch->ufds[0].revents & POLLIN)
	{
		readCan(ch);
	}
	sendDouble(controllerOutput(readDouble(ch)));
}












