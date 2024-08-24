
#include "tasks_functions.h"

#include <stdio.h>
#include <stdint.h> //byte type (e.g. int8_t)

int echo4sendNReceiveTime(CanHandler* ch)
{
	int32_t it_cnt = readInt32(ch);

	printf("it_cnt: %d\n", it_cnt);

	for (; it_cnt > 0; it_cnt--)
	{
//		printf("it_cnt in echo: %d\n", it_cnt);
		if (readNSend(ch) == -1)
		{
			return -1;
		}
	}

	return 0;
}

int readSeries4CapacityMeasurement(CanHandler* ch)
{
	int32_t it_cnt = readInt32(ch);

	printf("Series size: %d\n", it_cnt);

	if (readSeries(ch, it_cnt) == -1)
	{
		return -1;
	}

	return 0;
}

