
#include "tasks_functions.h"

#include <stdio.h>
#include <stdint.h> //byte type (e.g. int8_t)

#include "simulation.h"

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

int runInertiaSimulation(CanHandler* ch)
{
//	int missed_number = 0;
	int i;
	int idx = 1;
	double ctrl_signal = 0;

	FileHandler fh;
	initFileHandler(&fh);

	sendInt32(ch, STIME);
	sendDouble(ch, INIT_STATE); //check if 2 frames will be reveived after being
								// sent so without interval break

	while (idx < SIM_DATA_VEC_LEN)
	{
		poll(ch->ufds, 1, WAIT_MS);
		if (ch->ufds[0].revents & POLLIN)
		{
			ctrl_signal = readDouble(ch);
		}
//		ctrl_signal = controllerOutput(fh.data_vec[idx-1]);
		for (i = 0; i < CTR_SYS_RATIO; i++)
		{
			fh.data_vec[idx] = inertiaOutput(ctrl_signal);
			idx++;
		}

		sendDouble(ch, fh.data_vec[idx - 1]);
	}

	simDataToFile(&fh);

	return 0;
}






