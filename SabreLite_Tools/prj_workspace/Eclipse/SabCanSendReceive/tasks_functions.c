
#include "tasks_functions.h"

#include <stdio.h>
#include <stdint.h> //byte type (e.g. int8_t)
#include <stdlib.h> //free
#include <string.h> //memset

#include "simulation.h"
#include "timer.h"

int echo4sendNReceiveTime(CanHandler* ch)
{
	int32_t it_cnt = readInt32(ch); // read is blocking - will wait for frame

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

int checkFramesBuf(char* buf, int frame_nr)
{
	int i;
	int result = 0;
	for (i = 0; i <= frame_nr; i++)
	{
		if(buf[frame_nr] == 0)
		{
			printf("Frame nr: %d not received\n", frame_nr);
			result = -1;
		}
	}
	if (result == 0)
	{
		printf("Received all frames within 10s period\n");
	}
	return result;
}

int readPeriodically(CanHandler* ch)
{
	int const receiving_period = 10; // seconds
	int seconds = 0;

	char stdin_buf[20] = {0};
	char temp_char;

	int frames_in_sec = 0;
	int frame_nr = 0;

	char* buf = malloc(FRAMES_BUF_LEN);
	if (buf == NULL)
	{
		printf("Memory allocation failed, aborting!\n");
		return -1;
	}

	long long int expTmp;
	pollTimer_config(ch->ufds);
	pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds);

	ch->ufds[2].fd = STDIN_FILENO;
	ch->ufds[2].events = POLLIN;

	while (1)
	{
		poll(ch->ufds, 3, -1);

		if (ch->ufds[0].revents & POLLIN)
		{
			frames_in_sec++;
			frame_nr = readInt32(ch);
			if (frame_nr >= FRAMES_BUF_LEN)
			{
				printf("Phy has sent too many frames! Aborting\n");
				return -1;
			}
			buf[frame_nr] = 1;
		}
		if (ch->ufds[1].revents & POLLIN)
		{
			read(ch->ufds[1].fd, &expTmp, sizeof(long long int));
			printf("Frames received per sec: %d frame_nr: %d\n", frames_in_sec, frame_nr);
			frames_in_sec = 0;
			seconds++;
			if (seconds == receiving_period)
			{
				checkFramesBuf(buf, frame_nr);
				seconds = 0;
				memset(buf, 0, FRAMES_BUF_LEN);

				canWrite(ch); // notify Phy to set frame_nr to zero
			}
		}
		if (ch->ufds[2].revents & POLLIN)
		{
			scanf("%[^\n]", stdin_buf);
			scanf("%c", &temp_char);
			if (*stdin_buf == 'q')
			{
				break;
			}
		}
	}

	free(buf);
	return 0;
}

int runInertiaSimulation(CanHandler* ch)
{
//	int missed_number = 0;
	int idx = 0;
	double ctrl_signal = 0;

	FileHandler fh;
	initFileHandler(&fh);

	sendInt32(ch, STIME);
	sendDouble(ch, INIT_STATE); //check if 2 frames will be received after being
								// sent so without interval break

	/*
	  computes CTR_SYS_RATIO times, so first at the moment of simulation start
	  (==0s) and then "after each T seconds" until
	  sim time <= (CTR_SYS_RATIO - 1) * T. After that, next computation is
	  being done in the while loop, beginning from CTR_SYS_RATIO * T seconds
	  (having already input from controller)
	*/
	computOutputBetweenCtrlSignals(&fh, &idx, ctrl_signal);

	while (idx < SIM_DATA_VEC_LEN)
	{
//		for (i = 0; i < CTR_SYS_RATIO; i++)
//		{
//			fh.data_vec[idx] = inertiaOutput(ctrl_signal);
//			idx++;
//		}

		poll(ch->ufds, 1, WAIT_MS);
		if (ch->ufds[0].revents & POLLIN)
		{
			ctrl_signal = readDouble(ch);
		}

		computOutputBetweenCtrlSignals(&fh, &idx, ctrl_signal);

		sendDouble(ch, fh.data_vec[idx - 1]);
	}

	simDataToFile(&fh);

	return 0;
}






