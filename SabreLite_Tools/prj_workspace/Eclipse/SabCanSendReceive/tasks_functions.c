
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
		if(buf[i] == 0)
		{
//			printf("Frame nr: %d not received\n", i);
			result = -1;
		}
	}
	if (result == 0)
	{
		printf("Received all frames within 10s period\n");
	}
	else
	{
		printf("Missed some frames with id <= frame_nr\n");
	}
	return result;
}

// Helper function for debugging - to remove
void printBufSum(char* buf, int frame_nr)
{
	int i;
	int acc = 0;
	for (i = 0; i <= frame_nr; i++)
	{
		acc += buf[i];
	}
	printf("buf sum: %d\n", acc);
}

void emptyCanBuffer(CanHandler* ch)
{
	/*
	 * Assumption that if WAIT_MS == 300 ms has passed without receiving a frame,
	 * then the can buffer is empty
	 */
	while (1)
	{
		poll(ch->ufds, 3, WAIT_MS);
		if (ch->ufds[0].revents & POLLIN)
		{
			printf("Clear buffer frame nr: %d\n", readInt32(ch));
			continue;
		}
		break;
	}
}

int readPeriodically(CanHandler* ch)
{
	int const receiving_period = 5; // seconds
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
	pollTimer_config(ch->ufds, TIMER_UFDS_IDX1);
	pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds, TIMER_UFDS_IDX1);

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

				canWrite(ch);

				emptyCanBuffer(ch);

				canWrite(ch);
				pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds, TIMER_UFDS_IDX1);
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
			memset(stdin_buf, 0, 20);
		}
	}

	free(buf);
	return 0;
}

int runInertiaSimulation(CanHandler* ch)
{
	int i = 0;
	double ctrl_signal = 0.0;
	double inertia_output = 0.0;
	long long int expTmp;

	FileHandler fh;
	initFileHandler(&fh);

	pollTimer_config(ch->ufds, TIMER_UFDS_IDX1);
	pollTimer_set(T*NANO_IN_SEC, T*NANO_IN_SEC, ch->ufds, TIMER_UFDS_IDX1);

	pollTimer_config(ch->ufds, TIMER_UFDS_IDX2);
	pollTimer_set(TSEN*NANO_IN_MS, TSEN*NANO_IN_MS, ch->ufds, TIMER_UFDS_IDX2);

	printf("sim data vec len: %d\n", SIM_DATA_VEC_LEN);

	for (i = 1; i < SIM_DATA_VEC_LEN;) // starts from 1, since zero data is already there
	{
		poll(ch->ufds, 4, -1);

		if (ch->ufds[0].revents & POLLIN) // can control signal came
		{
			ctrl_signal = readDouble(ch);
		}
		if (ch->ufds[1].revents & POLLIN) // inertia state actualization timer tick
		{
			read(ch->ufds[1].fd, &expTmp, sizeof(long long int));
			inertia_output = inertiaOutput(ctrl_signal);
			printf("i: %d\toutput: %f\tctrl signal: %f\n", i, inertia_output, ctrl_signal);
			fh.data_vec[i++] = inertia_output;
//			printf("Inertia state timer\n");
		}
		if (ch->ufds[3].revents & POLLIN) // sensor timer tick
		{
			read(ch->ufds[3].fd, &expTmp, sizeof(long long int));
			sendDouble(ch, inertia_output);
//			printf("Sensor timer\n");
		}
	}

	simDataToFile(&fh);

	return 0;
}






