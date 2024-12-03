
#include "tasks_functions.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h> //byte type (e.g. int8_t)
#include <stdlib.h> //free
#include <string.h> //memset
#include <time.h>

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

void emptyCanBuffer(CanHandler* ch, int wait_ms)
{
	/*
	 * Assumption that if WAIT_MS == 300 ms has passed without receiving a frame,
	 * then the can buffer is empty
	 */
	while (1)
	{
		poll(ch->ufds, 3, wait_ms);
		if (ch->ufds[CAN_IDX].revents & POLLIN)
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
	pollTimer_config(ch->ufds, TIMER_IDX);
	pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds, TIMER_IDX);

	ch->ufds[2].fd = STDIN_FILENO;
	ch->ufds[2].events = POLLIN;

	while (1)
	{
		poll(ch->ufds, 3, -1);

		if (ch->ufds[CAN_IDX].revents & POLLIN)
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

				emptyCanBuffer(ch, WAIT_MS);

				canWrite(ch);
				pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds, TIMER_IDX);
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

float ticksToMs(clock_t ticks)
{
	return (float)ticks / CLOCKS_PER_SEC * 1000;
}

int* allocateArray(int cnt)
{
	int* ret = malloc(cnt*sizeof(int));
	if (ret == NULL)
	{
		printf("allocateArray failed to allocate memory.\n");
		exit(1);
	}
	return ret;
}

void computeRMSratio(Simulation* sim, int* indices, int cnt)
{
	int i, j;
	int sampl_nr;
	double f = FIRST_F;
	double rms[2] = {0.0, 0.0};
	for (i = 0; i < cnt; i++)
	{
		sampl_nr = 10/f/SIM_STEP + 0.5;
		for (j = indices[i] - (sampl_nr - 1); j <= indices[i]; j++)
		{
			rms[0] += sim->data_vec[j] * sim->data_vec[j]; // signal
			rms[1] += sim->u_vec[j]    * sim->u_vec[j];	   // disturbance
		}
		printf("f: %f\trms ratio: %f\n", f, sqrt(rms[0] / rms[1]));
		rms[0] = 0.0;
		rms[1] = 0.0;
		f += F_STEP;
	}
}

int runSimulation(CanHandler* ch)
{
	int i = 0;
	long long int expTmp;
	float dt_ms;
	clock_t t;

	double f; // disturbance frequency, Hz
	double params[PARAM_LEN] = {0.0}; // first array member is ctrl signal
	Simulation sim;
	double t_end;

	int f_nr = (LAST_F - FIRST_F) / F_STEP + 1;
	int* indices = allocateArray(f_nr);

	srand(time(NULL));
//	initSim(&sim, inertiaModel, X_LEN, SIM_STEP, params);
	initSim(&sim, suspensionModel, X_LEN, SIM_STEP, params);

	pollTimer_config(ch->ufds, TIMER_IDX);
	pollTimer_set(SIM_STEP*NANO_IN_SEC, SIM_STEP*NANO_IN_SEC, ch->ufds, TIMER_IDX);

	for (f = FIRST_F; f < (LAST_F + 0.1); f += F_STEP)
	{
		t_end = sim.t + 10/f + TR_T; // TR_T == 1 s
		params[UF_IDX] = f;
		while (sim.t < t_end)
		{
			t = clock();
			runSim(&sim);
			dt_ms = SIM_STEP * 1000 - ticksToMs(clock() - t);
			printf("time: %f\tF: %f\tout: %f\tu: %f\tf: %f\tcnt: %d\n", sim.t, params[IN_IDX], sim.x[OUT_IDX], params[U_IDX], f, sim.cnt);
			if (dt_ms <= 1)
			{
				printf("Simulation step took longer than (SIM_STEP - 1 ms)\n");
				return 1;
			}

			sendDouble(ch, (double)sim.x[OUT_IDX]); // change to sendFloat - all date to be changed to float
			poll(ch->ufds, CAN_IDX + 1, dt_ms * 0.9);
			if (ch->ufds[CAN_IDX].revents & POLLIN)
			{
				params[IN_IDX] = readDouble(ch); // change to readFloat
			}
			else
			{
				printf("Control signal has not come.\n"); // here recovering can be implemented
				exit(1);
			}

			if (( SIM_STEP * 1000 - ticksToMs(clock() - t) ) < 0.5)
			{
				printf("Simulation step took longer than (SIM_STEP - 0.5 ms)\n");
				return 1;
			}

			poll(ch->ufds, TIMER_IDX + 1, -1);
			tryReadTimer(&ch->ufds[TIMER_IDX], &expTmp);
		}
		indices[i++] = sim.cnt;
	}
	assert(i == f_nr);
	simDataToFile(&sim);
	computeRMSratio(&sim, indices, f_nr);

	return 0;
}






