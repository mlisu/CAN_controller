
#include "tasks_functions.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h> //free
#include <string.h> //memset
#include <time.h>

#include "simulation.h"
#include "timer.h"

int echo4sendNReceiveTime(CanHandler* ch)
{
	int it_cnt = readInt(ch);

	printf("it_cnt: %d\n", it_cnt);

	for (; it_cnt > 0; it_cnt--)
	{
		readNSend(ch);
	}

	return 0;
}

static int checkFramesBuf(char* buf, int frame_nr)
{
	int i;
	int result = 0;
	for (i = 0; i <= frame_nr; i++)
	{
		if(buf[i] == 0)
		{
			result = -1;
		}
	}
	if (result == 0)
	{
		printf("Received all frames within the period\n");
	}
	else
	{
		printf("Missed some frames\n");
	}
	return result;
}

static void emptyCanBuffer(CanHandler* ch, int wait_ms)
{
	/*
	 * Assumption that if wait_ms ms has passed without receiving a frame,
	 * then the can buffer is empty
	 */
	while (1)
	{
		poll(ch->ufds, CAN_IDX + 1, wait_ms);
		if (ch->ufds[CAN_IDX].revents & POLLIN)
		{
			readInt(ch);
			continue;

		}
		return;
	}
}

int readPeriodically(CanHandler* ch)
{
	int const receiving_period = 5; // seconds
	int seconds = 0;
	long long expTmp;

	char stdin_buf[20] = {0};
	char temp_char;

	int frames_in_sec = 0;
	int frame_nr = 0;
	int frame_nr_max = 0;
	int freq = 1;

	char* buf = malloc(FRAMES_BUF_LEN);
	if (buf == NULL)
	{
		printf("Memory allocation failed, aborting!\n");
		return -1;
	}
	memset(buf, 0, FRAMES_BUF_LEN);

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
			read2ints(ch, &frame_nr, &freq);

			if(frame_nr > frame_nr_max)
			{
				frame_nr_max = frame_nr;
			}

			if (frame_nr >= FRAMES_BUF_LEN)
			{
				printf("Phy has sent too many frames! Aborting\n");
				return -1;
			}
			buf[frame_nr] = 1;
		}
		if (ch->ufds[TIMER_IDX].revents & POLLIN)
		{
			read(ch->ufds[TIMER_IDX].fd, &expTmp, sizeof(long long));
			printf("Frames received per sec: %d frame_nr_max: %d\n", frames_in_sec, frame_nr_max);
			frames_in_sec = 0;
			seconds++;
			if (seconds == receiving_period)
			{
				checkFramesBuf(buf, freq*receiving_period - 1);
				seconds = 0;
				frame_nr_max = 0;
				memset(buf, 0, FRAMES_BUF_LEN);

				canWrite(ch);
				emptyCanBuffer(ch, WAIT_MS);
				canWrite(ch);

				pollTimer_set(NANO_IN_SEC, NANO_IN_SEC, ch->ufds, TIMER_IDX);
			}
		}
		if (ch->ufds[IO_IDX].revents & POLLIN)
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



static int* allocateArray(int cnt)
{
	int* ret = malloc(cnt*sizeof(int));
	if (ret == NULL)
	{
		printf("allocateArray failed to allocate memory.\n");
		exit(1);
	}
	return ret;
}

static void computeRMSratio(Simulation* sim, int* indices, int cnt)
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
			rms[0] += sim->data_vec1[j] * sim->data_vec1[j]; // signal
			rms[1] += sim->data_vec2[j] * sim->data_vec2[j]; // disturbance
		}
//		printf("f: %f\trms ratio: %f\n", f, sqrt(rms[0] / rms[1]));
//		printf("%f\n", sqrt(rms[0] / rms[1]));
		rms[0] = 0.0;
		rms[1] = 0.0;
		f += F_STEP;
	}
}

/*
 * timeExceeded checks if there already is a timer event. If so,
 * it means that the time period to which the timer is set, has passed
 */
static int timeExceeded(CanHandler* ch)
{
	poll(ch->ufds, TIMER_IDX + 1, 0);
	if (ch->ufds[TIMER_IDX].revents & POLLIN)
	{
		printf("tutaj\n");
		return 1;
	}
	return 0;
}

static void waitForTimerAndRead(CanHandler* ch)
{
	long long expTmp;

	poll(ch->ufds, TIMER_IDX + 1, -1);
	if (ch->ufds[TIMER_IDX].revents & POLLIN)
	{
		read(ch->ufds[TIMER_IDX].fd, &expTmp, sizeof(long long));
	}
}

int runSimulation(CanHandler* ch)
{
	int i;
	int first_it = 1;

	double f = FIRST_F; // disturbance frequency, Hz
	Params params = {{0, 0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	Simulation sim;
	double t_end;

	int f_nr = (LAST_F - FIRST_F) / F_STEP + 1.05;
	printf("f_nr: %d\n", f_nr);
	int* indices = allocateArray(f_nr);

	srand(time(NULL));
	initSim(&sim, suspensionModel, X_LEN, SIM_STEP, &params);

	pollTimer_config(ch->ufds, TIMER_IDX);
	pollTimer_set(SIM_STEP*NANO_IN_SEC, SIM_STEP*NANO_IN_SEC, ch->ufds, TIMER_IDX);

	for (i = 0; i < f_nr; i++)
	{
		printf("f: %f\tsim_cnt: %d\tsim.t: %f\n", f, sim.cnt, sim.t);
//		t_end = sim.t + 10/f + TR_T;
		t_end = 5; // uncomment for controller params comparision
		f_nr  = 1; // uncomment for controller params comparision
		params.data_dbl[UF_IDX] = f;
		while (sim.t < t_end)
		{
			runSim(&sim);

			// uncomment to print diagnostics data:
//			printf("time: %f\tF: %f\tout: %f\tu: %f\tf: %f\tcnt: %d\n",
//					sim.t, params.data_dbl[IN_IDX], sim.x[OUT_IDX], params.data_dbl[U_IDX], f, sim.cnt);

			if(first_it)
			{
				first_it = 0;
				sendDouble(ch, params.data_dbl[OUT_IDX]);
				waitForTimerAndRead(ch);
				continue;
			}

			poll(ch->ufds, CAN_IDX + 1, -1);
			if (ch->ufds[CAN_IDX].revents & POLLIN)
			{
				params.data_dbl[IN_IDX] = readDouble(ch);
			}

			sendDouble(ch, params.data_dbl[OUT_IDX]);
			// sprawdzić na całym zakresie f
			if(timeExceeded(ch) && !first_it) // first iteration takes longer (needed for sample times < 10 ms)
			{
				return 1;
			}

			waitForTimerAndRead(ch);
		}

		indices[i] = sim.cnt;
		f += F_STEP;
	}
	assert(i == f_nr);
	simDataToFile(&sim);
	computeRMSratio(&sim, indices, f_nr);

	deleteSim(&sim);
	free(indices);

	return 0;
}

static void send2WithIds(CanHandler* ch, Params* params)
{
	static long long i = 0;
	ch->inOutCanFrame.can_id = i++;
	sendDouble(ch, params->data_dbl[0]);
	ch->inOutCanFrame.can_id = i++;
	sendDouble(ch, params->data_dbl[1]);
}

int runRiddleSimulation(CanHandler* ch)
{
	assert(RSIM_STEPS_NR < SIM_DATA_VEC_LEN_MAX);
	int i = 0;
	unsigned char first_it = 1;

	Params params = {{0, 0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	Simulation sim;

	srand(time(NULL));

	initSim(&sim, riddleModel, X_LEN, SIM_STEP, &params);

	pollTimer_config(ch->ufds, TIMER_IDX);
	pollTimer_set(SIM_STEP*NANO_IN_SEC, SIM_STEP*NANO_IN_SEC, ch->ufds, TIMER_IDX);

	params.data_dbl[3] = MS;
	params.data_dbl[4] = IS;
	params.data_dbl[5] = WE;
	params.data_int[0] = 2600;
	params.data_int[1] = 2600;

	for (i = 1; i <= (int)(RSIM_STEPS_NR + 0.5); i++)
	{
		runSim(&sim);

		if(first_it)
		{
			first_it = 0;
			send2WithIds(ch, &params);
			waitForTimerAndRead(ch);
			continue;
		}

		poll(ch->ufds, CAN_IDX + 1, -1);
		if (ch->ufds[CAN_IDX].revents & POLLIN)
		{
			read2ints(ch, &(params.data_int[0]), &(params.data_int[1]));
			// for controller tunning and mass/frequency change
//			if(i >= 250)
//			{
////				params.data_int[0] = 600;		// for tunning
////				params.data_dbl[3] = 112;		// for mass change
////				params.data_dbl[4] = 16.13;		// for moment of inertia change
//				params.data_dbl[5] = 2*M_PI*21;
//			}

		}
//		printf("i: %d\taccf: %f\taccr: %f\n", i,  params.data_dbl[0], params.data_dbl[1]);
//		printf("c: %d\n", params.data_int[0]);

		send2WithIds(ch, &params);

		if(timeExceeded(ch))
		{
			return 1;
		}

		waitForTimerAndRead(ch);

	}

	simDataToFile(&sim);

	deleteSim(&sim);

	return 0;
}





