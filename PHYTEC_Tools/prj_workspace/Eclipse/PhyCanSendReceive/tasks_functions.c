
#include "tasks_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memset

#include "can_handler.h"
#include "controller.h"
#include "timer.h"

static long long execTime_count(struct timespec* timeStartPtr,
								struct timespec* timeStopPtr)
{
	long long cpt_ns, cpt_s;

	cpt_s = (timeStopPtr->tv_sec - timeStartPtr->tv_sec) * NANO_IN_SEC;
	if (timeStopPtr->tv_nsec > timeStartPtr->tv_nsec)
	{
		cpt_ns = timeStopPtr->tv_nsec - timeStartPtr->tv_nsec;
	}
	else
	{
		cpt_s -= NANO_IN_SEC;
		cpt_ns = NANO_IN_SEC + timeStopPtr->tv_nsec - timeStartPtr->tv_nsec;
	}
	return (cpt_s + cpt_ns);
}

static double calcExecTime(CanHandler* ch,
						   ssize_t (*fn)(CanHandler*),
						   int it_cnt)
{
	int i;
	struct timespec timeStampOld, timeStampNew;
	long long acc = 0;
	int curr_t;
	int tab[40] = {0};

	fn(ch); //cache warm up

	clock_gettime(CLOCK_MONOTONIC, &timeStampOld);
	for (i = 0; i < it_cnt; i++)
	{
		clock_gettime(CLOCK_MONOTONIC, &timeStampOld);
		fn(ch);
		clock_gettime(CLOCK_MONOTONIC, &timeStampNew);
		curr_t = execTime_count(&timeStampOld, &timeStampNew);
		acc += curr_t;

		if(curr_t < 200000 || curr_t >= 1000000)
		{
			printf("t < 200 us || > 1 ms: %d\n", curr_t);
			continue;
		}

		tab[(curr_t - 200000) / 20000]++;
	}

	for(i = 0; i < 40; i++)
	{
		printf("i: %d\ttab[i]: %d\n", i, tab[i]);
	}

	return (double)acc / it_cnt;
}



int sendNReceiveTime(CanHandler* ch, int it_cnt)
{
	double execTime;

	sendInt(ch, it_cnt + 1); //+1 for cache warm up

	execTime = calcExecTime(ch, sendNReceive, it_cnt);

	printf("SendNReceive time: %f\n", execTime);
	return 0;
}

/*
Sabre has a heap array of length (in bytes) equal to assumed max number of frames sent
within 10s (arbitrary chosen period).
The assumed max number is 100 kB. The shortest period of frame sending is thus 100 us
the corresponding max frequency is 10 kHz.
To factor in timer inaccuracy the Sabre buffer should be bigger (e.g. 110 kB).
*/
void sendPeriodically(CanHandler* ch)
{
	int frame_nr = 0;
	int const max_freq = 10000; // Hz
	int freq = 1; 				// Hz

	char stdin_buf[20] = {0};
	char temp_char;

	long long int expTmp;
	pollTimer_config(ch->ufds);
	pollTimer_set(NANO_IN_SEC / freq, NANO_IN_SEC / freq, ch->ufds);

	ch->ufds[2].fd = STDIN_FILENO;
	ch->ufds[2].events = POLLIN;

	while (1)
	{
		poll(ch->ufds, 3, -1);

		if (ch->ufds[1].revents & POLLIN)
		{
			read(ch->ufds[1].fd, &expTmp, sizeof(long long int));
			send2ints(ch, frame_nr, freq);
			frame_nr++;
		}
		if (ch->ufds[0].revents & POLLIN)
		{
//			printf("%d\n", readInt32(ch));
			readCan(ch); // Sabre told to stop sending
			frame_nr = 0;
//			printf("%d\n", readInt32(ch));
			readCan(ch); // Sabre told that it cleaned the CAN buffer

		}
		if (ch->ufds[2].revents & POLLIN)
		{
			scanf("%[^\n]", stdin_buf);
			scanf("%c", &temp_char);
			if (*stdin_buf == 'q')
			{
				break;
			}
			freq = atoi(stdin_buf);
			if(freq > max_freq)
			{
				freq = max_freq;
			}

			printf("Frequency set to: %d\n", freq);
			pollTimer_set(NANO_IN_SEC / freq, NANO_IN_SEC / freq, ch->ufds);
			memset(stdin_buf, 0, 20);
		}
	}
}

static double diff = 0.0;

static void controlSuspensionImpl(CanHandler* ch, double out_ref)
{
	double input = readDouble(ch);
	sendDouble(ch, PIDoutput(input, out_ref));
	diff += input*input;
}

static void controlRiddleImpl(CanHandler* ch, double out_ref)
{
	static double accf = 0.0;
	double acc;
	double rms;
	int ctrl;
	static long long id = -1;
	static long long i = 0;

	acc = readDouble(ch);

	if (!(ch->inOutCanFrame.can_id % 2))
	{
		id = ch->inOutCanFrame.can_id;
		accf = acc;
		return;
	}

	if((ch->inOutCanFrame.can_id - id) != 1)
	{
		printf("Broken frames order, skipping control signal\n");
		return;
	}

	rms = computeRMS(accf, acc);
//	printf("RMS: %f\ti: %d\taccf: %f\taccr: %f\n", rms, i, accf, acc);
	if(++i >= 2500) out_ref = 22.0; // for RMS change analysis
	ctrl = riddleControl(rms, out_ref);

	send2ints(ch, ctrl, ctrl);

//	printf("RMS: %f\tcontrol: %d\n", rms, ctrl);
	printf("%f\n", rms);

	diff += (rms - ROUT_REF) * (rms - ROUT_REF);
}

static void control(CanHandler* ch,
					double const out_ref,
					void (*fn)(CanHandler* ch, double out_ref))
{
	char stdin_buf[20] = {0};
	char temp_char;

	ch->ufds[2].fd = STDIN_FILENO;
	ch->ufds[2].events = POLLIN;

	while (1)
	{
		poll(ch->ufds, 3, -1);
		if (ch->ufds[0].revents & POLLIN)
		{
			fn(ch, out_ref);
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
	printf("sum diff: %f\n", diff);
//	printf("KCP: %d\n", KCP);
//	printf("sum diff: %f\tTCD: %f\n", diff, TCD);
}

void controlSuspension(CanHandler* ch)
{
	control(ch, OUT_REF, controlSuspensionImpl);
}

void controlRiddle(CanHandler* ch)
{
	control(ch, ROUT_REF, controlRiddleImpl);
}












