
#include "tasks_functions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memset

#include "can_handler.h"
#include "pi_controller.h"
#include "timer.h"
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
		printf("Failed to measure send-receive time. "
			   "execTime returned 0\n");
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
/*
Sabre has a heap array of length (in bytes) equal to assumed max number of frames sent
within 10s (arbitrary chosen period).
The assumed max number is 100 kB. The shortest period of frame sending is thus
the corresponding max frequency is 10 kHz.
To factor in timer inaccuracy the Sabre buffer should be bigger (e.e. 110 kB).
*/
void sendPeriodically(CanHandler* ch)
{
	int32_t frame_nr = 0;
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
//			printf("frame_nr: %d\n", frame_nr);
			sendInt32(ch, frame_nr++);
		}
		if (ch->ufds[0].revents & POLLIN)
		{
			readCan(ch);
			frame_nr = 0;
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
			freq = (freq > max_freq) ? max_freq : freq;
			memset(stdin_buf, 0, 20);
			printf("Frequency set to: %d\n", freq);
			pollTimer_set(NANO_IN_SEC / freq, NANO_IN_SEC / freq, ch->ufds);
		}
	}
}

void controlInertia(CanHandler* ch)
{
//	ch->inOutCanFrame.can_dlc = 8;
//	memset(ch->inOutCanFrame.data, 0, sizeof(ch->inOutCanFrame.data));
	int sim_time = 0;
	int signals2send = 0;

//	poll(ch->ufds, 1, WAIT_MS_FIRST);
//	if (!(ch->ufds[0].revents & POLLIN))
//	{
//		printf("Phytec hasn't received simulation time!\n");
//		exit(1);
//	}
	sim_time = readInt32(ch); // blocking
	signals2send = sim_time / TC - 1; /* Send signal after each TC seconds of simulation, besides at
										 the end of simulation. First signal is sent after TC seconds
										 from simulation begin (==0s) and last TC before end of simulation.
									  	 - check rounding thing; */

	for (;signals2send > 0; signals2send--)
	{
		poll(ch->ufds, 1, WAIT_MS);
		if (ch->ufds[0].revents & POLLIN)
		{
			sendDouble(ch, controllerOutput(readDouble(ch)));
		}
	}
}












