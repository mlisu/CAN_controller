#include "can_handler.h"

#include <net/if.h>
#include <stdio.h>
#include <string.h> //strcpy
#include <sys/ioctl.h>
#include <sys/socket.h>

int initCanHandler(CanHandler* ch)
{
	if (!(ch->canSocket = canConfig()))
	{
		perror("Can config error\n");
		return -1;
	}

	memset(ch->ufds, 0, sizeof(ch->ufds));
	ch->ufds[0].fd = ch->canSocket;
	ch->ufds[0].events = POLLIN;

	//default can frame:
	ch->inOutCanFrame.can_id = 111;
	ch->inOutCanFrame.can_dlc = 1;

	return 0;
}

void closeCanHandler(CanHandler* ch)
{
	close(ch->canSocket);
}

int canConfig()
{
	int l_canSocket;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if ((l_canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		printf("socket error\n");
		return 0;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, CANBUS);
	if (ioctl(l_canSocket, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		printf("SIOCGIFINDEX error\n");
		return 0;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(l_canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		printf("bind error\n");
		return 0;
	}

	return l_canSocket;
}

ssize_t readCan(CanHandler* ch)
{
	return read(ch->canSocket, &ch->inOutCanFrame, sizeof(struct can_frame));
}

int32_t readInt32(CanHandler* ch)
{
	readCan(ch);
	return *(int32_t*)ch->inOutCanFrame.data;
}

double readDouble(CanHandler* ch)
{
	readCan(ch);
	return *(double*)ch->inOutCanFrame.data;
}

void sendInt32(CanHandler* ch, int32_t data_in)
{
	ch->inOutCanFrame.can_dlc = 4;
	*(int32_t*)ch->inOutCanFrame.data = data_in;
	canWrite(ch);
}

void send2ints(CanHandler* ch, int first, int second)
{
	ch->inOutCanFrame.can_dlc = 2*sizeof(int);
	*(int*)ch->inOutCanFrame.data = first;
	*((int*)ch->inOutCanFrame.data + 1) = second;
	canWrite(ch);
}

void sendDouble(CanHandler* ch, double data_in)
{
	ch->inOutCanFrame.can_dlc = 8;
	*(double*)ch->inOutCanFrame.data = data_in;
	canWrite(ch);
}

ssize_t canWrite(CanHandler* ch)
{
	return write(ch->canSocket, &ch->inOutCanFrame, sizeof(ch->inOutCanFrame));
}

ssize_t sendNReceive(CanHandler* ch)
{
	ch->inOutCanFrame.can_id = 111;
	canWrite(ch);

	poll(ch->ufds, 1, WAIT_MS); // można usunąc
	if (ch->ufds[0].revents & POLLIN)
	{
		readCan(ch);
		return 0;
	}

	printf("No response on a sent frame before timeout!\n");
	return -1;
}

void sendSeries(CanHandler* ch, int32_t it_cnt)
{
	int i;
	for (i = 0; i < it_cnt; i++)
	{
		ch->inOutCanFrame.can_id = i;
		canWrite(ch);
	}
	printf("can id: %d\n", ch->inOutCanFrame.can_id);
}

uint32_t calcExecTime(CanHandler* ch,
					  ssize_t (*fn)(CanHandler*),
					  int32_t it_cnt)
{
	int32_t i;
	struct timespec timeStampOld, timeStampNew;
	uint64_t acc_time = 0;

	/*
		Poniżej wykonuje 1x funkcję której czas jest mierzony.
		Potem w pętli for wykonuje it_cnt razy, a więc w sumie
		funkcja jest wykonywana it_cnt + 1 razy.
	*/
	if (fn(ch) == -1) //cache warm up
	{
		printf("\nTime calculation failed!\n");
		return 0;
	}

	clock_gettime(CLOCK_MONOTONIC, &timeStampOld);
	for (i = 0; i < it_cnt; i++)
	{
		if (fn(ch) == -1)
		{
			printf("\nTime calculation failed!\n");
			return 0;
		}
	}
	clock_gettime(CLOCK_MONOTONIC, &timeStampNew);
	acc_time = execTime_count(&timeStampOld, &timeStampNew);

	return acc_time / it_cnt;
}

int64_t execTime_count(struct timespec* timeStartPtr, struct timespec* timeStopPtr)
{
	int64_t cpt_ns, cpt_s;

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













