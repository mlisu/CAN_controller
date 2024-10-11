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
//	printf("dlc: %d\n", ch->inOutCanFrame.can_dlc);
//	printf("data: 0: %d\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n",
//				ch->inOutCanFrame.data[0],
//				ch->inOutCanFrame.data[1],
//				ch->inOutCanFrame.data[2],
//				ch->inOutCanFrame.data[3],
//				ch->inOutCanFrame.data[4],
//				ch->inOutCanFrame.data[5],
//				ch->inOutCanFrame.data[6],
//				ch->inOutCanFrame.data[7]);
	return *(double*)ch->inOutCanFrame.data;
}

//ssize_t waitNRead(CanHandler* ch, int wait_ms)
//{
//	poll(ch->ufds, 1, wait_ms);
//	if (ch->ufds[0].revents & POLLIN)
//	{
//		readCan(ch);
//		return 0;
//	}
//	printf("waitNRead timed out!\n");
//	return -1;
//}

void sendInt32(CanHandler* ch, int32_t data_in)
{
	ch->inOutCanFrame.can_dlc = 4;
	*(int32_t*)ch->inOutCanFrame.data = data_in;
//	printf("data: 0: %d\t1: %d\t2: %d\t3: %d\n",
//			ch->inOutCanFrame.data[0],
//			ch->inOutCanFrame.data[1],
//			ch->inOutCanFrame.data[2],
//			ch->inOutCanFrame.data[3]);
	canWrite(ch);
}

void sendDouble(CanHandler* ch, double data_in)
{
	ch->inOutCanFrame.can_dlc = 8;
	*(double*)ch->inOutCanFrame.data = data_in;
	canWrite(ch);
}

//void setFrame(CanHandler* ch, uint32_t id, uint8_t dlc, uint8_t* data)
//{
//	ch->inOutCanFrame.can_id = id;
//	ch->inOutCanFrame.can_dlc = dlc;
//	*(unsigned long long*)ch->inOutCanFrame.data = *(unsigned long long*)data;
//}

ssize_t canWrite(CanHandler* ch)
{
	return write(ch->canSocket, &ch->inOutCanFrame, sizeof(ch->inOutCanFrame));
}

ssize_t sendNReceive(CanHandler* ch)
{
	ch->inOutCanFrame.can_id = 111;
	canWrite(ch);

	poll(ch->ufds, 1, WAIT_MS);
	if (ch->ufds[0].revents & POLLIN)
	{
		readCan(ch);
//		if (ch->inOutCanFrame.can_id == 111)
//		{
//			printf("the same returned 111\n");
//		}
//		else if (ch->inOutCanFrame.can_id == 112)
//		{
//			printf("112\n");
//		}
//		if (ch->inOutCanFrame.can_id == 111)
//		{
//			printf("the same returned 111\n");
//		}
//		printf("pollin\n");
		return 0;
	}

	printf("No response on a sent frame before timeout!\n");
	return -1;
}

void sendSeries(CanHandler* ch, int32_t it_cnt)
{
//	size_t i;

	for (; it_cnt > 1; it_cnt--)
	{
		canWrite(ch);
	}
	ch->inOutCanFrame.can_id = 2;
	printf("can id: %d\n", ch->inOutCanFrame.can_id);
	canWrite(ch);
}

uint32_t calcExecTime(CanHandler* ch,
					  ssize_t (*fn)(CanHandler*),
					  int32_t it_cnt)
{
	int32_t i;
	struct timespec timeStampOld, timeStampNew;
	uint64_t acc_time = 0;

	if (fn(ch) == -1) //cache warm up
	{
		printf("\nTime calculation failed!\n");
		return 0;
	}
//	usleep(time_itv_ms * 1000);
	// maybe better run all iteration and after loop divide total time by iteration number
	for (i = 0; i < it_cnt; i++)
	{
		clock_gettime(CLOCK_MONOTONIC, &timeStampOld);
		if (fn(ch) == -1)
		{
			printf("\nTime calculation failed!\n");
			return 0; // alternatively return negative number and give output by argument pointer
		}
		clock_gettime(CLOCK_MONOTONIC, &timeStampNew);
		//optionally add dividing after couple iterations to avoid overflow
		acc_time += execTime_count(&timeStampOld, &timeStampNew);
//		usleep(time_itv_ms * 1000);
//		sleep(1);
//		printf("%lu: %lld\n", i, execTime_count(&timeStampOld, &timeStampNew));
	}

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













