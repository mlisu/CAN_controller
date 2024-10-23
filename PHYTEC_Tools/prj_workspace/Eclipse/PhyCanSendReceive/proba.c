// standard libraries

#include <sys/poll.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
//#include <linux/can/raw.h>
#include <time.h>

//-----------------------------

#define NANO_IN_SEC					1000000000	// in ns
#define CANBUS "can0"

//-----------------------------
// functions

int canConfig2();
int pollTimer_config2(struct pollfd * ufds);
int pollTimer_set2(const long long tValueNs, const long long tIntervalNs, struct pollfd * ufds);
long execTime_count2( struct timespec * timeStartPtr, struct timespec * timeStopPtr);

//-----------------------------------
// main
int Delmain(int argc, char *argv[])
{
	int i;
	int canSocket;
	int canConfigFlag = 1;
//*
	if(!(canSocket = canConfig2()))
	{
		printf("can config error\n");
		canConfigFlag = 0;
	}
//*/
	//---------------------

	struct pollfd ufds[3];
	memset(ufds, 0, sizeof(ufds));

	//-------------------------
	// timer
	pollTimer_config2(ufds);
	pollTimer_set2(5000000000, 5000000000, ufds);
	long long int expTmp;

	//-------------------------
	// std input
	ufds[1].fd = STDIN_FILENO;
	ufds[1].events = POLLIN;
	char stdinBuf[100];
	char tempChar;

	//------------------------
	// canbus
	ufds[2].fd = canSocket;
	ufds[2].events = POLLIN;

	struct can_frame inOutCanFrame;

	struct timespec timeStampNew, timeStampOld;
	long execTime;

	//--------------------

	clock_gettime(CLOCK_MONOTONIC, &timeStampOld);

	while(1)
	{
		printf("poczatek\n");
		if (canConfigFlag)
		{
			printf("if\n");
			poll(ufds, 3, -1);
			printf("tutaj\n");
		}
		else
		{
			poll(ufds, 2, -1);
			printf("tutaj2\n");
		}

		if (ufds[0].revents & POLLIN)
		{
			read(ufds[0].fd, &expTmp, sizeof(long long int));
			printf("timer event\n");
		}
		else if (ufds[1].revents & POLLIN)
		{
			scanf("%[^\n]", stdinBuf);
			scanf("%c", &tempChar);
			printf("read string: %s\n", stdinBuf);

			if (stdinBuf[0] == 'q')
			{
				break;
			}
			else
			{
				if ((inOutCanFrame.can_dlc = sscanf(stdinBuf, "%d %hhd %hhd %hhd %hhd %hhd %hhd %hhd %hhd", &(inOutCanFrame.can_id), \
					inOutCanFrame.data, inOutCanFrame.data+1, inOutCanFrame.data+2, inOutCanFrame.data+3, \
					inOutCanFrame.data+4, inOutCanFrame.data+5, inOutCanFrame.data+6, inOutCanFrame.data+7 )) != EOF)
				{
					inOutCanFrame.can_dlc --;
					write(canSocket, &inOutCanFrame, sizeof(inOutCanFrame));
					printf("outgoing can frame: %d ", inOutCanFrame.can_id);
					for (i=0; i < inOutCanFrame.can_dlc; i++)
					{
						printf("%hhd ", inOutCanFrame.data[i]);
					}
					printf("\n");
				}
			}

		}
//*
		else if (ufds[2].revents & POLLIN)
		{
			// measure of time between incoming can frames
			clock_gettime(CLOCK_MONOTONIC, &timeStampNew);

			execTime = execTime_count2( &timeStampOld, &timeStampNew);
			timeStampOld.tv_sec = timeStampNew.tv_sec;
			timeStampOld.tv_nsec = timeStampNew.tv_nsec;

			// process incoming can frame
			read(canSocket, &inOutCanFrame, sizeof(struct can_frame));
			printf("incoming can frame: %d ", inOutCanFrame.can_id);
			for (i = 0; i <  inOutCanFrame.can_dlc; i++)
			{
				printf("%d ", inOutCanFrame.data[i]);
			}
			printf("\n");
			printf("time from last can incoming frame: %ld ns\n", execTime);
		}
//*/
	}

	close(canSocket);

	printf("Program properly closed\n");

	return 0;

}

//---------------
// functions

int canConfig2()
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

	// disable default receive filter on this RAW socket
	// This is obsolete as we do not read from the socket at all, but for
	// this reason we can remove the receive list in the Kernel to save a
	// little (really a very little!) CPU usage.
	// setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(l_canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		printf("bind error\n");
		return 0;
	}

	return l_canSocket;
}

int pollTimer_config2(struct pollfd * ufds)
{
	int tfd;

	if ((tfd = timerfd_create (CLOCK_MONOTONIC, TFD_NONBLOCK )) < 0)
		perror("timerfd create error");


	ufds[0].fd = tfd;
	ufds[0].events = POLLIN;

	return 0;
}

int pollTimer_set2(const long long tValueNs, const long long tIntervalNs, struct pollfd * ufds)
{
	struct itimerspec newValue;
	long tValue_sec, tValue_nsec, tInterval_sec, tInterval_nsec;

	tValue_sec = tValueNs/NANO_IN_SEC;
	tValue_nsec = tValueNs - tValue_sec * NANO_IN_SEC;
	newValue.it_value.tv_sec=tValue_sec;
	newValue.it_value.tv_nsec=tValue_nsec;

	tInterval_sec = tIntervalNs/NANO_IN_SEC;
	tInterval_nsec = tIntervalNs - tInterval_sec * NANO_IN_SEC;
	newValue.it_interval.tv_sec=tInterval_sec;
	newValue.it_interval.tv_nsec=tInterval_nsec;

	if( timerfd_settime(ufds[0].fd,0,&newValue,NULL) <0)
	{
		perror("settime error");
	}

	return 0;
}

long execTime_count2( struct timespec * timeStartPtr, struct timespec * timeStopPtr)
{
	long cpt_ns, cpt_s;

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
