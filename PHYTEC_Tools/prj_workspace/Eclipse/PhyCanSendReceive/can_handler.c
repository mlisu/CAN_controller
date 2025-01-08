#include "can_handler.h"

#include <net/if.h>
#include <stdio.h>
#include <stdlib.h> //exit
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

int readInt(CanHandler* ch)
{
	readCan(ch);
	return *(int*)ch->inOutCanFrame.data;
}

double readDouble(CanHandler* ch)
{
	readCan(ch);
	return *(double*)ch->inOutCanFrame.data;
}

void sendInt(CanHandler* ch, int data_in)
{
	ch->inOutCanFrame.can_dlc = sizeof(int);
	*(int*)ch->inOutCanFrame.data = data_in;
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
	canWrite(ch);

	poll(ch->ufds, 1, WAIT_MS);
	if (ch->ufds[0].revents & POLLIN)
	{
		readCan(ch);
		return 0;
	}

	printf("No response on a sent frame before timeout!\n");
	exit(1);
}
