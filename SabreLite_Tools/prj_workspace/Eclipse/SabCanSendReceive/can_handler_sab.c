#include "can_handler_sab.h"

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
	ch->ufds[CAN_IDX].fd = ch->canSocket;
	ch->ufds[CAN_IDX].events = POLLIN;

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

ssize_t readCan(CanHandler* ch)
{
	return read(ch->canSocket, &ch->inOutCanFrame, sizeof(struct can_frame));
}

int readSeries(CanHandler* ch, int32_t cnt)
{
	int32_t i;
	int32_t left2receive = cnt;

	for (i = 0; i < cnt; i++)
	{
		poll(ch->ufds, 1, WAIT_MS); // can removed only readCan suffices?
		if (ch->ufds[CAN_IDX].revents & POLLIN)
		{
			readCan(ch);
			printf("%d Frame id: %d\n", i, ch->inOutCanFrame.can_id);
			if (ch->inOutCanFrame.can_id != i)
			{
				printf("Wrong frames order!\n");
				return -1;
			}
			left2receive--;
		}
	}

	if (left2receive)
	{
		printf("Missed %u frame(s) from series!\n", left2receive);
		return -1;
	}

	return 0;
}

ssize_t readNSend(CanHandler* ch)
{
	poll(ch->ufds, 1, WAIT_MS);
	if (ch->ufds[CAN_IDX].revents & POLLIN)
	{
		readCan(ch);
		canWrite(ch);
		return 0;
	}
	printf("No incoming frame before timeout\n");
	return -1;
}

void sendInt32(CanHandler* ch, int32_t data_in)
{
	ch->inOutCanFrame.can_dlc = 4;
	*(int32_t*)ch->inOutCanFrame.data = data_in;
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






