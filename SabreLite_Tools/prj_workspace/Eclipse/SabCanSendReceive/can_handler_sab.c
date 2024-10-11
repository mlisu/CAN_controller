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

int32_t readInt32(CanHandler* ch)
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

//void numberFromBytes(CanHandler* ch, char n, unsigned char* number)
//{
//	// zamiast n mogłoby być nic i używać ch->dlc ale jak jest n to można
//	// podawać argument jako siezeof(var) - co jest bezpieczniejsze
//	int i;
//
//	for (i = 0; i < n; i++)
//	{
//		number[i] = ch->inOutCanFrame.data[i];
//	}
//}

int readSeries(CanHandler* ch, int32_t left2Receive)
{
	int32_t i;

	for (i = left2Receive; i > 0; i--)
	{
		poll(ch->ufds, 1, WAIT_MS); // can removed only readCan suffices
		if (ch->ufds[0].revents & POLLIN)
		{
			readCan(ch);
			left2Receive--;
		}
	}
	// What is the below for? - it was when above was left2Receive > 1 (not > 0)
//	poll(ch->ufds, 1, WAIT_MS);
//	if (ch->ufds[0].revents & POLLIN)
//	{
//		readCan(ch);
//		left2Receive--;
//	}
	printf("Last frame id: %d\n", ch->inOutCanFrame.can_id);

	if (left2Receive)
	{
		printf("Missed %u frame(s) from series!\n", left2Receive);
		return -1;
	}

	return 0;
}

ssize_t readNSend(CanHandler* ch)
{
	poll(ch->ufds, 1, WAIT_MS);
	if (ch->ufds[0].revents & POLLIN)
	{
		readCan(ch);
//		if (ch->inOutCanFrame.can_id == 112)
//		{
//			printf("the same returned 112\n");
//		}
//		ch->inOutCanFrame.can_id = 112;
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








