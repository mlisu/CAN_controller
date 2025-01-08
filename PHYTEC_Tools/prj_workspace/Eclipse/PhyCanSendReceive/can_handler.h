#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include <linux/can.h>
#include <stddef.h> //size_t
#include <sys/poll.h>
#include <time.h>
#include <unistd.h> //write & read

#define NANO_IN_SEC			1000000000
#define CANBUS				"can0"
#define WAIT_MS				300

typedef struct
{
	struct pollfd ufds[3];

	int canSocket;

	struct can_frame inOutCanFrame;

} CanHandler;

int initCanHandler(CanHandler* ch);
void closeCanHandler(CanHandler* ch);
int canConfig();

ssize_t readCan(CanHandler* ch);
int readInt(CanHandler* ch);
double readDouble(CanHandler* ch);

void sendInt(CanHandler* ch, int data_in);
void send2ints(CanHandler* ch, int first, int second);
void sendDouble(CanHandler* ch, double data_in);
ssize_t canWrite(CanHandler* ch);
ssize_t sendNReceive(CanHandler* ch);

#endif /* CAN_HANDLER_H_ */
