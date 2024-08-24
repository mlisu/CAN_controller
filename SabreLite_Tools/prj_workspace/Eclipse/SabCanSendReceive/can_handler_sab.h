#ifndef CAN_HANDLER_SAB_H_
#define CAN_HANDLER_SAB_H_

#include <linux/can.h>
#include <stddef.h> //size_t
#include <stdint.h> //byte type (e.g. int8_t)
#include <sys/poll.h>
#include <time.h>
#include <unistd.h> //write & read

#define NANO_IN_SEC			1000000000
#define CANBUS				"can0"
#define WAIT_MS				100
//#define STD_IN_CHARS_CNT	10

typedef struct CanHandler_
{
	struct pollfd ufds[2]; //2 bo jeszcze dla stdio

	int canSocket;

	struct can_frame inOutCanFrame;

	// std input - to quit program
//	char stdinBuf[STD_IN_CHARS_CNT];
//	char tempChar;
} CanHandler;

int initCanHandler(CanHandler* ch);

void closeCanHandler(CanHandler* ch);

int canConfig();

int32_t readInt32(CanHandler* ch);

ssize_t readCan(CanHandler* ch);

int readSeries(CanHandler* ch, int32_t left2Receive);

ssize_t readNSend(CanHandler* ch);

ssize_t canWrite(CanHandler* ch);

#endif /* CAN_HANDLER_SAB_H_ */
