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
#define WAIT_MS				300
#define FRAMES_BUF_LEN		110000 // 110 kB

// file descriptors indices:
#define TIMER_IDX	0
#define CAN_IDX		1
#define IO_IDX		2

typedef struct CanHandler_
{
	struct pollfd ufds[3];
	int canSocket;
	struct can_frame inOutCanFrame;
} CanHandler;

int initCanHandler(CanHandler* ch);
void closeCanHandler(CanHandler* ch);
int canConfig();

int32_t readInt32(CanHandler* ch);
double readDouble(CanHandler* ch);
ssize_t readCan(CanHandler* ch);
int readSeries(CanHandler* ch, int32_t cnt);
ssize_t readNSend(CanHandler* ch);

void sendInt32(CanHandler* ch, int32_t data_in);
void sendDouble(CanHandler* ch, double data_in);
ssize_t canWrite(CanHandler* ch);

#endif /* CAN_HANDLER_SAB_H_ */
