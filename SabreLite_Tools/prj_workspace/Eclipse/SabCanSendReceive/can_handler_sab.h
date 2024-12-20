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
#define FRAMES_BUF_LEN		110000 // 100 kB
//#define STD_IN_CHARS_CNT	10

typedef struct CanHandler_
{
	struct pollfd ufds[3]; //3 bo jeszcze dla timera i stdio

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
double readDouble(CanHandler* ch);
ssize_t readCan(CanHandler* ch);
int readSeries(CanHandler* ch, int32_t cnt);
ssize_t readNSend(CanHandler* ch);

void sendInt32(CanHandler* ch, int32_t data_in);
void sendDouble(CanHandler* ch, double data_in);
ssize_t canWrite(CanHandler* ch);

#endif /* CAN_HANDLER_SAB_H_ */
