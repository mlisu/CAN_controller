
#include <stdio.h>

#include "can_handler.h"
#include "tasks_functions.h"


int main(int argc, char *argv[])
{
	// prepare:
	CanHandler ch;
	if (initCanHandler(&ch) == -1)
	{
		printf("Failed to initialize CanHandler! Aborting!\n");
		return -1;
	}

	// perform task:
//	int const it_cnt = 20000000;
//	int const it_cnt = 0x01020304;
//	int const it_cnt = 0xfffe;
	int const it_cnt0 = 100000;  // hundret thousends
	int const it_cnt1 = 1000000; // million
	int const it_cnt2 = 10000000; // 10 millions
	int const it_cnt3 = 100000000; // 100 millions
	int const it_cnt4 = 1000000000; // billion

	int const it_cnt = 5;
//	sendNReceiveTime(&ch, it_cnt);
	sendSeries4CapacityMeasurement(&ch, it_cnt);

	// close program:
	closeCanHandler(&ch);
	
	printf("Program properly closed\n");
	
	return 0;
	
}





