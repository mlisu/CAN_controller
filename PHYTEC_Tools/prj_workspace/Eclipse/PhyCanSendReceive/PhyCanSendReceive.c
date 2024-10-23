
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
		// Task 1 - time measure.
//	int const it_cnt = 20000000;
//	int const it_cnt = 0x01020304;
//	int const it_cnt = 0xfffe;
//	int const it_cnt0 = 100000;		// hundret thousends
//	int const it_cnt1 = 1000000; 	// million
//	int const it_cnt2 = 10000000; 	// 10 millions
//	int const it_cnt3 = 100000000; 	// 100 millions
//	int const it_cnt4 = 1000000000; // billion
//	int const it_cnt5 = 10000; 		// 10 thousends
//	int const it_cnt6 = 5; 		// 10 thousends
//
//	int const it_cnt = it_cnt6;
//	sendNReceiveTime(&ch, it_cnt);

		// Task 2 - capacity measure - run Sab first
//	int const it_cnt = 200;
//	sendSeries4CapacityMeasurement(&ch, it_cnt);

		// Task 2b - send periodically
	sendPeriodically(&ch);

		/* Task 3 - control inertia - run Phy first */
//	controlInertia(&ch);

	// close program:
	closeCanHandler(&ch);

	printf("Program properly closed\n");
	
	return 0;
	
}





