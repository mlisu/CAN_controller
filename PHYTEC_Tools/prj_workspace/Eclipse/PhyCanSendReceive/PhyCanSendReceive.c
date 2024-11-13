
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
//	int const it_cnt = 1000;
//	sendNReceiveTime(&ch, it_cnt);

		// Task 2 - capacity measure - run Sab first
//	int const it_cnt = 200;
//	sendSeries4CapacityMeasurement(&ch, it_cnt);

		// Task 2b - send periodically
//	sendPeriodically(&ch);

		/* Task 3 - control inertia - run Phy first */
	controlInertia(&ch);

	// close program:
	closeCanHandler(&ch);

	printf("Program properly closed\n");
	
	return 0;
	
}





