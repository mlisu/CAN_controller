
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

	/*
	 * To run program uncomment one of the following:
	 * - sendNReceiveTime(&ch, 10000);
	 * - sendPeriodically(&ch);
	 * - controlSuspension(&ch);
	 * - controlRiddle(&ch);
	 * while keeping the others commented out
	 */

	// perform task:
		// Task 1 - time measure.
	sendNReceiveTime(&ch, 10000);

		// Task 2 - send periodically for capacity measurement
	// after running, input frequency as positive integral number <= 10tys
//	sendPeriodically(&ch);

		/* Task 3 - control - run Phy first */
//	controlSuspension(&ch);
//	controlRiddle(&ch);

	// close program:
	closeCanHandler(&ch);

	printf("Program properly closed\n");
	
	return 0;
	
}





