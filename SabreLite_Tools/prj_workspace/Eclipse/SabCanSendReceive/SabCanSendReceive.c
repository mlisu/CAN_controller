
#include <stdio.h>

#include "can_handler_sab.h"
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
//	echo4sendNReceiveTime(&ch);
	readSeries4CapacityMeasurement(&ch);

	// close program:
	closeCanHandler(&ch);

	printf("Program properly closed\n");

	return 0;

}





