
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
		/* Task 1 - time measure. Run program first on Sabre then Phy,
	because sabre calls blocking read CAN first */
//	echo4sendNReceiveTime(&ch);

		/* Task 2 - capacity mearue. Run first on Sabre */
//	readSeries4CapacityMeasurement(&ch);
	/*
	 * timer odczytywany przez poll przez np 10s z daną częstotliwością
	 * liczyć przez te 10 s czy w danej sekundzie dostałem all które wysłąłem
	 */
	/*
	 * może się kolejność zmienić o kilka czy ileś tam (można oknem wysyłąć)
	 */

		/* Task 2b Read periodically - run first on Sab*/
//	readPeriodically(&ch);

		/* Task 3 - inertia simulation - run Phy first */
//	runSimulation(&ch);
	runRiddleSimulation(&ch);

	// close program:
	closeCanHandler(&ch);

	printf("Program properly closed\n");

	return 0;

}





