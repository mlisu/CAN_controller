#include "simulation.h"

#include <math.h>
#include <stdlib.h>

void initFileHandler(FileHandler* const fh)
{
	fh->f = fopen(OUT_FILE_NAME, "w");
	if (fh->f == NULL)
	{
		printf("Could not open a file, exiting\n");
		exit(1);
	}
//	memset(fh->data_vec, 0, sizeof(fh->data_vec));
	fh->data_vec[0] = INIT_STATE;
}

void simDataToFile(FileHandler* const fh)
{
        int i;
        int it = 0;
        for (i = 0; i < SIM_DATA_VEC_LEN; i++)
        {
                it += sprintf(fh->buf + it, "%.3f\n", fh->data_vec[i]);
        }
        fprintf(fh->f, "%s", fh->buf);
        fclose(fh->f);
}

double inertiaOutput(double input)
{
	/* Move this comment elsewhere
	First inertia should be called because there could be initial state != 0
	and controller should not make assumption that the state is 0
	*/
	static double const alf = exp(-T/TS);
	static double out = 0;

	out = KS*input*(1-alf) + alf*out; // possibly implement with initial condition

	return out;
}

void computOutputBetweenCtrlSignals(FileHandler* fh, int* idx, double ctrl_signal)
{
	int i;
	for (i = 0; i < CTR_SYS_RATIO; i++)
	{
		fh->data_vec[*idx] = inertiaOutput(ctrl_signal);
		(*idx)++;
	}
}





