#include "simulation.h"

#include <math.h>
//#include <stdlib.h>

static void initFileHandler(FileHandler* const fh)
{
	fh->f = fopen(OUT_FILE_NAME, "w");
	if (fh->f == NULL)
	{
		printf("Could not open a file, exiting\n");
		exit(1);
	}
}

void simDataToFile(Simulation* const sim)
{
        int i;
        int it = 0;
        for (i = 0; i < SIM_DATA_VEC_LEN; i++)
        {
                it += sprintf(sim->fh.buf + it, "%.3f\n", sim->data_vec[i]);
        }
        fprintf(sim->fh.f, "%s", sim->fh.buf);
        fclose(sim->fh.f);
}

int inertiaModel(double t, const double x[], double dxdt[], void* params)
{
	double input = *(double*)params;
	dxdt[0] = (KS*input -x[0])/TS;

	return GSL_SUCCESS;
}

int suspensionModel(double t, const double x[], double dxdt[], void* params)
{
	// M1 and z1 are for the mass above
	double const u   = 0.1*sin(SIN_FREQ * t);
	double const up  = 0.1*cos(SIN_FREQ * t);
	double const z1  = x[0];
	double const z2  = x[1];
	double const z1p = x[2];
	double const z2p = x[3];

	double F = *(double*)params;

	dxdt[0] = z1p;                                      // z1'
	dxdt[1] = z2p;                                      // z2'
	dxdt[2] = (K1*(z2 - z1) + C1*(z2p - z1p) + F) / M1; // z1"
	dxdt[3] = (K2*(u  - z2) + C2*(up  - z2p)			// z2"
			 - K1*(z2 - z1) - C1*(z2p - z1p) - F) / M2;

	return GSL_SUCCESS;
}

void initSim(Simulation* const sim,
		 	int (*model)(double, const double[], double[], void*),
			int const dimension,
			double const dt,
			double* const params)
{
	gsl_odeiv2_system sys = {model, NULL, dimension, params /*sim->params*/};
	int i;

	sim->sys    = sys;
	sim->t	    = 0;
	sim->t_end  = dt;
	sim->dt	    = dt;
	sim->d 	    = gsl_odeiv2_driver_alloc_y_new(&sim->sys,
												gsl_odeiv2_step_rkf45,
												1e-6, 1e-3, 1e-3);

	for (i = 0; i < X_LEN; i++)
	{
		sim->x[i] = 0.0;
	}

	initFileHandler(&sim->fh);
	sim->data_vec[0] = INIT_STATE;
}

int runSim(Simulation* const sim)
{
    static size_t i = 1;

    int status = gsl_odeiv2_driver_apply(sim->d, &sim->t, sim->t_end, sim->x);
    if (status != GSL_SUCCESS)
    {
        printf("Failed to solve ode, error: %d\n", status);
    }

    sim->t_end         += sim->dt;
    sim->data_vec[i]  = sim->x[OUT_IDX];
    i++;

    return status;
}

