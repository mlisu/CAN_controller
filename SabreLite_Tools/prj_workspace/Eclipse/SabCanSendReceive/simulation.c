#include "simulation.h"

#include <assert.h>
#include <math.h>

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
        for (i = 0; i <= sim->cnt; i++) // <= ---> see cnt incrementation in runSim here
        {
                fprintf(sim->fh.f, "%.4f;" , sim->t_vec[i]);
                fprintf(sim->fh.f, "%.4f;" , sim->data_vec[i]);
                fprintf(sim->fh.f, "%.4f\n", sim->u_vec[i]);
        }
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
	double const uw  = 2*M_PI * ((double*)params)[UF_IDX];
	double const u   = 0.1*sin(uw * t); // change 0.1 to A macro
	double const up  = 0.1*SIN_W*cos(uw * t);
	double const z1  = x[0];
	double const z2  = x[1];
	double const z1p = x[2];
	double const z2p = x[3];

	double const F = ((double*)params)[IN_IDX];

	dxdt[0] = z1p;                                      // z1'
	dxdt[1] = z2p;                                      // z2'
	dxdt[2] = (K1*(z2 - z1) + C1*(z2p - z1p) + F) / M1; // z1"
	dxdt[3] = (K2*(u  - z2) + C2*(up  - z2p)			// z2"
			 - K1*(z2 - z1) - C1*(z2p - z1p) - F) / M2;

	((double*)params)[U_IDX] = u; // is U right name for disturbance?

	return GSL_SUCCESS;
}

void initSim(Simulation* const sim,
		 	int (*model)(double, const double[], double[], void*),
			int const dimension,
			double const dt,
			double* const params)
{
	assert(OUT_IDX < X_LEN);
	assert(U_IDX < PARAM_LEN);
	assert(IN_IDX < PARAM_LEN);

	gsl_odeiv2_system sys = {model, NULL, dimension, params /*sim->params*/};
	int i;

	sim->cnt    = 0;
	sim->sys    = sys;
	sim->t	    = 0;
	sim->t_end  = dt;
	sim->dt	    = dt;
	sim->params = params;
	sim->d 	    = gsl_odeiv2_driver_alloc_y_new(&sim->sys,
												gsl_odeiv2_step_rkf45,
												1e-6, 1e-3, 1e-3);

	sim->data_vec = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	sim->u_vec = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	sim->t_vec = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	if (sim->data_vec == NULL || sim->u_vec == NULL || sim->t_vec == NULL)
	{
		printf("initSim failed to allocate memory.\n");
		exit(1);
	}

	for (i = 0; i < dimension; i++)
	{
		sim->x[i] = 0.0;
	}

	initFileHandler(&sim->fh);
	sim->data_vec[0] = INIT_STATE;
}

int runSim(Simulation* const sim)
{
    sim->cnt++;

    if (sim->cnt >= SIM_DATA_VEC_LEN_MAX)
    {
    	printf("Sim data buffer exceeded.\n");
    	exit(1);
    }

    int status = gsl_odeiv2_driver_apply(sim->d, &sim->t, sim->t_end, sim->x);
    if (status != GSL_SUCCESS)
    {
        printf("Failed to solve ode, error: %d\n", status);
    }

    sim->t_end       	   += sim->dt;
    sim->data_vec[sim->cnt] = (float)sim->x[OUT_IDX];
    sim->u_vec[sim->cnt]	= (float)sim->params[U_IDX];
    sim->t_vec[sim->cnt]	= (float)sim->t;

    return status;
}

