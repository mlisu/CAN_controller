#include "simulation.h"

#include <assert.h>
#include <math.h>

static void initFileHandler(FILE** f)
{
	*f = fopen(OUT_FILE_NAME, "w");
	if (*f == NULL)
	{
		printf("Could not open a file, exiting\n");
		exit(1);
	}
}

void simDataToFile(Simulation* const sim)
{
	int i;
	for (i = 0; i <= sim->cnt; i++)
	{
		fprintf(sim->f, "%.4f;" , sim->t_vec[i]);
		fprintf(sim->f, "%.4f;" , sim->data_vec1[i]);
		fprintf(sim->f, "%.4f;", sim->data_vec2[i]);
		fprintf(sim->f, "%.4f\n", sim->data_vec3[i]);
	}
	fclose(sim->f);
}

int suspensionModel(double t, const double x[], double dxdt[], void* params)
{
	// M1 and z1 are for the mass at the top
	double const uw  = 2*M_PI * ((Params*)params)->data_dbl[UF_IDX];
	double const u   = 0.1*sin(uw * t);
	double const up  = 0.1*SIN_W*cos(uw * t);
	double const z1  = x[0];
	double const z2  = x[1];
	double const z1p = x[2];
	double const z2p = x[3];

	double const F = ((Params*)params)->data_dbl[IN_IDX];

	dxdt[0] = z1p;                                      // z1'
	dxdt[1] = z2p;                                      // z2'
	dxdt[2] = (K1*(z2 - z1) + C1*(z2p - z1p) + F) / M1; // z1"
	dxdt[3] = (K2*(u  - z2) + C2*(up  - z2p)			// z2"
			 - K1*(z2 - z1) - C1*(z2p - z1p) - F) / M2;

	((Params*)params)->data_dbl[OUT_IDX] = z1;
	((Params*)params)->data_dbl[U_IDX] = u;

	return GSL_SUCCESS;
}

int riddleModel(double t, const double x[], double dxdt[], void* params)
{
	double const xs  = x[0];
	double const zs  = x[1];
	double const ps  = x[2];
	double const xsp = x[3];
	double const zsp = x[4];
	double const psp = x[5];

	double const sinps = sin(ps);
	double const cosps = cos(ps);

	double const xsf = xs + LSF*cosps + HSF * sinps;
	double const xsr = xs - LSR*cosps + HSR * sinps;
	double const zsf = zs + HSF*cosps - LSF * sinps;
	double const zsr = zs + HSR*cosps + LSR * sinps;

	double const zsfp = zsp - psp * (HSF*sinps + LSF*cosps);
	double const zsrp = zsp - psp * (HSR*sinps - LSR*cosps);

	double const Fkfx = -KX*(xsf - XSF0);
	double const Fkrx = -KX*(xsr - XSR0);
	double const Fkfz = -KZ*(zsf - ZSF0);
	double const Fkrz = -KZ*(zsr - ZSR0);

	double const Fcxs = -CX*xsp;
	double const Fczs = -CZ*zsp;
	double const Mcps = -CPS*psp;

	int const cf = ((Params*)params)->data_int[0]; // from controller for front
	int const cr = ((Params*)params)->data_int[1]; // for rear
	double const CFczf = -cf*zsfp; // controlled force front
	double const CFczr = -cr*zsrp; // rear

	double const Fsx = Fkfx + Fkrx + Fcxs;
	double const Fsz = Fkfz + Fkrz + Fczs + CFczf + CFczr;
	double const Ms	 = Mcps + Fkfx * (zsf - zs) + Fkrx * (zsr - zs)
					   -(Fkfz + CFczf)*(xsf - xs) - (Fkrz + CFczr)*(xsr - xs);

	double const we = ((Params*)params)->data_dbl[5];

	double const Fex = FE*sin(we*t);
	double const Fez = FE*cos(we*t); // angle is 0 when exciter is up

	double const ms = ((Params*)params)->data_dbl[3];
	double const is = ((Params*)params)->data_dbl[4];

	dxdt[0] = xsp;				// xs'
	dxdt[1] = zsp;				// zs'
	dxdt[2] = psp;				// ps'
	dxdt[3] = (Fex + Fsx) / ms;	// xs"
	dxdt[4] = (Fez + Fsz) / ms;	// zs"
	dxdt[5] = Ms / is;			// ps"

	// Accelerations to be sent to controller:
	((Params*)params)->data_dbl[0] = dxdt[4] - dxdt[5] * (HSF*sinps + LSF*cosps)
											 - psp*psp * (HSF*cosps - LSF*sinps);

	((Params*)params)->data_dbl[1] = dxdt[4] - dxdt[5] * (HSR*sinps - LSR*cosps)
											 - psp*psp * (HSR*cosps + LSR*sinps);

	// save control signal:
	((Params*)params)->data_dbl[2] = (double)((Params*)params)->data_int[0];

	return GSL_SUCCESS;
}

void initSim(Simulation* const sim,
		 	int (*model)(double, const double[], double[], void*),
			int const dimension,
			double const dt,
			Params* const params)
{
	gsl_odeiv2_system sys = {model, NULL, dimension, params};
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

	sim->data_vec1 = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	sim->data_vec2 = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	sim->data_vec3 = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	sim->t_vec = malloc(SIM_DATA_VEC_LEN_MAX * sizeof(float));
	if (sim->data_vec1 == NULL
		|| sim->data_vec2 == NULL
		|| sim->data_vec3 == NULL
		|| sim->t_vec == NULL)
	{
		printf("initSim failed to allocate memory.\n");
		exit(1);
	}

	for (i = 0; i < dimension; i++)
	{
		sim->x[i] = 0.0;
	}

	initFileHandler(&sim->f);
	sim->data_vec1[0] = 0.0;
	sim->data_vec2[0] = 0.0;
	sim->data_vec3[0] = 0.0;
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

    sim->t_end       	     += sim->dt;
    sim->data_vec1[sim->cnt] = (float)sim->params->data_dbl[0]; // for suspension 0 is z1 position (upper mass position) and 1 is disturbance
    sim->data_vec2[sim->cnt] = (float)sim->params->data_dbl[1];	// for riddle these are acc front and rear
    sim->data_vec3[sim->cnt] = (float)sim->params->data_dbl[2]; // control
    sim->t_vec[sim->cnt]	 = (float)sim->t;

    return status;
}

void deleteSim(Simulation* const sim)
{
	free(sim->data_vec1);
	free(sim->data_vec2);
	free(sim->data_vec3);
	free(sim->t_vec);
}






