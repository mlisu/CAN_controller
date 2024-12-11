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

int riddleModel(double t, const double x[], double dxdt[], void* params)
{
	// siła
	// 19
	/*
	 * siła Fex = sin
	 */
	Fex = Fe*sin(2*pi*19*t);
	Fez = Fe*cos...
	Fe = me*(2*pi*19)^2*re; <--- siłą odśrodkowa
	re = 0.007;//[m];
	// Siłą F odpowiada sile odsrodowej

	double const xs  = x[0];
	double const zs  = x[1];
	double const ps  = x[2];
	double const xsp = x[3];
	double const zsp = x[4];
	double const psp = x[5];

	double const xsf = xs + LSF*cos(ps) + HSF * sin(ps);
	double const xsr = xs - LSR*cos(ps) + HSR * sin(ps);
	double const zsf = zs + HSF*cos(ps) - LSF * sin(ps);
	double const zsr = zs + HSR*cos(ps) + LSR * sin(ps);

	double const zsfp = zsp - psp * (HSF*sin(ps) + LSF*cos(ps)); //zsp - HSF*sin(ps)*psp - LSF*cos(ps)*psp;
	double const zsrp = zsp - psp * (HSR*sin(ps) - LSR*cos(ps)); //zsp - HSR*sin(ps)*psp + LSR*cos(ps)*psp;
	// kxf == kxr; kzf == kzr // grawitacja to siądzie ale nie zakłądać

	double const Fkfx = -KX*(xsf - XSF0);
	double const Fkrx = -KX*(xsr - XSR0);
	double const Fkfz = -KZ*(zsf - ZSF0);
	double const Fkrz = -KZ*(zsr - ZSR0);

	double const Fcxs = -CX*xsp;
	double const Fczs = -CZ*zsp;
	double const Mcps = -CPS*psp;

	double const cf = ((double*)params)[RIN_IDXF]; // from controller for front
	double const cr = ((double*)params)[RIN_IDXR]; // for rear
	double const cff = cf*zsfp;	// controlled force front
	double const cfr = cr*zsrp; // rear

	double const Fsx = Fkfx + Fkrx + Fcxs;
	double const Fsz = Fkfz + Fkrz + Fczs + cff + cfr;
	double const Ms	 = Mcps + Fkfx * (zsf - zs) + Fkrx * (zsr - zs)
					   -(Fkfz + cff)*(xsf - xs) - (Fkrz + cfr)*(xsr - xs);

	Isphis = Ms;
	msxspp = Fex + Fsx;
	mszspp = Fez + Fsz;

	dxdt[0] = xsp;			// xs'
	dxdt[1] = zsp;			// zs'
	dxdt[2] = psp;			// ps'
	dxdt[3] =

	/*
	 * Na początek można uprościć że cr == cf. Ale potem lepiej różne jak mam czas
	 * cmin = 0; cmax = 1000;Ns/m
	 * steruje cr == cf albo oddzilnie; Można inty przesyłać
	 */
	// CAN ma 1 Mb/s; w wersj Fd to 5 Mb/s (Phytec) - 64 bajty ramki

	/*
	 * Z pomiarów dostaje przyspieszenie - z tego wyliczam RMS i dla zadanego RMS
	 * staruje. Jak RMS > zadane RMS -> zwięskza C, i odwrotnie
	 * regulator PI ale samo I też może działać. Nastawy prób i błędów albo odpowiedź
	 * skokowa. Średnia krocząca / filtry inercyjny
	 * Mamy stałą f = 19Hz, można policzyć ile próbek: 100/19 = 5.26 próbki na okres.
	 * RMS najlepiej liczyć z całego okresu - może być dla 4 okresóœ = 21,05
	 * Dla kolejnych wywłań regulatora biore ostatnie 21 próbek - najstarszą wyrzucam
	 * biore aktualną. I uśredniam sposobem średniej kroczącej. Próbka to kwadrate wartości
	 * przyspieszenia pionowe z przodu i z tyłu. Wysyłąm 2 ramki - 1 azf, druga azr.
	 * Na postawie tych a liczymy średnią i to jest as (przyspieszenie środka)
	 * as (oszacowanie as- oznaczyć żę to oszacowanie) podnosze do kwadratu sqrt((suma as od 1 do 21) / 21)
	 *
	 * Średnia krocząca dodaje wartości w buforze (21 wartości) i dzili przez 21.
	 * Żeby obliczyć RMS to na początku w buforze daje kwadrat as. Wyjście średniej kroczącej pierwiastkuje.
	 * Dodaje kwadraty asów do bufora je dziele przez 21. RMS = sqrt(średnia krocząca)
	 * 30m/s2; RMS = 20 m/s2 (19) np.
	 * Narysować schemat sterowania
	 */
	/*
	 * samym cz sterować; jak będzie czas
	 */
	sim(1ms)
	poll
	sim(1ms)
	poll
	(sim(1ms))


	sim(10ms)
	send state
	sim(1ms)
	/*
	 * Dokładnie co okres próbkowania odbierać sterowania. Wysyłąmy w tej samej chwli stan co odbieramy sterowanie.
	 */
	// Czyli zrobić że zawsze jest 1 próbka opóźnienia.


	/*
	 *
	 */

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

void deleteSim(Simulation* const sim)
{
	free(sim->data_vec);
	free(sim->u_vec);
	free(sim->t_vec);
}






