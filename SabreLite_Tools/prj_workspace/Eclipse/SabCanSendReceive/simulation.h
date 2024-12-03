#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// simulation parameters:
#define SIM_DATA_VEC_LEN_MAX	50000
#define OUT_FILE_NAME			"system_response.csv"

#define X_LEN					4 		// vector of state variables length for inertia; 1 for inertia, 4 for suspension
#define PARAM_LEN				3		// parameter vector length
#define OUT_IDX					0		// index of the observed state variable in the state vector (system output)
#define U_IDX					2		// index of disturbance value in params
#define UF_IDX					1		// index of disturbance freq in params
#define IN_IDX					0		// index of system input in params
#define SIM_STEP				0.010 	// 10 ms
#define SIN_W					3.14/4 	// rad/s - sinus disturbance angular frequency
#define TR_T					1		// s - transient time after disturbance f change
#define FIRST_F					2		// Hz - starting disturbance f
#define LAST_F					3		// Hz
#define F_STEP					0.1		// Hz

// inertia parameters:
#define TS         5		// s; inertia system time constant
#define KS         1		// gain of the inertia system
#define T          0.030	// s, inertia state actualization period - 30 ms
#define TSEN	   300		// ms, inertia state measurement period
#define INIT_STATE 0		// initial state of the system output

// suspension parameters:
#define M1 290		// kg		// alternative: 300
#define M2 59		// kg		// 20
#define K1 16812	// N/m		// 15000
#define K2 190000	// N/m		// 200000
#define C1 1000		// N/(m/s)	// 1000
#define C2 2500		// N/(m/s)	// 2500

typedef struct FileHandler_ // move f to Simylation, buf is unnecessary
{
	FILE* f;
//	char buf[FILE_BUF_LEN];

} FileHandler;

typedef struct Simulation_
{
	gsl_odeiv2_system sys;
	gsl_odeiv2_driver* d;

	int	   cnt;		 // counter of sim steps
	double t;		 // stores current simulation time
	double t_end;	 // end of simulation step
	double dt;		 // simulation step
	double x[X_LEN]; // vector of state variables
	double* params;	 // incoming parameters like disturbance

	float* data_vec;
	float* u_vec;	 // vector for disturbance data
	float* t_vec;	 // time vector - can be removed; it will be 0.1, 0.2, 0.3 ...
	FileHandler fh;

} Simulation;

void simDataToFile(Simulation* const sim);

int inertiaModel(double t, const double y[], double dxdt[], void* params);
int suspensionModel(double t, const double y[], double dxdt[], void* params);

void initSim(Simulation* const sim,
		    int (*model)(double, const double[], double[], void*),
		    int const dimension,
			double const dt,
		    double* const params);

int runSim(Simulation* const sim);

#endif /* SIMULATION_H_ */
