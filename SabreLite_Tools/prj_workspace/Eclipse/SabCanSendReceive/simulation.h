#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// common simulation parameters:
//#define SIM_DATA_VEC_LEN_MAX	50000
#define SIM_DATA_VEC_LEN_MAX	500000
#define OUT_FILE_NAME			"system_response.csv"
#define X_LEN					4 		// vector of state variables length 4 for suspension, 6 for riddle
#define SIM_STEP				0.010 	// 10 ms
//#define SIM_STEP				0.005 	// 5 ms
//#define SIM_STEP				0.003 	// 3 ms
//#define SIM_STEP				0.001 	// 1 ms
#define INIT_STATE 				0		// initial state of the system output

// suspension sim params
#define OUT_IDX					0		 // index of system output in params
#define U_IDX					1		 // index of disturbance value in params
#define IN_IDX					2		 // index of system input in params
#define UF_IDX					3		 // index of disturbance freq in params

#define SIN_W					3.1416/4 // rad/s - sinus disturbance angular frequency
#define TR_T					1		 // s - transient time after disturbance f change
#define FIRST_F					2		 // Hz - starting disturbance f
#define LAST_F					2.2		 // Hz
#define F_STEP					0.1		 // Hz

// suspension parameters:
#define M1 290		// kg		// alternative: 300
#define M2 59		// kg		// 20
#define K1 16812	// N/m		// 15000
#define K2 190000	// N/m		// 200000
#define C1 1000		// N/(m/s)	// 1000
#define C2 2500		// N/(m/s)	// 2500

// riddle parameters:
#define MS		102.6
#define IS		14.78
#define LSF		0.34
#define HSF		0.14
#define LSR		0.35
#define HSR		0.08
#define KX		44.966
#define KZ		57.094
#define CX		800
#define CZ		800
#define CPS		31
#define ME		32.4
#define FREQF	19
#define RE		0.007
#define WE		2*M_PI*FREQF	// exciter angular frequency
#define FE		ME*WE*WE*RE
// initial coordinates:
#define XSF0	 LSF
#define XSR0	-LSR
#define ZSF0	 HSF
#define ZSR0	 HSR

// Riddle simulation params
#define RSIM_TIME 		5 // s
#define RSIM_STEPS_NR	(RSIM_TIME / SIM_STEP)

// Data to be sent to / from simulation model:
typedef struct
{
	int data_int[2];
	double data_dbl[6];
} Params;

typedef struct
{
	gsl_odeiv2_system sys;
	gsl_odeiv2_driver* d;

	int	   cnt;		 // counter of sim steps
	double t;		 // stores current simulation time
	double t_end;	 // end of simulation step
	double dt;		 // simulation step
	double x[X_LEN]; // vector of state variables
	Params* params;	 // incoming parameters like disturbance

	float* data_vec1;
	float* data_vec2;	 // vector for disturbance data (by syspension)
	float* data_vec3;	 // vector for control
	float* t_vec;	 // time vector - can be removed; it will be 0.1, 0.2, 0.3 ...

	FILE* f;

} Simulation;

void simDataToFile(Simulation* const sim);

int suspensionModel(double t, const double y[], double dxdt[], void* params);
int riddleModel(double t, const double x[], double dxdt[], void* params);

void initSim(Simulation* const sim,
		     int (*model)(double, const double[], double[], void*),
		     int const dimension,
			 double const dt,
		     Params* const params);

int runSim(Simulation* const sim);

void deleteSim(Simulation* const sim);

#endif /* SIMULATION_H_ */
