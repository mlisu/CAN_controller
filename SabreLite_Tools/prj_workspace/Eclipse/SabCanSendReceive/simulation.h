#ifndef SIMULATION_H_
#define SIMULATION_H_

/*
  Assumed max simulation time is 300s.
  Assumed inertia sampling interval is 30ms.
  Therefore max number of samples is 300s/30ms = 10000 + 1 (+1 for zero sample).
  This amount is defined by the SIM_DATA_VEC_LEN_MAX constant.
  Assumed sample data type is "double", so 10001*8 = 80008 bytes for data buffer
  is needed.
  Apart from the SIM_DATA_VEC_LEN_MAX, there is real simulation samples number
  defined, which equals to STIME / T + 1, (see descriptions of macros; +1 for the
  zero sample) and is defined by the  SIM_DATA_VEC_LEN macro.
  Introducing both SIM_DATA_VEC_LEN_MAX and SIM_DATA_VEC_LEN is for defining
  fixed array size with SIM_DATA_VEC_LEN_MAX.
  A buffer for data samples converted to chars of format "123.123\n" (8 bytes) is being used
  to limit file i/o operations. Because of the char data format (8 bytes),
  the buffer lengths equals to the size of raw data buffer (80008) + 1 = 80009
  (+1 for the terminating \0). This buffer length is defined by the
  FILE_BUF_LEN macro.
  The 2 buffers consume 160 kB of memory. Any increasing should be done upon consideration
  of the stack size (commonly 1 or 2 MB, but should be checked) also taking in consideration
  proper reserve for another data.

  There could be more data write ops and bufs overwriting during runtime, which allows longer sim time.
*/
#include <stdio.h>
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>

// simulation parameters:
#define STIME 					5 //s simulation time, assumed MAX is 50s
#define SIM_DATA_VEC_LEN		((int)(STIME/SIM_STEP + 0.5) + 1) /* assuming STIME/T = 15,78, then we have 16 data point
															  + 1 for zero data point. Keep some reserve between MAX
															  and REAL in case of timers inaccuracy
													 	   */
#define SIM_DATA_VEC_LEN_MAX	50000
//#define FILE_BUF_LEN     		80009
#define FILE_BUF_LEN     		1 // remove or adapt
#define OUT_FILE_NAME			"system_response.csv"

#define X_LEN_I					1		// vector of state variables length for inertia
#define X_LEN_S					4		// for suspension
#define X_LEN					X_LEN_S
#define PARAM_LEN				2		// parameter vector length
#define OUT_IDX					0		// index of the observed state variable in the state vector (system output)
										// 0 for inertia, 2 for suspension
#define U_IDX					2		// index of disturbance value in params
#define UF_IDX					1		// index of disturbance freq in params
#define IN_IDX					0		// index of system input in params
#define SIM_STEP				0.010 	// 10 ms
#define SIN_W					3.14/4 	// rad/s - sinus disturbance angular frequency

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

typedef struct FileHandler_
{
	FILE* f;
	char buf[FILE_BUF_LEN];

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
	float* t_vec;	 // time vector
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
